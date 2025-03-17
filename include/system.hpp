#include <GenApi/GenApi.h>
#include <cstdlib> // for std::getenv
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>

// Forward declarations for external functions and classes used here.
std::vector<std::string> getAvailableGenTLs(const char *path);
std::shared_ptr<GenApi::CNodeMapRef> allocNodeMap(const std::shared_ptr<const class GenTLWrapper> &gentl, void *tl,
                                                  class CPort *cport);
#define GENTL_INSTALL_PATH "/opt/genicam/genTL" // or your default path

// Forward declarations for classes used below.
namespace gic {
    class GenTLWrapper;
    class Interface;
    class CPort;
    class GenTLException; // assumed to be defined elsewhere

    class System : public std::enable_shared_from_this<System> {
      public:
        // Destructor
        ~System() {
            if (n_open > 0 && tl != nullptr) {
                gentl->TLClose(tl);
            }
            gentl->GCCloseLib();
        }

        static bool setSystemsPath(const char *path, const char *ignore) {
            std::lock_guard<std::recursive_mutex> lock(system_mtx);
            if (system_list.empty()) {
                system_path = "";
                system_ignore = "";
                if (path == nullptr || path[0] == '\0') {
                    system_path = GENTL_INSTALL_PATH;
                } else {
                    system_path = path;
                }
                if (ignore != nullptr) {
                    system_ignore = ignore;
                }
                return true;
            }
            return false;
        }

        static std::vector<std::shared_ptr<System>> getSystems() {
            std::lock_guard<std::recursive_mutex> lock(system_mtx);
            std::vector<std::shared_ptr<System>> ret;
            // Ensure that system_path is defined.
            if (system_path.empty()) {
                const char *env = (sizeof(size_t) == 8) ? "GENICAM_GENTL64_PATH" : "GENICAM_GENTL32_PATH";
                setSystemsPath(std::getenv(env), nullptr);
            }
            // Get list of all available transport layer libraries.
            std::vector<std::string> names = getAvailableGenTLs(system_path.c_str());
            std::ostringstream info;

            for (size_t i = 0; i < names.size(); i++) {
                int k = find(system_list, names[i]);
                if (!system_ignore.empty()) {
                    // Skip if the name equals the ignore string.
                    if (names[i].size() >= system_ignore.size() &&
                        names[i].substr(names[i].size() - system_ignore.size()) == system_ignore) {
                        continue;
                    }
                }
                if (k >= 0) {
                    ret.push_back(system_list[static_cast<size_t>(k)]);
                } else {
                    try {
                        System *p = new System(names[i]);
                        ret.push_back(std::shared_ptr<System>(p));
                    } catch (const std::exception &ex) {
                        // Record failure in the info stream (or handle as needed).
                        info << ex.what() << std::endl;
                    }
                }
            }

            system_list = ret;
            if (ret.empty()) {
                info << "No transport layers found in path: " << system_path;
                throw GenTLException(info.str());
            }
            return ret;
        }

        static void clearSystems() {
            std::lock_guard<std::recursive_mutex> lock(system_mtx);
            for (size_t i = 0; i < system_list.size(); i++) {
                system_list[i]->clearInterfaces();
            }
            system_list.clear();
        }

        // Public member functions

        const std::string &getFilename() const { return filename; }

        void open() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            if (n_open == 0) {
                if (gentl->TLOpen(&tl) != GenTL::GC_ERR_SUCCESS) {
                    throw GenTLException("System::open()", gentl);
                }
            }
            n_open++;
        }

        void close() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            if (n_open > 0) {
                n_open--;
                if (n_open == 0) {
                    gentl->TLClose(tl);
                    tl = nullptr;
                    nodemap = nullptr;
                    cport = nullptr;
                }
            }
        }

        std::vector<std::shared_ptr<Interface>> getInterfaces() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            std::vector<std::shared_ptr<Interface>> ret = ilist;
            // ENUM-WORKAROUND: Enumerate interfaces only once.
            if (tl != nullptr && ilist.empty()) {
                if (gentl->TLUpdateInterfaceList(tl, 0, 10) != GenTL::GC_ERR_SUCCESS) {
                    throw GenTLException("System::getInterfaces()", gentl);
                }
                uint32_t n = 0;
                if (gentl->TLGetNumInterfaces(tl, &n) != GenTL::GC_ERR_SUCCESS) {
                    throw GenTLException("System::getInterfaces()", gentl);
                }
                for (uint32_t i = 0; i < n; i++) {
                    char tmp[256] = "";
                    size_t size = sizeof(tmp);
                    if (gentl->TLGetInterfaceID(tl, i, tmp, &size) != GenTL::GC_ERR_SUCCESS) {
                        throw GenTLException("System::getInterfaces()", gentl);
                    }
                    int k = find(ret, tmp);
                    if (k >= 0) {
                        ret.push_back(ret[static_cast<size_t>(k)]);
                    } else {
                        ret.push_back(std::shared_ptr<Interface>(new Interface(shared_from_this(), gentl, tmp)));
                    }
                }
                ilist.clear();
                for (size_t i = 0; i < ret.size(); i++) {
                    ilist.push_back(ret[i]);
                }
                for (size_t i = 0; i < ret.size(); i++) {
                    ret[i]->open();
                }
            }
            return ret;
        }

        std::string getID() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return cTLGetInfo(tl, gentl, GenTL::TL_INFO_ID);
        }

        std::string getVendor() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return cTLGetInfo(tl, gentl, GenTL::TL_INFO_VENDOR);
        }

        std::string getModel() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return cTLGetInfo(tl, gentl, GenTL::TL_INFO_MODEL);
        }

        std::string getVersion() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return cTLGetInfo(tl, gentl, GenTL::TL_INFO_VERSION);
        }

        std::string getTLType() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return cTLGetInfo(tl, gentl, GenTL::TL_INFO_TLTYPE);
        }

        std::string getName() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return cTLGetInfo(tl, gentl, GenTL::TL_INFO_NAME);
        }

        std::string getPathname() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return cTLGetInfo(tl, gentl, GenTL::TL_INFO_PATHNAME);
        }

        std::string getDisplayName() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return cTLGetInfo(tl, gentl, GenTL::TL_INFO_DISPLAYNAME);
        }

        bool isCharEncodingASCII() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            bool ret = true;
            GenTL::INFO_DATATYPE type;
            int32_t v;
            size_t size = sizeof(v);
            GenTL::GC_ERROR err = GenTL::GC_ERR_SUCCESS;
            if (tl != nullptr) {
                err = gentl->TLGetInfo(tl, GenTL::TL_INFO_CHAR_ENCODING, &type, &v, &size);
            } else {
                err = gentl->GCGetInfo(GenTL::TL_INFO_CHAR_ENCODING, &type, &v, &size);
            }
            if (err == GenTL::GC_ERR_SUCCESS && type == GenTL::INFO_DATATYPE_INT32 &&
                v != GenTL::TL_CHAR_ENCODING_ASCII) {
                ret = false;
            }
            return ret;
        }

        int getMajorVersion() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            uint32_t ret = 0;
            GenTL::INFO_DATATYPE type;
            size_t size = sizeof(ret);
            if (tl != nullptr) {
                gentl->TLGetInfo(tl, GenTL::TL_INFO_GENTL_VER_MAJOR, &type, &ret, &size);
            } else {
                gentl->GCGetInfo(GenTL::TL_INFO_GENTL_VER_MAJOR, &type, &ret, &size);
            }
            return static_cast<int>(ret);
        }

        int getMinorVersion() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            uint32_t ret = 0;
            GenTL::INFO_DATATYPE type;
            size_t size = sizeof(ret);
            if (tl != nullptr) {
                gentl->TLGetInfo(tl, GenTL::TL_INFO_GENTL_VER_MINOR, &type, &ret, &size);
            } else {
                gentl->GCGetInfo(GenTL::TL_INFO_GENTL_VER_MINOR, &type, &ret, &size);
            }
            return static_cast<int>(ret);
        }

        std::shared_ptr<GenApi::CNodeMapRef> getNodeMap() {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            if (tl != nullptr && !nodemap) {
                cport = std::shared_ptr<CPort>(new CPort(gentl, &tl));
                nodemap = allocNodeMap(gentl, tl, cport.get());
            }
            return nodemap;
        }

        void *getHandle() const { return tl; }

      private:
        // Helper functions as private static inline methods

        static int find(const std::vector<std::shared_ptr<System>> &list, const std::string &filename) {
            for (size_t i = 0; i < list.size(); i++) {
                if (list[i]->getFilename() == filename) return static_cast<int>(i);
            }
            return -1;
        }

        static int find(const std::vector<std::shared_ptr<Interface>> &list, const std::string &id) {
            for (size_t i = 0; i < list.size(); i++) {
                if (list[i]->getID() == id) return static_cast<int>(i);
            }
            return -1;
        }

        static std::string cTLGetInfo(GenTL::TL_HANDLE tl, const std::shared_ptr<const GenTLWrapper> &gentl,
                                      GenTL::TL_INFO_CMD info) {
            std::string ret;
            GenTL::INFO_DATATYPE type;
            char tmp[1024] = "";
            size_t tmp_size = sizeof(tmp);
            GenTL::GC_ERROR err = GenTL::GC_ERR_SUCCESS;
            if (tl != nullptr) {
                err = gentl->TLGetInfo(tl, info, &type, tmp, &tmp_size);
            } else {
                err = gentl->GCGetInfo(info, &type, tmp, &tmp_size);
            }
            if (err == GenTL::GC_ERR_SUCCESS && type == GenTL::INFO_DATATYPE_STRING) {
                for (size_t i = 0; i < tmp_size && tmp[i] != '\0'; i++) {
                    ret.push_back(tmp[i]);
                }
            }
            return ret;
        }

        // Clear all interfaces.
        void clearInterfaces() {
            for (size_t i = 0; i < ilist.size(); i++) {
                ilist[i]->close();
            }
            ilist.clear();
        }

        // Private constructor (only accessible via getSystems())
        System(const std::string &_filename) : filename(_filename), n_open(0), tl(nullptr) {
            gentl = std::shared_ptr<const GenTLWrapper>(new GenTLWrapper(filename));
            if (gentl->GCInitLib() != GenTL::GC_ERR_SUCCESS) {
                throw GenTLException("System::System()", gentl);
            }
        }
        System(System &) = delete;
        System &operator=(const System &) = delete;

        // Private member variables
        std::string filename;
        std::shared_ptr<const GenTLWrapper> gentl;
        std::recursive_mutex mtx;
        int n_open;
        void *tl;
        std::shared_ptr<CPort> cport;
        std::shared_ptr<GenApi::CNodeMapRef> nodemap;
        std::vector<std::shared_ptr<Interface>> ilist;

        // Static variables moved from the anonymous namespace
        inline static std::recursive_mutex system_mtx;
        inline static std::vector<std::shared_ptr<System>> system_list;
        inline static std::string system_path;
        inline static std::string system_ignore;
    };

} // namespace gic
