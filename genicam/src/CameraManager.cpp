#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <std_msgs/msg/int16.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <genicam/CameraManager.hpp>


#include <ArenaApi.h>
#include <GenICam.h>

CameraManager::CameraManager() : Node("camera_manager") {
}
CameraManager::~CameraManager() {
}


// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     std::cout << "... GETTING CAMERAS ..." << std::endl;
//     rclcpp::executors::SingleThreadedExecutor exe;
//     auto camera_manager = std::make_shared<CameraManager>();
//     exe.add_node(camera_manager);
// }


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;
    Arena::ISystem* pSystem = nullptr;
    std::vector<std::pair<Arena::IDevice*, uint64_t>> vDevices = std::vector<std::pair<Arena::IDevice*, uint64_t>>();

    try {
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(1000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
			Arena::IDevice* pDevice;
            uint64_t mac = camset::convert_mac(deviceInfo.MacAddressStr().c_str());
            if (camset::by_mac.find(mac) != camset::by_mac.end()) {
                vDevices.push_back(std::make_pair(pDevice, mac));
            }
		}
    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", ge.what());
    }

    if (vDevices.size() == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No cameras found!, exiting...");
        exit(1);
    }
            
    rclcpp::executors::MultiThreadedExecutor executor;

    std::vector<std::shared_ptr<CameraNode>> camera_nodes;
    for (auto& pDevice : vDevices) {
        std::string camera_name = camset::by_mac.at(pDevice.second).name;
        auto camera_node = std::make_shared<CameraNode>(pSystem, camera_name, pDevice.second, false);
        // camera_node->add_system(pSystem);
        // camera_node->add_device(pDevice.first);
        camera_nodes.push_back(camera_node);
    }

    for (auto& camera_node : camera_nodes) {
        executor.add_node(camera_node->get_node_base_interface());
    }

    try {
        executor.spin();
    } catch (const std::exception &e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
    }

    for (auto& pDevice : vDevices) {
        pSystem->DestroyDevice(pDevice.first);
    }

    for (auto& camera_node : camera_nodes) {
        executor.remove_node(camera_node->get_node_base_interface());
    }

    Arena::CloseSystem(pSystem);
    rclcpp::shutdown();
    return 0;
}