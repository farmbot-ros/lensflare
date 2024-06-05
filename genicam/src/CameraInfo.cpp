#include <rclcpp/rclcpp.hpp>
#include <harvester_interfaces/msg/camera_device.hpp>
#include <harvester_interfaces/msg/camera_device_array.hpp>

#include <ArenaApi.h>
#include <genicam/CameraSets.hpp>

#include <genicam/CameraInfo.hpp>


uint64_t convert_mac(std::string mac) {
    mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
    return strtoul(mac.c_str(), NULL, 16);
}

CameraArray::CameraArray(harvester_interfaces::msg::CameraDeviceArray::SharedPtr camera_info) : Node("camera_array"), camera_info (camera_info) {
    publish_array_info_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&CameraArray::publish_camera_info, this));
    array_pub_ = this->create_publisher<harvester_interfaces::msg::CameraDeviceArray>("/caminfo", 10);
    service = this->create_service<camc>("caminfo", std::bind(&CameraArray::service_trigger, this, std::placeholders::_1, std::placeholders::_2));
}

void CameraArray::service_trigger(const std::shared_ptr<camc::Request> request, std::shared_ptr<camc::Response> response) {
    if (camera_info->cameras.empty()) {
        response->success = false;
        response->message = "No cameras found";
        return;
    }
    std::string camera_name = request->camera_name;
    std::string mac_address = request->mac_address;
    harvester_interfaces::msg::CameraDevice camera_device = harvester_interfaces::msg::CameraDevice();
    if (!mac_address.empty()) {
        uint64_t hex_mac = convert_mac(mac_address);
        for (const auto &camera : camera_info->cameras) {
            if (convert_mac(camera.mac) == hex_mac) {
                camera_device = camera;
                response->success = true;
                break;
            }
        }
    } else if (!camera_name.empty()) {
        for (const auto &camera : camera_info->cameras) {
            if (camera.name == camera_name) {
                camera_device = camera;
                response->success = true;
                break;
            }
        }
    } else {
        response->success = false;
        response->message = "Missing camera name or mac address";
    }
}

void CameraArray::publish_camera_info() {
    if (camera_info->cameras.empty()) {
        return;
    }
    array_pub_->publish(*camera_info);
    std::vector<std::string> ids;
    for (const auto &camera : camera_info->cameras) {
        ids.push_back(camera.id);
    }
}


CameraInfo::CameraInfo(harvester_interfaces::msg::CameraDeviceArray::SharedPtr camera_info) : Node("camera_info"), camera_info (camera_info) {
    update_camera_info_timer_ = this->create_wall_timer(std::chrono::minutes(1), std::bind(&CameraInfo::update_camera_info, this));

    this->declare_parameter("param_int", 0);
    this->declare_parameter("param_float", 0.0);
    this->declare_parameter("param_bool", true);
    this->declare_parameter("param_str", "default");

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    param_int = param_subscriber_->add_parameter_callback("param_int", std::bind(&CameraInfo::param_callback, this, std::placeholders::_1));
    param_str = param_subscriber_->add_parameter_callback("param_float", std::bind(&CameraInfo::param_callback, this, std::placeholders::_1));
    param_float = param_subscriber_->add_parameter_callback("param_bool", std::bind(&CameraInfo::param_callback, this, std::placeholders::_1));
    param_bool = param_subscriber_->add_parameter_callback("param_str", std::bind(&CameraInfo::param_callback, this, std::placeholders::_1));

    update_camera_info();
}

void CameraInfo::param_callback(const rclcpp::Parameter & p) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Param type: %s", p.get_type_name().c_str());
    if (p.get_type_name() == "string") {
        harvester_interfaces::msg::CameraDevice camera_device = harvester_interfaces::msg::CameraDevice();
        camera_device.id = p.get_parameter_value().get<std::string>();
        camera_info->cameras.push_back(camera_device);
        auto val = p.get_parameter_value().get<std::string>();
    } else if (p.get_type_name() == "integer") {
        auto val = p.get_parameter_value().get<int>();
    } else if (p.get_type_name() == "double") {
        auto val = p.get_parameter_value().get<float>();
    } else if (p.get_type_name() == "bool") {
        auto val = p.get_parameter_value().get<bool>();
    }
}


void CameraInfo::update_camera_info() {
    Arena::ISystem* pSystem = nullptr;
    std::vector<std::pair<Arena::IDevice*, uint64_t>> vDevices = std::vector<std::pair<Arena::IDevice*, uint64_t>>();
    harvester_interfaces::msg::CameraDeviceArray camera_array = harvester_interfaces::msg::CameraDeviceArray();

    try {
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(1000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
            harvester_interfaces::msg::CameraDevice camera_device = harvester_interfaces::msg::CameraDevice();
            camera_device.model = deviceInfo.ModelName();
            camera_device.vendor = deviceInfo.VendorName();
            camera_device.serial = deviceInfo.SerialNumber();
            camera_device.ip = deviceInfo.IpAddressStr();
            camera_device.subnetmask = deviceInfo.SubnetMaskStr();
            camera_device.defaultgateway = deviceInfo.DefaultGatewayStr();
            camera_device.mac = deviceInfo.MacAddressStr();
            camera_device.name = deviceInfo.UserDefinedName();
            camera_device.dhcp = deviceInfo.IsDHCPConfigurationEnabled();
            camera_device.presistentip = deviceInfo.IsPersistentIpConfigurationEnabled();
            camera_device.lla = deviceInfo.IsLLAConfigurationEnabled();

            uint64_t hex_mac = convert_mac(deviceInfo.MacAddressStr().c_str());
            if (camset::by_mac.find(hex_mac) != camset::by_mac.end()) {
                camera_device.light_ip = camset::by_mac.at(hex_mac).ip;
                camera_device.light_port = "6512";
                camera_device.light_channel = camset::by_mac.at(hex_mac).channel;
                camera_device.id = camset::by_mac.at(hex_mac).name;
            }
            camera_array.cameras.push_back(camera_device);   
        }
        Arena::CloseSystem(pSystem);
        *camera_info = camera_array;

    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", ge.what());
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;

    rclcpp::executors::MultiThreadedExecutor executor;
    harvester_interfaces::msg::CameraDeviceArray::SharedPtr camera_info = std::make_shared<harvester_interfaces::msg::CameraDeviceArray>();

    auto camera_info_node = std::make_shared<CameraInfo>(camera_info);
    auto camera_array_node = std::make_shared<CameraArray>(camera_info);

    executor.add_node(camera_info_node);
    executor.add_node(camera_array_node);

    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "An error occurred: %s", e.what());
    }

    executor.remove_node(camera_info_node);
    executor.remove_node(camera_array_node);

    rclcpp::shutdown();
    return 0;
}