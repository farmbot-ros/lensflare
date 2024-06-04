#include <rclcpp/rclcpp.hpp>
#include <harvester_interfaces/msg/camera_device.hpp>
#include <harvester_interfaces/msg/camera_device_array.hpp>

#include <ArenaApi.h>
#include <genicam/CameraSets.hpp>

#include <genicam/CameraInfo.hpp>



CameraArray::CameraArray(harvester_interfaces::msg::CameraDeviceArray::SharedPtr camera_info) : Node("camera_array"), camera_info (camera_info) {
    publish_array_info_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&CameraArray::publishArrayInfo, this));
    array_pub_ = this->create_publisher<harvester_interfaces::msg::CameraDeviceArray>("/caminfo", 10);
}


void CameraArray::publishArrayInfo() {
    // if (camera_info->cameras.empty()) {
    //     return;
    // }
    array_pub_->publish(*camera_info);
    std::vector<std::string> ids;
    for (const auto &camera : camera_info->cameras) {
        ids.push_back(camera.id);
    }
}


CameraInfo::CameraInfo(harvester_interfaces::msg::CameraDeviceArray::SharedPtr camera_info) : Node("camera_info"), camera_info (camera_info) {
    update_camera_info_timer_ = this->create_wall_timer(std::chrono::minutes(1), std::bind(&CameraInfo::updateCameraInfo, this));

    this->declare_parameter("param", 0);
    this->declare_parameter("param2", "default");

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    cb_handle_ = param_subscriber_->add_parameter_callback("param", std::bind(&CameraInfo::paramCallback, this, std::placeholders::_1));
    cb_handle_ = param_subscriber_->add_parameter_callback("param2", std::bind(&CameraInfo::paramCallback, this, std::placeholders::_1));

    updateCameraInfo();
}

void CameraInfo::paramCallback(const rclcpp::Parameter & p) {
    RCLCPP_INFO(this->get_logger(), "Parameter \"%s\" of type %s", p.get_name().c_str(), p.get_type_name().c_str());
    // camera_info->cameras[0].id = p.as_string();
    // RCLCPP_INFO(this->get_logger(), "Id: %s", camera_info->cameras[0].id.c_str());
}

uint64_t CameraInfo::convert_mac(std::string mac) {
    mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
    return strtoul(mac.c_str(), NULL, 16);
}

void CameraInfo::updateCameraInfo() {
    Arena::ISystem* pSystem = nullptr;
    std::vector<std::pair<Arena::IDevice*, uint64_t>> vDevices = std::vector<std::pair<Arena::IDevice*, uint64_t>>();
    harvester_interfaces::msg::CameraDeviceArray camera_array;

    try {
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(1000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
            harvester_interfaces::msg::CameraDevice camera_device;
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
        camera_info = std::make_shared<harvester_interfaces::msg::CameraDeviceArray>(camera_array);

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

    // harvester_interfaces::msg::CameraDeviceArray camera_array;
    // harvester_interfaces::msg::CameraDevice camera_device;
    // camera_device.id = "1";
    // camera_array.cameras.push_back(camera_device);
    // camera_info = std::make_shared<harvester_interfaces::msg::CameraDeviceArray>(camera_array);

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