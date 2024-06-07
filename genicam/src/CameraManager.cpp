#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <std_msgs/msg/int16.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <genicam/CameraManager.hpp>
#include <genicam/CameraNode.hpp>
#include <genicam/CameraSets.hpp>


#include <ArenaApi.h>
#include <GenICam.h>

uint64_t convert_mac(std::string mac) {
  mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
  return strtoul(mac.c_str(), NULL, 16);
}

CameraManager::CameraManager() : Node("camera_manager") {
    try {
        pSystem = Arena::OpenSystem();
    } catch(const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", ge.what());
    }
}

CameraManager::~CameraManager() {
    Arena::CloseSystem(pSystem);
}

void CameraManager::init_cameras() {
    for (auto device : camset::by_mac) {
        std::string camera_name = device.second.name;
        uint64_t mac_address = device.first;
        auto camera_node = std::make_shared<CameraNode>(pSystem, camera_name, mac_address, false);
        camera_nodes.push_back(camera_node);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;
    rclcpp::executors::SingleThreadedExecutor exe;
    auto camera_manager = std::make_shared<CameraManager>();
    exe.add_node(camera_manager);
    exe.spin();
    rclcpp::shutdown();
}