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

CameraManager::CameraManager() : Node("camera_manager") {
    try {
        pSystem = Arena::OpenSystem();
    } catch(const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", ge.what());
    }
    camera_get = this->create_client<harvester_interfaces::srv::CreateCamera>("caminfo");
    init_cameras();
    check_cameras();
}

CameraManager::~CameraManager() {
    Arena::CloseSystem(pSystem);
}

void CameraManager::init_cameras() {
    for (auto device : camset::by_mac) {
        std::string camera_name = device.second.name;
        uint64_t mac_address = device.first;
        auto camera_node = std::make_shared<CameraNode>(pSystem, camera_name, mac_address, false);
        RCLCPP_INFO(this->get_logger(), "Camera %s initialized", camera_name.c_str());
        camera_nodes.push_back(camera_node);
    }
}

void CameraManager::check_cameras() {
    for (auto camera_node : camera_nodes) {
        auto request = std::make_shared<harvester_interfaces::srv::CreateCamera::Request>();
        request->camera_name = camera_node->name;


        if (!camera_get->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available after waiting");
            return;
        }

        auto result = camera_get->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Camera %s is available", camera_node->name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Camera %s is NOT", camera_node->name.c_str());
            }
        }
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