#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <harvester_interfaces/srv/create_camera.hpp>
#include <harvester_interfaces/msg/camera_device.hpp>
#include <harvester_interfaces/msg/camera_device_array.hpp>
#include <genicam/CameraNode.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <genicam/CameraNode.hpp>
// #include <genicam/CameraSets.hpp>

#include <ArenaApi.h>

class CameraManager : public rclcpp::Node {
    private:
        rclcpp::executors::MultiThreadedExecutor executor;
        harvester_interfaces::msg::CameraDeviceArray::SharedPtr camera_info;
        rclcpp::Client<harvester_interfaces::srv::CreateCamera>::SharedPtr camera_get;
        Arena::ISystem* pSystem = nullptr;
        std::vector<std::shared_ptr<CameraNode>> camera_nodes;
    public:
        CameraManager();
        ~CameraManager();

    private:
        void init_cameras();
        void check_cameras();
};