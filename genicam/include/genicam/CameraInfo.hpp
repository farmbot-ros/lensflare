#pragma once

#include <rclcpp/rclcpp.hpp>
#include <genicam/msg/camera_device.hpp>
#include <genicam/msg/camera_device_array.hpp>
#include <genicam/srv/create_camera.hpp>


#include <ArenaApi.h>


class CameraArray : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr publish_array_info_timer_;
    rclcpp::Publisher<genicam::msg::CameraDeviceArray>::SharedPtr array_pub_;
    genicam::msg::CameraDeviceArray::SharedPtr camera_info;
    rclcpp::Service<genicam::srv::CreateCamera>::SharedPtr service;
    using camc = genicam::srv::CreateCamera;
    public:
        CameraArray(genicam::msg::CameraDeviceArray::SharedPtr camera_info);
    private:
        void publish_camera_info();
        void service_trigger(const std::shared_ptr<camc::Request> request, std::shared_ptr<camc::Response> response);

};

class CameraInfo : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr update_camera_info_timer_;
    genicam::msg::CameraDeviceArray::SharedPtr camera_info;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_int;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_str;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_float;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_bool;
    
    public:
        CameraInfo(genicam::msg::CameraDeviceArray::SharedPtr camera_info);
    private:
        void update_camera_info();
        void param_callback(const rclcpp::Parameter & p);

};