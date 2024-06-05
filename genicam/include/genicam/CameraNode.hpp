#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <std_msgs/msg/int16.hpp>

#include <ArenaApi.h>

class CameraNode : public rclcpp::Node {
    private:
        using trigg = harvester_interfaces::srv::TriggerCamera;
        image_transport::Publisher save_pub_;
        image_transport::Publisher view_pub_;
        image_transport::Publisher inf_pub_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr trigger;
        rclcpp::Service<trigg>::SharedPtr service;
        std::string name;
        uint64_t mac;
        Arena::IDevice* pDevice;

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> exposure;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> gain;

    public:
        CameraNode(Arena::IDevice* const pDevice, std::string camera_name, uint64_t mac_address);
        ~CameraNode();

    private:
        void init_cameras();
        void load_settings_from_func();
        void load_settings_from_file();
        void get_image(sensor_msgs::msg::Image::SharedPtr msg_image);
        void topic_trigger(const std_msgs::msg::Int16::SharedPtr msg);
        void service_trigger(const std::shared_ptr<trigg::Request> request, std::shared_ptr<trigg::Response> response);
        void param_callback(const rclcpp::Parameter & p);
};