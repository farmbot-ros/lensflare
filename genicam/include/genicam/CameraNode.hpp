#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ArenaApi.h>

class CameraNode : public rclcpp_lifecycle::LifecycleNode {
    private:
        using trigg = harvester_interfaces::srv::TriggerCamera;
        using lni = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

        // image_transport::Publisher save_pub_;
        // image_transport::Publisher view_pub_;
        // image_transport::Publisher inf_pub_;
        
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr save_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr view_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr inf_pub_;

        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr trigger;
        rclcpp::Service<trigg>::SharedPtr service;

        std::string name;
        uint64_t mac;
        Arena::IDevice* pDevice;

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> exposure;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> gain;

    public:
        CameraNode(Arena::IDevice* const pDevice, std::string camera_name, uint64_t mac_address, bool incom);
        ~CameraNode();

    private:
        void init_cameras();
        void load_settings_from_func();
        void load_settings_from_file();
        sensor_msgs::msg::Image::SharedPtr get_image(int trigger_type);
        void topic_trigger(const std_msgs::msg::Int16::SharedPtr msg);
        void service_trigger(const std::shared_ptr<trigg::Request> request, std::shared_ptr<trigg::Response> response);
        void param_callback(const rclcpp::Parameter & p);

        // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
        // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
        // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
        // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
        // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
};