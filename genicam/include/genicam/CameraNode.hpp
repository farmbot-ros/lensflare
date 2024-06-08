#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <harvester_interfaces/srv/create_camera.hpp>
#include <harvester_interfaces/msg/camera_device.hpp>
#include <harvester_interfaces/msg/camera_device_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <genicam/CameraSets.hpp>

#include <ArenaApi.h>

class CameraNode : public rclcpp_lifecycle::LifecycleNode {
    private:
        using lni = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
        
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr save_lpub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr save_pub_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr view_lpub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr view_pub_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr inf_lpub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr inf_pub_;
        
        rclcpp::Service<harvester_interfaces::srv::TriggerCamera>::SharedPtr service;

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> exposure;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> gain;

        Arena::ISystem* pSystem;
        bool has_system;
        Arena::IDevice* pDevice;
        bool has_device;

    public:
        std::string name;
        uint64_t mac;
        bool lifecycled;

        CameraNode(std::string camera_name, uint64_t mac_address, bool init);
        CameraNode(Arena::IDevice* const pDevice, std::string camera_name, uint64_t mac_address, bool init);
        CameraNode(Arena::ISystem* const pSystem, std::string camera_name, uint64_t mac_address, bool init);
        void add_device(Arena::IDevice* const pDevice, bool start_streaming);
        void add_system(Arena::ISystem* const pSystem, bool start_streaming);
        ~CameraNode();

    private:
        void config_node(bool managed);
        void load_camera_settings();
        void load_settings_from_func();
        void load_settings_from_file();
        sensor_msgs::msg::Image::SharedPtr get_image(int trigger_type);
        void service_trigger(
            const std::shared_ptr<harvester_interfaces::srv::TriggerCamera::Request> request, 
            std::shared_ptr<harvester_interfaces::srv::TriggerCamera::Response> response);
        void param_callback(const rclcpp::Parameter & p);

        lni::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
        lni::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
        // lni::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
        // lni::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
        // lni::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
};