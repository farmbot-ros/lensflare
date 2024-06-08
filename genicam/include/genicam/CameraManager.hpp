#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <harvester_interfaces/srv/create_camera.hpp>
#include <harvester_interfaces/msg/camera_device.hpp>
#include <harvester_interfaces/msg/camera_device_array.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <genicam/CameraSets.hpp>

#include <ArenaApi.h>



class CameraManager : public rclcpp::Node {
    private:
        typedef std::pair<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> cstate;
        typedef std::pair<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> cchange;
        
        harvester_interfaces::msg::CameraDeviceArray::SharedPtr camera_infos;
        rclcpp::Subscription<harvester_interfaces::msg::CameraDeviceArray>::SharedPtr camera_info_sub;
        rclcpp::TimerBase::SharedPtr camera_check_timer;
        rclcpp::Client<harvester_interfaces::srv::CreateCamera>::SharedPtr camera_get;

        std::vector<cstate> camera_get_state;
        std::vector<cchange> camera_change_state;

        bool has_caminfos;

    public:
        CameraManager();
        ~CameraManager();
        unsigned int get_state(cstate camera, std::chrono::seconds timeout);
        bool change_state(cchange camera, unsigned int state, std::chrono::seconds timeout);

    private:
        void cam_info_update(const harvester_interfaces::msg::CameraDeviceArray::SharedPtr msg);
        void cam_check_update();
};