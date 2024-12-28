#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <genicam/srv/trigger_camera.hpp>
#include <genicam/srv/create_camera.hpp>
#include <genicam/msg/camera_device.hpp>
#include <genicam/msg/camera_device_array.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ArenaApi.h>



class CameraManager : public rclcpp::Node {
    private:
        typedef std::pair<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> cstate;
        typedef std::pair<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> cchange;
        
        rclcpp::Subscription<genicam::msg::CameraDeviceArray>::SharedPtr camera_info_sub;
        rclcpp::TimerBase::SharedPtr camera_check_timer;
        rclcpp::Client<genicam::srv::CreateCamera>::SharedPtr camera_get;

        genicam::msg::CameraDeviceArray::SharedPtr camera_infos;
        std::vector<genicam::msg::CameraDevice> camera_devices;

        std::map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> camera_get_state;
        std::map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> camera_change_state;

        bool has_caminfos;

    public:
        CameraManager();
        ~CameraManager();

    private:
        void cam_info_update(const genicam::msg::CameraDeviceArray::SharedPtr msg);
        void cam_check_update();
        void state_controller(const genicam::msg::CameraDevice, lifecycle_msgs::msg::State curr_state, bool runnning);
        template <typename FutureT, typename WaitTimeT>
        
        std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait);
        unsigned int get_state(genicam::msg::CameraDevice camera_node, std::chrono::seconds timeout);
        bool change_state(genicam::msg::CameraDevice camera_node, std::uint8_t transition, std::chrono::seconds timeout);
};