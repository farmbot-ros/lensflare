#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <std_msgs/msg/int16.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <genicam/CameraManager.hpp>
#include <genicam/CameraSets.hpp>


#include <ArenaApi.h>
#include <GenICam.h>

typedef std::pair<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> cstate;
typedef std::pair<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> cchange;


CameraManager::CameraManager() : Node("camera_manager") {
    camera_info_sub = this->create_subscription<harvester_interfaces::msg::CameraDeviceArray>("/caminfo", 10, std::bind(&CameraManager::cam_info_update, this, std::placeholders::_1));
    camera_check_timer = this->create_wall_timer(std::chrono::seconds(5), std::bind(&CameraManager::cam_check_update, this));
    camera_get = this->create_client<harvester_interfaces::srv::CreateCamera>("caminfo");

    for (auto& cam : camset::by_mac) {
        auto cam_topic_name = "camera_" + cam.second.name;
        camera_get_state.push_back(cstate(cam_topic_name, this->create_client<lifecycle_msgs::srv::GetState>(cam_topic_name + "/get_state")));
        camera_change_state.push_back(cchange(cam_topic_name, this->create_client<lifecycle_msgs::srv::ChangeState>(cam_topic_name + "/change_state")));

    }
}

CameraManager::~CameraManager() {
}


void CameraManager::cam_info_update(const harvester_interfaces::msg::CameraDeviceArray::SharedPtr msg) {
    camera_infos = msg;
    has_caminfos = true;
}

void CameraManager::cam_check_update() {
    if (!has_caminfos) {
        RCLCPP_ERROR(this->get_logger(), "No camera info available");
        return;
    }
    for (auto camera_node : camera_infos->cameras) {


        // while (!camera_get->wait_for_service(std::chrono::seconds(2))) {
        //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available, please launch CameraInfo node");
        // }

        // auto request = std::make_shared<harvester_interfaces::srv::CreateCamera::Request>();
        // request->camera_name = camera_node.name;
        // auto result = camera_get->async_send_request(request);

        // auto shared_future = result.share();
        // auto future_result = shared_future.wait_for(std::chrono::seconds(0));

        // if (future_result == std::future_status::ready) {
        //     RCLCPP_INFO(this->get_logger(), "Camera %s is available", camera_node.name.c_str());
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Camera %s is NOT", camera_node.name.c_str());
        // }



        for (auto camera : camera_get_state) {
            unsigned int state = get_state(camera, std::chrono::seconds(2));
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                RCLCPP_INFO(this->get_logger(), "Camera %s is ACTIVE", camera_node.name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Camera %s is NOT ACTIVE", camera_node.name.c_str());
            }
        }

        
    }
    RCLCPP_INFO(this->get_logger(), "Camera info updated");
}

unsigned int CameraManager::get_state(cstate camera, std::chrono::seconds timeout) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    while (!camera.second->wait_for_service(timeout)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available, please launch CameraInfo node");
    }
    auto result = camera.second->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        return result.get()->current_state.id;
    } else {
        return 0;
    }
}

bool CameraManager::change_state(cchange camera, unsigned int state, std::chrono::seconds timeout) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = state;
    while (!camera.second->wait_for_service(timeout)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available, please launch CameraInfo node");
    }
    auto result = camera.second->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        return result.get()->success;
    } else {
        return false;
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