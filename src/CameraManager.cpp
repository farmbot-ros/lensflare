#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <genicam/srv/trigger_camera.hpp>
#include <std_msgs/msg/int16.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <genicam/CameraManager.hpp>
#include <genicam/CameraSets.hpp>


#include <ArenaApi.h>
#include <GenICam.h>

using namespace std::chrono_literals;

inline uint64_t convert_mac(std::string mac) {
    mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
    return strtoul(mac.c_str(), NULL, 16);
}

CameraManager::CameraManager() : Node("camera_manager") {
    camera_info_sub = this->create_subscription<genicam::msg::CameraDeviceArray>("/caminfo", 10, std::bind(&CameraManager::cam_info_update, this, std::placeholders::_1));
    camera_check_timer = this->create_wall_timer(std::chrono::seconds(30), std::bind(&CameraManager::cam_check_update, this));
    camera_get = this->create_client<genicam::srv::CreateCamera>("caminfo");

    for (auto& cam : camset::by_mac) {
        auto cam_topic_name = "camera_" + cam.second.name;
        camera_get_state[cam_topic_name] = this->create_client<lifecycle_msgs::srv::GetState>(cam_topic_name + "/get_state");
        camera_change_state[cam_topic_name] = this->create_client<lifecycle_msgs::srv::ChangeState>(cam_topic_name + "/change_state");

        auto device = genicam::msg::CameraDevice();
        device.id = cam.second.name;
        device.mac = std::to_string(cam.first);
        camera_devices.push_back(device);
    }
}

CameraManager::~CameraManager() {
}


void CameraManager::cam_info_update(const genicam::msg::CameraDeviceArray::SharedPtr msg) {
    camera_infos = msg;
    has_caminfos = true;
}

void CameraManager::cam_check_update() {
    if (!has_caminfos) {
        RCLCPP_ERROR(this->get_logger(), "No camera info available");
        return;
    }
    for (auto camera_node : camera_devices) {
        while (!camera_get->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available, please launch CameraInfo node");
        }
        auto request = std::make_shared<genicam::srv::CreateCamera::Request>();
        request->camera_name = camera_node.id;
        auto result = camera_get->async_send_request(request, [this, camera_node](rclcpp::Client<genicam::srv::CreateCamera>::SharedFuture future) {
            try {
                auto response = future.get();
                bool success = response->success;
                auto client_state = camera_get_state["camera_" + camera_node.id];
                auto state_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
                if (!client_state->wait_for_service(std::chrono::seconds(2))) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available, please launch CameraNode node");
                }
                if (client_state->prune_requests_older_than(std::chrono::system_clock::now() - 30s) > 0) {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Prunning some old requests...");
                }
                auto state_result = client_state->async_send_request(state_request, [this, camera_node, success](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
                    try {
                        auto state_response = future.get();
                        state_controller(camera_node, state_response->current_state, success);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Service call failed");
                    }
                });
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        });
    }
    RCLCPP_INFO(this->get_logger(), "Cameras updated");
}



void CameraManager::state_controller(const genicam::msg::CameraDevice camera, lifecycle_msgs::msg::State curr_state, bool running) {
    // RCLCPP_INFO(this->get_logger(), "Camera %s is %s in current state %i", camera.id.c_str(), running ? "running" : "not running", curr_state.id);
    auto client_change = camera_change_state["camera_" + camera.id];
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    bool proceed = false;
    // State constants
    // static constexpr uint8_t PRIMARY_STATE_UNKNOWN = 0u;
    // static constexpr uint8_t PRIMARY_STATE_UNCONFIGURED = 1u;
    // static constexpr uint8_t PRIMARY_STATE_INACTIVE = 2u;
    // static constexpr uint8_t PRIMARY_STATE_ACTIVE = 3u;
    // static constexpr uint8_t PRIMARY_STATE_FINALIZED = 4u;
    // static constexpr uint8_t TRANSITION_STATE_CONFIGURING = 10u;
    // static constexpr uint8_t TRANSITION_STATE_CLEANINGUP = 11u;
    // static constexpr uint8_t TRANSITION_STATE_SHUTTINGDOWN =12u;
    // static constexpr uint8_t TRANSITION_STATE_ACTIVATING = 13u;
    // static constexpr uint8_t TRANSITION_STATE_DEACTIVATING = 14u;
    // static constexpr uint8_t TRANSITION_STATE_ERRORPROCESSING = 15u;
    if (curr_state.id == 1 && running) {
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        proceed = true;
    } else if (curr_state.id == 2 && running) {
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        proceed = true;
    } else if (curr_state.id == 3  && !running){
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        proceed = true;
    } else if (curr_state.id == 2 && !running) {
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
        proceed = true;
    }

    if (!proceed) {
        // RCLCPP_INFO(this->get_logger(), "No state change needed for camera %s", camera.id.c_str());
        return;
    }

    auto result = client_change->async_send_request(request, [this, camera](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Camera %s state changed", camera.id.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Camera %s state change failed", camera.id.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    });
}

template <typename FutureT, typename WaitTimeT>
std::future_status CameraManager::wait_for_result(FutureT & future, WaitTimeT time_to_wait){
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do{
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)){ break; }
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  }
  while(rclcpp::ok() && status != std::future_status::ready);
  return status;
}

unsigned int CameraManager::get_state(genicam::msg::CameraDevice camera_node, std::chrono::seconds timeout = 3s) {
    auto client_get_state = camera_get_state["camera_" + camera_node.id];
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state->wait_for_service(timeout)) {
        RCLCPP_ERROR(get_logger(), "Service %s not available", client_get_state->get_service_name());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto future_result = client_get_state->async_send_request(request);
    auto future_status = wait_for_result(future_result, timeout);

    if (future_status != std::future_status::ready){
        RCLCPP_ERROR(get_logger(), "Server timed out while getting current state for %s", camera_node.id.c_str());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (future_result.get()){
        auto state = future_result.get()->current_state.id;
        RCLCPP_INFO(get_logger(), "Node %s has current state %s", camera_node.id.c_str(), future_result.get()->current_state.label.c_str());
        return state;
    }
    else{
        RCLCPP_ERROR(get_logger(), "Failed to get current state for %s", camera_node.id.c_str());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
}

bool CameraManager::change_state(genicam::msg::CameraDevice camera_node, std::uint8_t transition, std::chrono::seconds timeout = 3s){
    auto client_change_state = camera_change_state["camera_" + camera_node.id];
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state->wait_for_service(timeout)){
        RCLCPP_ERROR(get_logger(), "Service %s not available", client_change_state->get_service_name());
        return false;
    }

    auto future_result = client_change_state->async_send_request(request);
    auto future_status = wait_for_result(future_result,timeout);

    if (future_status!=std::future_status::ready){
        RCLCPP_ERROR(get_logger(), "Server timed out while getting current state for %s", camera_node.id.c_str());
        return false;
    }

    if (future_result.get()->success){
        RCLCPP_INFO(get_logger(), "Transition %d successfully triggered", static_cast<unsigned int>(transition));
        return true;
    }
    else{
        RCLCPP_WARN(get_logger(), "Failed to get trigger transition %d for %s", static_cast<unsigned int>(transition), camera_node.id.c_str());
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