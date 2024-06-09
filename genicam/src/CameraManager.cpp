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

        auto device = harvester_interfaces::msg::CameraDevice();
        device.id = cam.second.name;
        device.mac = std::to_string(cam.first);
        camera_devices.push_back(device);
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
    for (auto camera_node : camera_devices) {
        while (!camera_get->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available, please launch CameraInfo node");
        }

        auto request = std::make_shared<harvester_interfaces::srv::CreateCamera::Request>();
        request->camera_name = camera_node.name;

        auto result = camera_get->async_send_request(request, [this](rclcpp::Client<harvester_interfaces::srv::CreateCamera>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Camera %s is available", response->camera_device.name);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        });

        // auto future_result = camera_get->async_send_request(request);
        // auto future_status = wait_for_result(future_result, std::chrono::seconds(2));

        // if (future_status == std::future_status::ready) {
        //     auto response = future_result.get();
        //     RCLCPP_INFO(this->get_logger(), "Camera %s is available", response->camera_device.name.c_str());
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Service call failed");
        // }



        // for (auto camera : camera_get_state) {
        //     unsigned int state = get_state(camera, std::chrono::seconds(2));
        //     if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        //         RCLCPP_INFO(this->get_logger(), "Camera %s is ACTIVE", camera_node.name.c_str());
        //     } else {
        //         RCLCPP_ERROR(this->get_logger(), "Camera %s is NOT ACTIVE", camera_node.name.c_str());
        //     }
        // }

        
    }
    RCLCPP_INFO(this->get_logger(), "Camera info updated");
}

template <typename FutureT, typename DurationT>
std::future_status CameraManager::wait_for_result(FutureT& future, DurationT timeout) {
    auto end = std::chrono::steady_clock::now() + timeout;
    std::chrono::milliseconds wait_period(1000);
    std::future_status status = std::future_status::timeout;

    do {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0)) {
            break;
        }
        status = future.wait_for(time_left< wait_period ? time_left : wait_period);
    } while ( rclcpp::ok() && status != std::future_status::ready);

    return status;
}

unsigned int CameraManager::get_state(cstate camera, std::chrono::seconds timeout) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto client_state = camera.second;
    while (!camera.second->wait_for_service(timeout)) {
        auto service_name = client_state->get_service_name();
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service %s not available, please launch CameraInfo node", service_name);
    }


    auto future_result = camera.second->async_send_request(request);
    auto future_stat = this->wait_for_result(future_result, timeout);
    
    // auto future_status = wait_for_result(future_result, timeout);

    // if (future_status == std::future_status::ready) {
    //     return future_result.get()->current_state.id;
    //     RCLCPP_INFO(this->get_logger(), "Camera %s is ACTIVE", camera.first.c_str());
    // } else {
    //     return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    // }
}

// bool CameraManager::change_state(cchange camera, unsigned int state, std::chrono::seconds timeout) {
//     auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
//     request->transition.id = state;
//     while (!camera.second->wait_for_service(timeout)) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available, please launch CameraInfo node");
//     }
//     auto result = camera.second->async_send_request(request);
//     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
//         return result.get()->success;
//     } else {
//         return false;
//     }
// }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;
    rclcpp::executors::SingleThreadedExecutor exe;
    auto camera_manager = std::make_shared<CameraManager>();
    exe.add_node(camera_manager);
    exe.spin();
    rclcpp::shutdown();
}