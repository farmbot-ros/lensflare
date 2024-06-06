#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <std_msgs/msg/int16.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <genicam/CameraManager.hpp>
#include <genicam/CameraNode.hpp>
#include <genicam/CameraSets.hpp>


#include <ArenaApi.h>
#include <GenICam.h>

uint64_t convert_mac(std::string mac) {
  mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
  return strtoul(mac.c_str(), NULL, 16);
}

CameraManager::CameraManager() : Node("camera_manager") {
    try {
        pSystem = Arena::OpenSystem();
    } catch(const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", ge.what());
    }

    get_cameras();
    run_cameras();
}

CameraManager::~CameraManager() {
    for (auto& pDevice : vDevices) {
        pSystem->DestroyDevice(pDevice.first);
    }

    for (auto& camera_node : camera_nodes) {
        executor.remove_node(camera_node->get_node_base_interface());
    }
    Arena::CloseSystem(pSystem);
}

void CameraManager::get_cameras() {
    vDevices = std::vector<std::pair<Arena::IDevice*, uint64_t>>();
    try {
        pSystem->UpdateDevices(5000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
            Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfo);
            uint64_t mac = convert_mac(deviceInfo.MacAddressStr().c_str());
            if (camset::by_mac.find(mac) != camset::by_mac.end()) {
                vDevices.push_back(std::make_pair(pDevice, mac));
            }
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", ge.what());
    }
}

int CameraManager::run_cameras() {
    if (vDevices.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "No cameras found!, exiting...");
        return 1;
    }
    
    for (auto& pDevice : vDevices) {
        std::string camera_name = camset::by_mac.at(pDevice.second).name;
        auto camera_node = std::make_shared<CameraNode>(camera_name, pDevice.second);
        camera_node->add_device(pDevice.first);
        camera_nodes.push_back(camera_node);
    }

    for (auto& camera_node : camera_nodes) {
        executor.add_node(camera_node->get_node_base_interface());
    }

    // std::thread spin_thread([&executor]() {
    //     while (rclcpp::ok()) {
    //         executor.spin();
    //     }
    // });
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;
    rclcpp::executors::SingleThreadedExecutor exe;
    auto camera_manager = std::make_shared<CameraManager>();
    exe.add_node(camera_manager);
}