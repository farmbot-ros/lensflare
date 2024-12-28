#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/cmd_line_parser.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/int16.hpp>

#include <genicam/CameraNode.hpp>
#include <genicam/CameraSets.hpp>
#include <genicam/srv/trigger_camera.hpp>
#include <genicam/srv/trigger_capture.hpp>

#include <ArenaApi.h>
#include <GenICam.h>

using lni = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

inline uint64_t convert_mac(std::string mac) {
    mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
    return strtoul(mac.c_str(), NULL, 16);
}

CameraNode::CameraNode(Arena::IDevice* const pDevice, std::string camera_name, uint64_t mac_address, bool init = true) 
: rclcpp_lifecycle::LifecycleNode("camera_" + camera_name) {
    this->pDevice = pDevice;
    this->has_device = true;
    this->has_system = false;
    name = camera_name;
    mac = mac_address;
    RCLCPP_INFO(this->get_logger(), "Camera node for camera %s with MAC address %li is created", name.c_str(), mac);
    init_params();
    if (init) {
        config_node(false);
        add_device(pDevice, false);
        pDevice->StartStream();
    }
}

CameraNode::CameraNode(Arena::ISystem* const pSystem, std::string camera_name, uint64_t mac_address, bool init = true) 
: rclcpp_lifecycle::LifecycleNode("camera_" + camera_name) {
    this->pSystem = pSystem;
    this->has_system = true;
    this->has_device = false;
    name = camera_name;
    mac = mac_address;
    RCLCPP_INFO(this->get_logger(), "Camera node for camera %s with MAC address %li is created", name.c_str(), mac);
    init_params();
    if (init) {
        config_node(false);
        add_system(pSystem, false);
        pDevice->StartStream();
    }
}

CameraNode::CameraNode(std::string camera_name, uint64_t mac_address, bool init = true) 
: rclcpp_lifecycle::LifecycleNode("camera_" + camera_name) {
    this->has_system = false;
    this->has_device = false;
    name = camera_name;
    mac = mac_address;
    RCLCPP_INFO(this->get_logger(), "Camera node for camera %s with MAC address %li is created", name.c_str(), mac);
    init_params();
    if (init){
        config_node(false);
    }
}

CameraNode::~CameraNode() {
    RCLCPP_INFO(this->get_logger(), "Destroying camera node for camera %s with MAC address %li", name.c_str(), mac);
    pDevice->StopStream();
}

void CameraNode::add_device(Arena::IDevice* const pDevice, bool start_streaming = true) {
    this->pDevice = pDevice;
    this->has_device = true;
    load_camera_settings();
    if (start_streaming) {
        pDevice->StartStream();
    }
}

void CameraNode::add_system(Arena::ISystem* const pSystem, bool start_streaming = false) {
    this->pSystem = pSystem;
    this->has_system = true;
    try {
        pSystem->UpdateDevices(1000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
            uint64_t mac = convert_mac(deviceInfo.MacAddressStr().c_str());
            if (mac == this->mac) {
                add_device(pSystem->CreateDevice(deviceInfo), start_streaming);
            }
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", ge.what());
    }
}

void CameraNode::init_params() {
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    if (exposure == nullptr) {
        this->declare_parameter("exposure", 0.0);
        exposure = param_subscriber_->add_parameter_callback("param_int", std::bind(&CameraNode::param_callback, this, std::placeholders::_1));
    }
    if (gain == nullptr) {
        this->declare_parameter("gain", 0.0);
        gain = param_subscriber_->add_parameter_callback("param_float", std::bind(&CameraNode::param_callback, this, std::placeholders::_1));
    }
    captrig = this->create_client<genicam::srv::TriggerCapture>("captrig");
}

void CameraNode::config_node(bool managed = false) {
    // init_params();
    if (managed) {
        save_lpub_ = this->create_publisher<sensor_msgs::msg::Image>("save_" + name, 10);
        view_lpub_ = this->create_publisher<sensor_msgs::msg::Image>("view_" + name, 10);
        inf_lpub_ = this->create_publisher<sensor_msgs::msg::Image>("inf_" + name, 10);
    } else {
        RCLCPP_INFO(this->get_logger(), "Camera node %s with MAC address %li configured", name.c_str(), mac);
        save_pub_ = this->create_publisher<sensor_msgs::msg::Image>("save_" + name, 10);
        view_pub_ = this->create_publisher<sensor_msgs::msg::Image>("view_" + name, 10);
        inf_pub_ = this->create_publisher<sensor_msgs::msg::Image>("inf_" + name, 10);
    }
    service = this->create_service<genicam::srv::TriggerCamera>(
        "camtrig_" + name, std::bind(&CameraNode::service_trigger, this, 
        std::placeholders::_1, std::placeholders::_2));
}

lni::CallbackReturn CameraNode::on_configure(const rclcpp_lifecycle::State &state) {
    this->lifecycled = true;
    config_node(this->lifecycled);
    if (!has_device) {
        add_system(pSystem, false);
    }
    RCLCPP_INFO(this->get_logger(), "Configured transition succesfull for camera %s ", name.c_str());
    return lni::CallbackReturn::SUCCESS;
}

lni::CallbackReturn CameraNode::on_activate(const rclcpp_lifecycle::State &state) {
    if (!has_device) {
        RCLCPP_ERROR(this->get_logger(), "No device found for camera %s with MAC address %li", name.c_str(), mac);
        return lni::CallbackReturn::FAILURE;
    }
    pDevice->StartStream(); 
    save_lpub_->on_activate();
    view_lpub_->on_activate();
    inf_lpub_->on_activate();
    RCLCPP_INFO(this->get_logger(), "Activated transition succesfull for camera %s ", name.c_str());
    return lni::CallbackReturn::SUCCESS;
}

lni::CallbackReturn CameraNode::on_deactivate(const rclcpp_lifecycle::State &state) {
    pDevice->StopStream();
    save_lpub_->on_deactivate();
    view_lpub_->on_deactivate();
    inf_lpub_->on_deactivate();
    RCLCPP_INFO(this->get_logger(), "Deactivated transition succesfull for camera %s ", name.c_str());
    return lni::CallbackReturn::SUCCESS;
}

lni::CallbackReturn CameraNode::on_cleanup(const rclcpp_lifecycle::State &state) {
    save_lpub_.reset();
    view_lpub_.reset();
    inf_lpub_.reset();
    if (has_device) {
        pSystem->DestroyDevice(pDevice);
        has_device = false;
    }
    RCLCPP_INFO(this->get_logger(), "Cleanup transition succesfull for camera %s ", name.c_str());
    return lni::CallbackReturn::SUCCESS;
}

// lni::CallbackReturn CameraNode::on_shutdown(const rclcpp_lifecycle::State &state) {
//     pDevice->StopStream();
//     return lni::CallbackReturn::SUCCESS;
// }

void CameraNode::param_callback(const rclcpp::Parameter & p) {
    auto name = p.get_name();
    if (p.get_type_name() == "string") {
        auto val = p.get_parameter_value().get<std::string>();
    } else if (p.get_type_name() == "integer") {
        auto val = p.get_parameter_value().get<int>();
    } else if (p.get_type_name() == "double") {
        auto val = p.get_parameter_value().get<float>();
        if (name == "exposure") {
            GenApi::CFloatPtr p_exposure = pDevice->GetNodeMap()->GetNode("ExposureTime");
            p_exposure->SetValue(val);
        } else if (name == "gain") {
            GenApi::CFloatPtr p_gain = pDevice->GetNodeMap()->GetNode("Gain");
            p_gain->SetValue(val);
        }
    } else if (p.get_type_name() == "bool") {
        auto val = p.get_parameter_value().get<bool>();
    }
}

void CameraNode::load_camera_settings() {
    load_settings_from_func();
    load_settings_from_file();

    Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "PtpEnable", true);
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "AcquisitionStartMode", "PTPSync");
    GenApi::CFloatPtr framerate = pDevice->GetNodeMap()->GetNode("AcquisitionFrameRate");
    framerate->SetValue(framerate->GetMax());
    GenApi::CIntegerPtr pStreamChannelPacketDelay = pDevice->GetNodeMap()->GetNode("GevSCPD");
    pStreamChannelPacketDelay->SetValue(camset::by_mac.at(mac).scpd);
    GenApi::CIntegerPtr pStreamChannelFrameTransmissionDelay = pDevice->GetNodeMap()->GetNode("GevSCFTD");
    pStreamChannelFrameTransmissionDelay->SetValue(camset::by_mac.at(mac).scftd);
}

void CameraNode::load_settings_from_func() {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
    GenApi::CIntegerPtr width_node = pDevice->GetNodeMap()->GetNode("Width");
    width_node->SetValue(width_node->GetMax());
    GenApi::CIntegerPtr height_node = pDevice->GetNodeMap()->GetNode("Height");
    height_node->SetValue(height_node->GetMax());
    GenApi::CIntegerPtr channle_size = pDevice->GetNodeMap()->GetNode("DeviceStreamChannelPacketSize");
    channle_size->SetValue(channle_size->GetMax());
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "BGR8");
    // Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureAutoLimitAuto", "Off");
    // Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "GainAuto", "Continuous");
}

void CameraNode::load_settings_from_file() {
    auto config_dir = ament_index_cpp::get_package_share_directory("genicam") + "/configs/";
    std::string settings_file = config_dir + name + ".txt";
    RCLCPP_INFO(this->get_logger(), "Reading from file: %s", settings_file.c_str());
    Arena::FeatureStream stream(pDevice->GetNodeMap());
    stream.Read(settings_file.c_str());
}


sensor_msgs::msg::Image::SharedPtr CameraNode::get_image(int trigger_type) {
    captrig->wait_for_service(std::chrono::seconds(2));
    captrig->prune_pending_requests();
    
    auto request = std::make_shared<genicam::srv::TriggerCapture::Request>();
    request->mac_address = mac;
    auto result = captrig->async_send_request(request, [](rclcpp::Client<genicam::srv::TriggerCapture>::SharedFuture future) {
        auto response = future.get();
        bool success = response->success;
    });
    
    //sleep 2 seconds
    RCLCPP_INFO(this->get_logger(), "-- 1 --");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "-- 2 --");

    sensor_msgs::msg::Image::SharedPtr msg_image = nullptr;
    try{
        Arena::IImage* pImage = pDevice->GetImage(10000);
        if (pImage != nullptr) {
            cv::Mat image = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_8UC3, (void *)pImage->GetData());
            std_msgs::msg::Header header;
            header.stamp = this->now();
            msg_image = cv_bridge::CvImage( header, "bgr8", image).toImageMsg();
            pDevice->RequeueBuffer(pImage);
        }
    }
    catch (GenICam::GenericException& ge) {
        RCLCPP_WARN(this->get_logger(), "Warn: %s", ge.what());
    }
    return msg_image;
}

void CameraNode::service_trigger(
const std::shared_ptr<genicam::srv::TriggerCamera::Request> request, 
std::shared_ptr<genicam::srv::TriggerCamera::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Camera %s triggered with type %d", name.c_str(), request->type);
    if (this->lifecycled == true){
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_ERROR(this->get_logger(), "Camera %s is not active", name.c_str());
            response->success = false;
            response->status = "Camera is not active";
            return;
        }
    }
    sensor_msgs::msg::Image::SharedPtr msg_image = get_image(request->type);
    if (msg_image == nullptr) {
        response->success = false;
        response->status = "No image received";
        return;
    } else if (request->type == 1 || request->type == 10) {
        if (this->lifecycled == true) {
            save_lpub_->publish(*msg_image);
        } else {
            save_pub_->publish(*msg_image);
        }
        response->success = true;
        response->status = "Image saved";
    } else if (request->type == 2 || request->type == 20) {
        if (this->lifecycled == true) {
            view_lpub_->publish(*msg_image);
        } else {
            view_pub_->publish(*msg_image);
        }
        response->success = true;
        response->status = "Image shown";
    } else if (request->type == 3 || request->type == 30) {
        if (this->lifecycled == true) {
            save_lpub_->publish(*msg_image);
            inf_lpub_->publish(*msg_image);
        } else {
            save_pub_->publish(*msg_image);
            inf_pub_->publish(*msg_image);
        }
        response->success = true;
        response->status = "Image saved and sent to inference";
    } else if (request->type == 4 || request->type == 40) {
        if (this->lifecycled == true) {
            inf_lpub_->publish(*msg_image);
        } else {
            inf_pub_->publish(*msg_image);
        }
        response->success = true;
        response->status = "Image sent to inference";
    } else {
        response->success = false;
        response->status = "Invalid trigger value %d", request->type;
    }
}


int main_unmanaged(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;
    Arena::ISystem* pSystem = nullptr;
    std::vector<std::pair<Arena::IDevice*, uint64_t>> vDevices = std::vector<std::pair<Arena::IDevice*, uint64_t>>();

    try {
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(1000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
			Arena::IDevice* pDevice;
            uint64_t mac = convert_mac(deviceInfo.MacAddressStr().c_str());
            if (camset::by_mac.find(mac) != camset::by_mac.end()) {
                vDevices.push_back(std::make_pair(pDevice, mac));
            }
		}
    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", ge.what());
    }

    if (vDevices.size() == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No cameras found!, exiting...");
        exit(1);
    }
            
    rclcpp::executors::MultiThreadedExecutor executor;

    std::vector<std::shared_ptr<CameraNode>> camera_nodes;
    for (auto& pDevice : vDevices) {
        std::string camera_name = camset::by_mac.at(pDevice.second).name;
        auto camera_node = std::make_shared<CameraNode>(pSystem, camera_name, pDevice.second);
        // camera_node->add_system(pSystem);
        // camera_node->add_device(pDevice.first);
        camera_nodes.push_back(camera_node);
    }

    for (auto& camera_node : camera_nodes) {
        executor.add_node(camera_node->get_node_base_interface());
    }

    try {
        executor.spin();
    } catch (const std::exception &e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
    }

    for (auto& pDevice : vDevices) {
        pSystem->DestroyDevice(pDevice.first);
    }

    for (auto& camera_node : camera_nodes) {
        executor.remove_node(camera_node->get_node_base_interface());
    }

    Arena::CloseSystem(pSystem);
    rclcpp::shutdown();
    return 0;
}

int main_managed(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;

    Arena::ISystem* pSystem = nullptr;
    rclcpp::executors::MultiThreadedExecutor executor;
    std::vector<std::shared_ptr<CameraNode>> camera_nodes;
    try {
        pSystem = Arena::OpenSystem();
        for (auto& cam : camset::by_mac) {
            camera_nodes.push_back(std::make_shared<CameraNode>(pSystem, cam.second.name, cam.first, false));
            executor.add_node(camera_nodes.back()->get_node_base_interface());
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error (405): %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error (407): %s", ge.what());
    }

    try {
        executor.spin();
    } catch (const std::exception &e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
    }

    for (auto& camera_node : camera_nodes) {
        executor.remove_node(camera_node->get_node_base_interface());
    }

    Arena::CloseSystem(pSystem);
    rclcpp::shutdown();
    return 0;
}

int named_camera(int argc, char **argv, std::string camera_name) {
    rclcpp::init(argc, argv);
    
    Arena::ISystem* pSystem = nullptr;
    rclcpp::executors::MultiThreadedExecutor executor;
    std::vector<std::shared_ptr<CameraNode>> camera_nodes;
    try {
        pSystem = Arena::OpenSystem();
        for (auto& cam : camset::by_mac) {
            if (cam.second.name == camera_name){
                camera_nodes.push_back(std::make_shared<CameraNode>(pSystem, cam.second.name, cam.first, false));
                executor.add_node(camera_nodes.back()->get_node_base_interface());
            }
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error (405): %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error (407): %s", ge.what());
    }

    if(camera_nodes.empty()){
        return 1;
    }

    try {
        executor.spin();
    } catch (const std::exception &e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
    }

    for (auto& camera_node : camera_nodes) {
        executor.remove_node(camera_node->get_node_base_interface());
    }

    Arena::CloseSystem(pSystem);
    rclcpp::shutdown();
    return 0;
}

// int named_camera(int argc, char **argv, std::string camera_name) {
//     rclcpp::init(argc, argv);
//     std::cout << "... LAUNCHING CAMERA " << camera_name << " ..." << std::endl;
//     Arena::ISystem* pSystem = nullptr;
//     rclcpp::executors::MultiThreadedExecutor executor;
//     try {
//         pSystem = Arena::OpenSystem();
//         auto camera_node = std::make_shared<CameraNode>(pSystem, camera_name, camset::by_name.at(camera_name));
//         executor.add_node(camera_node->get_node_base_interface());
//         executor.spin();
//     } catch(const std::exception& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error (436): %s", e.what());
//     } catch (GenICam::GenericException& ge) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error (438): %s", ge.what());
//     }

//     Arena::CloseSystem(pSystem);
//     rclcpp::shutdown();
//     return 0;
// }

std::string parse_arguments(int argc, char * argv[], std::string custom_arg){
    auto return_string = "";
    for (int i = 1; i < argc; ++i){
        if (std::string(argv[i]) == "--" + custom_arg){
            if (i + 1 < argc) {  // Make sure we aren't at the end of argv!
                return_string = argv[++i];  // Increment 'i' so we don't get the argument as the next argv[i].
            }
            else { // There was no argument to the custom flag
                std::cerr << "--" << custom_arg << " option requires one argument." << std::endl;
            }
        }
    }
    return return_string;
}

int main(int argc, char **argv) {
    for (int i = 0; i < argc; ++i) {
        auto managed_str = parse_arguments(argc, argv, "managed");
        if (managed_str == "true") {
            return main_managed(argc, argv);
        } else if (managed_str == "false") {
            return main_unmanaged(argc, argv);
        } else if (managed_str != "") {
            return named_camera(argc, argv, managed_str);
        } else {
            std::cerr << "Invalid argument for managed flag\nuse --managed true|false or --managed <cam_name>" << std::endl;
            return 0;
        }
    }
}