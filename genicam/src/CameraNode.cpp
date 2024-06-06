#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <harvester_interfaces/srv/trigger_camera.hpp>
#include <std_msgs/msg/int16.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <genicam/CameraNode.hpp>
#include <genicam/CameraSets.hpp>


#include <ArenaApi.h>
#include <GenICam.h>


CameraNode::CameraNode(Arena::IDevice* const pDevice, std::string camera_name, uint64_t mac_address, bool incom = true) 
: rclcpp_lifecycle::LifecycleNode("camera_" + camera_name, rclcpp::NodeOptions().use_intra_process_comms(incom)) {
    this->pDevice = pDevice;
    name = camera_name;
    mac = mac_address;
    
    config_node();
    init_cameras();
}

CameraNode::CameraNode(std::string camera_name, uint64_t mac_address, bool incom = true) 
: rclcpp_lifecycle::LifecycleNode("camera_" + camera_name, rclcpp::NodeOptions().use_intra_process_comms(incom)) {
    name = camera_name;
    mac = mac_address;
    config_node();
}

int CameraNode::add_device(Arena::IDevice* const pDevice) {
    this->pDevice = pDevice;
    init_cameras();
}

void CameraNode::config_node() {
    RCLCPP_INFO(this->get_logger(), "Camera node %s with MAC address %li", name.c_str(), mac);
    // save_pub_ = image_transport::create_publisher(this, "save_" + name);
    save_pub_ = this->create_publisher<sensor_msgs::msg::Image>("save_" + name, 10);
    // view_pub_ = image_transport::create_publisher(this, "view_" + name);
    view_pub_ = this->create_publisher<sensor_msgs::msg::Image>("view_" + name, 10);
    // inf_pub_ = image_transport::create_publisher(this, "inf_" + name);
    inf_pub_ = this->create_publisher<sensor_msgs::msg::Image>("inf_" + name, 10);
    
    this->declare_parameter("exposure", 0.0);
    this->declare_parameter("gain", 0.0);
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    exposure = param_subscriber_->add_parameter_callback("param_int", std::bind(&CameraNode::param_callback, this, std::placeholders::_1));
    gain = param_subscriber_->add_parameter_callback("param_float", std::bind(&CameraNode::param_callback, this, std::placeholders::_1));

    trigger = this->create_subscription<std_msgs::msg::Int16>("trigger_" + name, 1, std::bind(&CameraNode::topic_trigger, this, std::placeholders::_1));
    service = this->create_service<trigg>("camtrig_" + name, std::bind(&CameraNode::service_trigger, this, std::placeholders::_1, std::placeholders::_2));
}

CameraNode::~CameraNode() {
    RCLCPP_INFO(this->get_logger(), "Destroying camera node for camera %s with MAC address %li", name.c_str(), mac);
    pDevice->StopStream();
}

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

void CameraNode::init_cameras() {
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

    pDevice->StartStream();
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
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureAutoLimitAuto", "Off");
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "GainAuto", "Continuous");
}

void CameraNode::load_settings_from_file() {
    auto config_dir = ament_index_cpp::get_package_share_directory("genicam") + "/configs/";
    std::string settings_file = config_dir + name + ".txt";
    RCLCPP_INFO(this->get_logger(), "Reading from file: %s", settings_file.c_str());
    Arena::FeatureStream stream(pDevice->GetNodeMap());
    stream.Write(settings_file.c_str());
}


sensor_msgs::msg::Image::SharedPtr CameraNode::get_image(int trigger_type) {
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

void CameraNode::topic_trigger(const std_msgs::msg::Int16::SharedPtr msg) {
    sensor_msgs::msg::Image::SharedPtr msg_image = get_image(msg->data);

    if (msg_image == nullptr) {
        RCLCPP_WARN(this->get_logger(), "No image received");
    } else if (msg->data == 1 || msg->data == 10) {
        save_pub_->publish(*msg_image);
        RCLCPP_INFO(this->get_logger(), "Image saved");
    } else if (msg->data == 2 || msg->data == 20) {
        view_pub_->publish(*msg_image);
        RCLCPP_INFO(this->get_logger(), "Image shown");
    } else if (msg->data == 3 || msg->data == 30) {
        inf_pub_->publish(*msg_image);
        save_pub_->publish(*msg_image);
    } else if (msg->data == 4 || msg->data == 40) {
        inf_pub_->publish(*msg_image);
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid trigger value %d", msg->data);
    }
}
void CameraNode::service_trigger(const std::shared_ptr<trigg::Request> request, std::shared_ptr<trigg::Response> response) {
    sensor_msgs::msg::Image::SharedPtr msg_image = get_image(request->type);

    if (msg_image == nullptr) {
        response->success = false;
        response->status = "No image received";
        return;
    } else if (request->type == 1 || request->type == 10) {
        save_pub_->publish(*msg_image);
        response->success = true;
        response->status = "Image saved";
    } else if (request->type == 2 || request->type == 20) {
        view_pub_->publish(*msg_image);
        response->success = true;
        response->status = "Image shown";
    } else if (request->type == 3 || request->type == 30) {
        inf_pub_->publish(*msg_image);
        save_pub_->publish(*msg_image);
        response->success = true;
        response->status = "Image saved and sent to inference";
    } else if (request->type == 4 || request->type == 40) {
        inf_pub_->publish(*msg_image);
        response->success = true;
        response->status = "Image sent to inference";
    } else {
        response->success = false;
        response->status = "Invalid trigger value %d", request->type;
    }
}

uint64_t convert_mac(std::string mac) {
  mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
  return strtoul(mac.c_str(), NULL, 16);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;
    Arena::ISystem* pSystem = nullptr;
    std::vector<std::pair<Arena::IDevice*, uint64_t>> vDevices = std::vector<std::pair<Arena::IDevice*, uint64_t>>();

    try {
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(1000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
			Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfo);
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
        auto camera_node = std::make_shared<CameraNode>(camera_name, pDevice.second);
        camera_node->add_device(pDevice.first);
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