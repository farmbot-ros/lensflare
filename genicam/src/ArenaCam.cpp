#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <std_msgs/msg/int16.hpp>

#include <ArenaApi.h>
#include <GenICam.h>


const std::string c_left   = "192.168.2.253";  // 9 LEFT      70:B3:D5:DD:90:ED
const std::string c_right  = "192.168.2.251";  // 7 RIGHT     70:B3:D5:DD:90:FD
const std::string c_center = "192.168.2.252";  // - CENTER    70:B3:D5:DD:90:EF

const int GAIN = 1;
const float EXP = 1.2;

struct Data {
    std::string ip;
    std::string channel;
    std::string name;
    double exp;
    int gain;
    int sw;
    int scpd;
    double scftd;
};

std::unordered_map<uint64_t, Data> by_mac = {
    {30853686642969, {c_left,     "7",   "9b",   521 * EXP,    30 * GAIN,   1,   240000,    0 * 80000}},
    {30853686643065, {c_left,     "1",   "9a",   521 * EXP,    30 * GAIN,   1,   240000,    1 * 80000}},
    {30853686646563, {c_left,     "2",   "9c",   521 * EXP,    30 * GAIN,   1,   240000,    2 * 80000}},
    {30853686653149, {c_left,     "5",   "9d",   521 * EXP,    30 * GAIN,   1,   240000,    3 * 80000}},
    {30853686643056, {c_left,     "6",   "9e",   521 * EXP,    30 * GAIN,   2,   240000,    0 * 80000}},
    {30853686646554, {c_right,    "6",   "7a",   521 * EXP,    30 * GAIN,   2,   240000,    1 * 80000}},
    {30853686653140, {c_right,    "3",   "7e",   521 * EXP,    30 * GAIN,   2,   240000,    2 * 80000}},
    {30853686445113, {c_right,    "5",   "7c",   521 * EXP,    30 * GAIN,   2,   240000,    3 * 80000}},
    {30853686646528, {c_right,    "2",   "7d",   521 * EXP,    30 * GAIN,   3,   240000,    0 * 80000}},
    {30853686652294, {c_right,    "1",   "7b",   521 * EXP,    30 * GAIN,   3,   240000,    2 * 80000}},
    {30853686643187, {c_right,    "0",   "1",    2847 * 1.1,   20 * GAIN,   3,   240000,    1 * 80000}},
    {30853686650340, {c_center,   "6",   "4",    528 * 2.8,    30 * GAIN,   3,   240000,    3 * 80000}},
    {30853686646397, {c_center,   "5",   "3",    810 * 2.8,    20 * GAIN,   4,   240000,    0 * 80000}},
    {30853686643152, {c_center,   "1",   "12",   460 * 3.8,    40 * GAIN,   4,   240000,    1 * 80000}},
    {30853686646406, {c_center,   "3",   "10",   867.0,        30 * GAIN,   4,   240000,    2 * 80000}},
    {30853686643161, {c_center,   "2",   "13",   805 * 3.8,    20 * GAIN,   0,   240000,    0 * 80000}}, // directly
    {30853686650366, {c_center,   "7",   "11b",  498.0,        15 * GAIN,   0,   240000,    0 * 80000}}, // directly
    {30853686650497, {c_center,   "7",   "11a",  498.0,        15 * GAIN,   0,   240000,    0 * 80000}}  // directly
};

class CameraNode : public rclcpp::Node {
    private:
        image_transport::Publisher save_pub_;
        image_transport::Publisher view_pub_;
        image_transport::Publisher inf_pub_;

        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr trigger;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv;
        //subscriber
        std::string name;
        uint64_t mac;
        Arena::IDevice* pDevice;

    public:
        CameraNode(Arena::IDevice* const pDevice, std::string camera_name, uint64_t mac_address) : Node("camera_" + camera_name) {
            this->pDevice = pDevice;
            name = camera_name;
            mac = mac_address;
            save_pub_ = image_transport::create_publisher(this, "save_" + name);
            view_pub_ = image_transport::create_publisher(this, "view_" + name);
            inf_pub_ = image_transport::create_publisher(this, "inf_" + name);
            RCLCPP_INFO(this->get_logger(), "Camera node created for camera %s with MAC address %li", camera_name.c_str(), mac_address);
            trigger = this->create_subscription<std_msgs::msg::Int16>("trigger_" + name, 1, std::bind(&CameraNode::get_image, this, std::placeholders::_1));
            init_cameras();
        }

        // Destructor
        ~CameraNode() {
            RCLCPP_INFO(this->get_logger(), "Destroying camera node for camera %s with MAC address %li", name.c_str(), mac);
            pDevice->StopStream();
        }

    private:
        void init_cameras() {
            // Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");
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

            Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "PtpEnable", true);
            Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "AcquisitionStartMode", "PTPSync");

            GenApi::CFloatPtr framerate = pDevice->GetNodeMap()->GetNode("AcquisitionFrameRate");
            framerate->SetValue(framerate->GetMax());

            GenApi::CFloatPtr pPTPSyncFrameRate = pDevice->GetNodeMap()->GetNode("PTPSyncFrameRate");
            pPTPSyncFrameRate->SetValue(7.0);

            GenApi::CIntegerPtr pStreamChannelPacketDelay = pDevice->GetNodeMap()->GetNode("GevSCPD");
            pStreamChannelPacketDelay->SetValue(by_mac.at(mac).scpd);

            GenApi::CIntegerPtr pStreamChannelFrameTransmissionDelay = pDevice->GetNodeMap()->GetNode("GevSCFTD");
            pStreamChannelFrameTransmissionDelay->SetValue(by_mac.at(mac).scftd);

            pDevice->StartStream();
        }

        void get_image(const std_msgs::msg::Int16::SharedPtr msg) {
            sensor_msgs::msg::Image::SharedPtr msg_image = nullptr;
            try{
                Arena::IImage* pImage = pDevice->GetImage(5000);
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
                return;
            }
            if (msg_image == nullptr) {
                RCLCPP_WARN(this->get_logger(), "No image received");
                return;
            } else if (msg->data == 1) {
                save_pub_.publish(msg_image);
                RCLCPP_INFO(this->get_logger(), "Image saved");
            } else if (msg->data == 2) {
                view_pub_.publish(msg_image);
                RCLCPP_INFO(this->get_logger(), "Image shown");
            } else if (msg->data == 3) {
                inf_pub_.publish(msg_image);
                save_pub_.publish(msg_image);
            } else if (msg->data == 4) {
                inf_pub_.publish(msg_image);
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid trigger value %d", msg->data);
            }
        }
};

uint64_t convert_mac(std::string mac) {
  mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
  return strtoul(mac.c_str(), NULL, 16);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;
    Arena::ISystem* pSystem = nullptr;
    // std::vector<Arena::IDevice*> vDevices = std::vector<Arena::IDevice*>();
    std::vector<std::pair<Arena::IDevice*, uint64_t>> vDevices = std::vector<std::pair<Arena::IDevice*, uint64_t>>();

    try {
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(1000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
			Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfo);
            uint64_t mac = convert_mac(deviceInfo.MacAddressStr().c_str());
			// vDevices.push_back(pDevice);
            if (by_mac.find(mac) != by_mac.end()) {
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
        std::string camera_name = by_mac.at(pDevice.second).name;
        auto camera_node = std::make_shared<CameraNode>(pDevice.first, camera_name, pDevice.second);
        camera_nodes.push_back(camera_node);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera found with MAC address %li", pDevice.second);
    }

    for (auto& camera_node : camera_nodes) {
        executor.add_node(camera_node);
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
        executor.remove_node(camera_node);
    }

    Arena::CloseSystem(pSystem);
    rclcpp::shutdown();
    return 0;
}