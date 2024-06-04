#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <ArenaApi.h>
#include <GenICam.h>

const std::string c_left   = "192.168.2.253";  // 9 LEFT      70:B3:D5:DD:90:ED
const std::string c_right  = "192.168.2.251";  // 7 RIGHT     70:B3:D5:DD:90:FD
const std::string c_center = "192.168.2.252";  // - CENTER    70:B3:D5:DD:90:EF

const std::map<uint64_t, std::vector<std::string>> by_mac = {
    {30853686642969, {c_left,       "7",    "9b"}},
    {30853686643065, {c_left,       "1",    "9a"}},
    {30853686646563, {c_left,       "2",    "9c"}},
    {30853686653149, {c_left,       "5",    "9d"}},
    {30853686643056, {c_left,       "6",    "9e"}},
    {30853686646554, {c_right,      "6",    "7a"}},
    {30853686652294, {c_right,      "1",    "7b"}},
    {30853686445113, {c_right,      "5",    "7c"}},
    {30853686646528, {c_right,      "2",    "7d"}},
    {30853686653140, {c_right,      "3",    "7e"}},
    {30853686643187, {c_right,      "0",    "1"}},
    {30853686650340, {c_center,     "6",    "4"}},
    {30853686646397, {c_center,     "5",    "3"}},
    {30853686643161, {c_center,     "2",    "13"}},
    {30853686643152, {c_center,     "1",    "12"}},
    {30853686646406, {c_center,     "3",    "10"}},
    {30853686650497, {c_center,     "7",    "11a"}},
    {30853686650366, {c_center,     "7",    "11b"}}
};

class CameraNode : public rclcpp::Node {
    private:
        image_transport::Publisher image_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string name;
        uint64_t mac;
        Arena::IDevice* pDevice;
        cv::Mat image;

    public:
        CameraNode(Arena::IDevice* const pDevice, std::string camera_name, uint64_t mac_address) : Node("camera_" + camera_name) {
            this->pDevice = pDevice;
            name = camera_name;
            mac = mac_address;
            image_pub_ = image_transport::create_publisher(this, "camera_" + name + "/image");
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&CameraNode::get_image, this));
            RCLCPP_INFO(this->get_logger(), "Camera node created for camera %s with MAC address %li", camera_name.c_str(), mac_address);
            init_cameras();
        }

    private:
        void init_cameras() {
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

        void get_image() {
            pDevice->StartStream();
            try{
                Arena::IImage* pImage = pDevice->GetImage(1000);
                if (pImage != nullptr) {
                    image = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_8UC3, (void *)pImage->GetData());

                    std_msgs::msg::Header header;
                    header.stamp = this->now();
                    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage( header, "bgr8", image).toImageMsg();
                    image_pub_.publish(msg);

                    pDevice->RequeueBuffer(pImage);
                }
            }
            catch (GenICam::GenericException& ge) {
                RCLCPP_WARN(this->get_logger(), "Warn: %s", ge.what());
            }
            pDevice->StopStream();
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
        std::string camera_name = by_mac.at(pDevice.second).at(2);
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