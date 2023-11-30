#include "slam-publisher.hpp"
//using std::placeholders::_1;

SLAMPublisher::SLAMPublisher() 
: Node("SLAM_Publisher") {
  std::cout << "\n======== Initializing SLAM publisher =========" << std::endl;
  
  keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
      "/KeyFrame", 
      3);

  publisher_ = this->create_publisher<std_msgs::msg::String>(
      "Chatter", 
      10);
}

SLAMPublisher::~SLAMPublisher() {
  // Stop all threads
  std::cout << "SLAMPublisher node destroyed." << std::endl;
}

void SLAMPublisher::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  orbslam3_interfaces::msg::KeyFrame msgKf = MsgConverter::ORBSLAM3KeyFrameToROS(pKf);

  keyframe_publisher_->publish(msgKf);
}

void SLAMPublisher::publishMessage(const std::string& message_text) {
  auto message = std_msgs::msg::String();
  message.data = message_text;
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s", message.data.c_str());
  publisher_->publish(message);
}

