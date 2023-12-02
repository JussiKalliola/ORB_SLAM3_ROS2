#include "slam-publisher.hpp"
//using std::placeholders::_1;

SLAMPublisher::SLAMPublisher() 
: Node("SLAM_Publisher") {
  std::cout << "\n======== Initializing SLAM publisher =========" << std::endl;
  
  keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
      "/KeyFrame", 
      10);

  m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
      "KeyFrame",
      10,
      std::bind(&SLAMPublisher::GrabKeyFrame, this, std::placeholders::_1));
  
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

void SLAMPublisher::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf) {
  std::cout << "Got a new keyframe" << std::endl;
  ORB_SLAM3::KeyFrame* oKf = MsgConverter::ROSKeyFrameToORBSLAM3(rKf);
  std::cout << "Converted ros keyframe to orbslam3." << std::endl;
}

