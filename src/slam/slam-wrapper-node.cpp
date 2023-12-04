#include "slam-wrapper-node.hpp"
//using std::placeholders::_1;

SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM) : Node("slam_wrapper_node") {
  std::cout << "\n======== Initializing SLAM Wrapper node =========" << std::endl;
  
  m_SLAM = pSLAM;
  
  keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
      "/KeyFrame", 
      10);

  m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
      "KeyFrame",
      10,
      std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));
  
  //publisher_ = this->create_publisher<std_msgs::msg::String>(
  //    "Chatter", 
  //    10);
}


SlamWrapperNode::~SlamWrapperNode() {
  // Stop all threads
  RCLCPP_INFO(this->get_logger(), "SlamWrapperNode is destroyed");
}

void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  orbslam3_interfaces::msg::KeyFrame msgKf = MsgConverter::ORBSLAM3KeyFrameToROS(pKf);

  RCLCPP_INFO(this->get_logger(), "Publishing keyframe with id: %d", pKf->mnId);
  keyframe_publisher_->publish(msgKf);
}
//
//void SLAMPublisher::publishMessage(const std::string& message_text) {
//  auto message = std_msgs::msg::String();
//  message.data = message_text;
//  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s", message.data.c_str());
//  publisher_->publish(message);
//}

void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf) {
  RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d", rKf->mn_id);
  ORB_SLAM3::KeyFrame* oKf = MsgConverter::ROSKeyFrameToORBSLAM3(rKf);
  RCLCPP_INFO(this->get_logger(), "Keyframe with ID '%d is converted.", rKf->mn_id);
   
  m_SLAM->GetAtlas()->AddKeyframeFromRos(oKf);
}

