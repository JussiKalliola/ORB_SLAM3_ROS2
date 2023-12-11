#include "slam-wrapper-node.hpp"

SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam) : Node("SlamWrapperNode") {
  RCLCPP_INFO(this->get_logger(), "Initializing SLAM Wrapper Node.");
  m_SLAM = pSLAM;
  mpLocalMapper_ = m_SLAM->GetMapperPtr();
  mpAtlas_ = m_SLAM->GetAtlas();

  mpOrbKeyFrames=std::map<long unsigned int, ORB_SLAM3::KeyFrame*>();

  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame");
  keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
      "/KeyFrame", 
      10);
  

  if (subscribe_to_slam) 
  {
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
    m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
        "KeyFrame",
        10,
        std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));
  }
  
}


SlamWrapperNode::~SlamWrapperNode() {
  // Stop all threads
  RCLCPP_INFO(this->get_logger(), "SlamWrapperNode is destroyed");
}

void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  orbslam3_interfaces::msg::KeyFrame msgKf = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKf);
  
  RCLCPP_INFO(this->get_logger(), "Publishing keyframe with id: %d", pKf->mnId);
  keyframe_publisher_->publish(msgKf);
}

void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf) {
  RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d", rKf->mn_id);
  ORB_SLAM3::KeyFrame* oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf, mpOrbKeyFrames);
  mpOrbKeyFrames.insert(std::make_pair(oKf->mnId, oKf));
  RCLCPP_INFO(this->get_logger(), "Size of the orb kf map %d", mpOrbKeyFrames.size() );

  RCLCPP_INFO(this->get_logger(), "Keyframe with ID '%d is converted.", rKf->mn_id); 
  //mpLocalMapper_->InsertKeyframeFromRos(oKf);
  mpAtlas_->AddKeyFrame(oKf, true);
}

