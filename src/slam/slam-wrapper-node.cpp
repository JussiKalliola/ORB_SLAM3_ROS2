#include "slam-wrapper-node.hpp"

SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam) : Node("SlamWrapperNode") {
  RCLCPP_INFO(this->get_logger(), "Initializing SLAM Wrapper Node.");
  m_SLAM = pSLAM;
  mpLocalMapper_ = m_SLAM->GetMapperPtr();
  mpAtlas_ = m_SLAM->GetAtlas();

  mpOrbKeyFrames=std::map<long unsigned int, ORB_SLAM3::KeyFrame*>();
  
  /* Init publishers */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame");
  keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
      "/KeyFrame", 
      10);
  

  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
  map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
      "/Map", 
      10);
  

  /* Init subscribers */
  if (subscribe_to_slam) 
  {
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
    m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
        "KeyFrame",
        10,
        std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));
  
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
    m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
        "Map",
        10,
        std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));
  }
  
  ORB_SLAM3::Map* pM = mpAtlas_->GetCurrentMap();
  publishMap(pM);
}


SlamWrapperNode::~SlamWrapperNode() {
  // Stop all threads
  RCLCPP_INFO(this->get_logger(), "SlamWrapperNode is destroyed");
}

/* Publish functions */


/*
Publish new Map's from ORB_SLAM3 system.
 */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
  RCLCPP_INFO(this->get_logger(), "Publishing a new map with id: %d", pM->GetId());
  orbslam3_interfaces::msg::Map msgM = Converter::MapConverter::OrbMapToRosMap(pM);
  map_publisher_->publish(msgM);
}

/*
Publish new KeyFrame's from ORB_SLAM3 system.
 */
void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  orbslam3_interfaces::msg::KeyFrame msgKf = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKf);
  
  RCLCPP_INFO(this->get_logger(), "Publishing keyframe with id: %d", pKf->mnId);
  keyframe_publisher_->publish(msgKf);
}


/* Subscription callbacks */

// Callback for /KeyFrame
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf) {
  RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d", rKf->mn_id);
  ORB_SLAM3::KeyFrame* oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf, mpOrbKeyFrames, mpOrbMaps);
  mpOrbKeyFrames.insert(std::make_pair(oKf->mnId, oKf));

  RCLCPP_INFO(this->get_logger(), "Keyframe with ID '%d is converted.", rKf->mn_id); 
  //mpLocalMapper_->InsertKeyframeFromRos(oKf);
  //mpAtlas_->AddKeyFrame(oKf, true);
}

// Callback for /Map
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM) {
  RCLCPP_INFO(this->get_logger(), "Got a new map, id: %d", rM->mn_id);
  ORB_SLAM3::Map* opM = Converter::MapConverter::RosMapToOrbMap(rM, mpOrbKeyFrames);
  mpOrbMaps.insert(std::make_pair(opM->GetId(), opM));

  RCLCPP_INFO(this->get_logger(), "Map with ID '%d is converted.", opM->GetId()); 
  //mpLocalMapper_->InsertKeyframeFromRos(oKf);
  //mpAtlas_->AddKeyFrame(oKf, true);
}
