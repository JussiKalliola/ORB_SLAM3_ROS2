#include "slam-wrapper-node.hpp"
#include <cstdlib>

SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam) : Node("SlamWrapperNode") {
  
  const char* systemId = std::getenv("SLAM_SYSTEM_ID");
  
  if (systemId != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Initializing SLAM Wrapper Node. System ID=%s", systemId);
  } else {
    
    RCLCPP_FATAL(this->get_logger(), "Initializing SLAM Wrapper Node. System ID not found. Set up SLAM_SYSTEM_ID before continuing, exitting...");
    exit(1);
  }

  m_SLAM = pSLAM;
  mpLocalMapper_ = m_SLAM->GetMapperPtr();
  mpAtlas_ = m_SLAM->GetAtlas();

  mpOrbKeyFrames=std::map<long unsigned int, ORB_SLAM3::KeyFrame*>();
  
  CreatePublishers();

  /* Init subscribers */
  if (subscribe_to_slam) { 
    CreateSubscribers(); 
  }
  
  //unsigned long int mnLastInitKFidMap = mpAtlas_->GetLastInitKFid();
  //publishNewMapId(mnLastInitKFidMap);
}


SlamWrapperNode::~SlamWrapperNode() {
  // Stop all threads
  RCLCPP_FATAL(this->get_logger(), "SlamWrapperNode is destroyed");
}

/* Publish functions */


/*
Publish new Map's from ORB_SLAM3 system with full data.
 */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
  RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
  orbslam3_interfaces::msg::Map msgM = Converter::MapConverter::OrbMapToRosMap(pM);
  map_publisher_->publish(msgM);
}

/*
Publish new Map's from ORB_SLAM3 system with initial id.
 */
void SlamWrapperNode::publishMapAction() {
  RCLCPP_INFO(this->get_logger(), "Publishing a map action.");
  //std_msgs::msg::Int64 msg;
  //msg.data = static_cast<long int>(mnLastInitKFidMap);
  //map_action_publisher_->publish(msg);
}


void SlamWrapperNode::publishAtlasAction(orbslam3_interfaces::msg::AtlasActions aMsg) {
  //RCLCPP_INFO(this->get_logger(), "Publishing Atlas Action.");
  atlas_action_publisher_->publish(aMsg);
}

void SlamWrapperNode::publishKFAction(orbslam3_interfaces::msg::KeyFrameActions kfMsg) {
  RCLCPP_INFO(this->get_logger(), "Publishing KeyFrame Action.");
  kf_action_publisher_->publish(kfMsg);
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
  
  std::set<ORB_SLAM3::MapPoint*> mvpMapPoints = oKf->GetMapPoints();

  // Iterate through the set
  for (ORB_SLAM3::MapPoint* mapPointPtr : mvpMapPoints) {
    unsigned long int idToCheck = mapPointPtr->mnId;

    // Check if the id exists in the map
    if (mpOrbMapPoints.find(idToCheck) == mpOrbMapPoints.end()) {
      // If it doesn't exist, add it to the map
      mpOrbMapPoints[idToCheck] = mapPointPtr;
    }
  }

  mpOrbKeyFrames.insert(std::make_pair(oKf->mnId, oKf));

  RCLCPP_INFO(this->get_logger(), "Keyframe with ID '%d is converted.", rKf->mn_id); 
  //mpLocalMapper_->InsertKeyframeFromRos(oKf);
  mpAtlas_->AddKeyFrame(oKf, true);
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


void SlamWrapperNode::GrabKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rM) {
  RCLCPP_INFO(this->get_logger(), "Got new KeyFrame action.");

}



// Callback for /Map
void SlamWrapperNode::GrabAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rAA) {
  //RCLCPP_INFO(this->get_logger(), "Got new atlas action.");
  int actionId = Parser::Action::parseAtlasAction(rAA, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);

  if (actionId == 0) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> MapPoint found already.");
    mpAtlas_->AddMapPoint(mpOrbMapPoints[rAA->add_map_point_id], true);
  }     
  
  if (actionId==1) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Map found.");
    mpAtlas_->ChangeMap(mpOrbMaps[rAA->change_map_to_id], true);
  }

  if (actionId==2) {
    mpAtlas_->AddKeyFrame(mpOrbKeyFrames[rAA->add_kf_id], true);
  }

  if (actionId==3) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Map found.");
    mpAtlas_->SetMapBad(mpOrbMaps[rAA->set_map_bad_id], true);
  }
  
  if (actionId==4) { 
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Create new Map");
    mpAtlas_->CreateNewMap(true);
  }

  if (actionId==5) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Inform new big change.");
    mpAtlas_->InformNewBigChange(true);
  }

  if (actionId==6) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Clear current map.");
    mpAtlas_->clearMap(true);
  }

  if (actionId==7) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Clear Atlas.");
    mpAtlas_->clearAtlas(true);
  }

  if (actionId==8) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Remove bad maps.");
    mpAtlas_->RemoveBadMaps(true);
  }

  if (actionId==9) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Set Inertial Sensor.");
    mpAtlas_->SetInertialSensor(true);
  }
  
  if (actionId==10) {
    //RCLCPP_INFO(this->get_logger(), "AtlasAction >> Set IMU initialized.");
    mpAtlas_->SetImuInitialized(true);
  }
        
}



void SlamWrapperNode::CreatePublishers() {
  /* Init publishers */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame");
  keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
      "/KeyFrame", 
      10);
  

  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
  map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
      "/Map", 
      10);
  
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map/Actions");
  map_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapActions>(
      "/Map/Actions", 
      10);

  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Atlas/Actions");
  atlas_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::AtlasActions>(
      "/Atlas/Actions", 
      10);
  
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame/Actions");
  kf_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrameActions>(
      "/KeyFrame/Actions", 
      10);
}

void SlamWrapperNode::CreateSubscribers() {
    
  /* KF */  
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
  m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
      "KeyFrame",
      10,
      std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));
  
  /* KF Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame/Actions");
  m_kf_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrameActions>(
      "KeyFrame/Actions",
      10,
      std::bind(&SlamWrapperNode::GrabKeyFrameAction, this, std::placeholders::_1));
  
  /* Map */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
  m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
      "Map",
      10,
      std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));

  /* Atlas Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Atlas/Actions");
  m_atlas_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::AtlasActions>(
      "Atlas/Actions",
      10,
      std::bind(&SlamWrapperNode::GrabAtlasAction, this, std::placeholders::_1));
  
}
