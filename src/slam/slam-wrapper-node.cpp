#include "slam-wrapper-node.hpp"
#include <cstdlib>


// TODO:
// - From the functions which are used e.g., in loop closing or local mapping, only send the information about modified objects (DetectNBestCandidates->placerecognitionscore ++ etc.).
// - Checking in another thread? So one node is continuosuly receiving and storing data and another one is processing it.
// - Add headers to the messages so that timestamp can be used.
// - Save data when destroyed in the subscription side (also print stuff)
// - Create separate thread (similar way as in SLAM with local mapping etc.) where the new actions and data is checked and performed.
//    - One thread for checking the stuff inside of while loop?
//    - OR one main thread which then sends the data to other threads where computations are performed?



// TODO NEW:
// - KF grid stuff -> create ros to orb etc.
// - Modify in a way that the current mpOrbKeyFrames etc. are removed and add vector where keyframes which are ready to be added are located. E.g., keyframe is unprocessed, wait till all the data is stored, process, add it to vector, loop through vector and add to local mapper when suitable
// - Everywhere where the mpOrbKeyFrames etc. are used, use SLAM original variables where the keyframes are stored.
// - Try to fix the lock problem between local mapping and ros threads.
//
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
  mpKfDb_ = m_SLAM->GetKeyFrameDatabase();
  
  mbLocalMappingIsIdle = true;

  mpLocalMapper_->AllowLocalMapping(true);
  
  // Initialize maps for the SLAM data, unprocessed and processed
  //mpUnprocOrbKeyFrames=std::map<long unsigned int, std::tuple<ORB_SLAM3::KeyFrame*, orbslam3_interfaces::msg::KeyFrame::SharedPtr>>();
  //mpUnprocOrbMapPoints=std::map<long unsigned int, std::tuple<ORB_SLAM3::MapPoint*, orbslam3_interfaces::msg::MapPoint::SharedPtr>>();
  //mpUnprocOrbMaps=std::map<long unsigned int, std::tuple<ORB_SLAM3::Map*, orbslam3_interfaces::msg::Map::SharedPtr>>();

  //mpOrbKeyFrames=std::map<long unsigned int, ORB_SLAM3::KeyFrame*>();
  //mpOrbMapPoints=std::map<long unsigned int, ORB_SLAM3::MapPoint*>();
  //mpOrbMaps=std::map<long unsigned int, ORB_SLAM3::Map*>();
 

  CreatePublishers();


  /* Init subscribers */
  if (subscribe_to_slam) { 
    CreateSubscribers();
    
    

    action_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SlamWrapperNode::checkForNewActions, this)
    );
  
    //mpActionChecker = new ActionChecker(mpAtlas_, mpKfDb_);
    //mptActionChecker = new thread(&ActionChecker::Run, mpActionChecker);
    // Add the current map to the maps
    //std::vector<ORB_SLAM3::Map*> mpAtlasMaps = mpAtlas_->GetAllMaps();
    //for(const auto& map : mpAtlasMaps) {
    //  mpActionChecker->InsertMap(map);
    //}
  }

  //// Add the current map to the maps
  std::vector<ORB_SLAM3::Map*> mpAtlasMaps = mpAtlas_->GetAllMaps();
  for(const auto& map : mpAtlasMaps) {
    mpOrbMaps.insert(std::make_pair(map->GetId(), map));
  }
  
  //unsigned long int mnLastInitKFidMap = mpAtlas_->GetLastInitKFid();
  //publishNewMapId(mnLastInitKFidMap);
}


SlamWrapperNode::~SlamWrapperNode() {

  RCLCPP_FATAL(this->get_logger(), "Destroying SlamWrapperNode...");
  m_SLAM->Shutdown();
  // Stop all threads
//  std_msgs::msg::Bool endMsg;
  //endMsg.data = true;
  //end_publisher_->publish(endMsg);
  //RCLCPP_FATAL(this->get_logger(), "Sent msg to destroy other nodes in the network... Shutting down..");

}


void SlamWrapperNode::checkForNewActions() {
  if(mpUnprocOrbKeyFrames.size() > 0) {
    // Check if variables can be filled with new data
    std::cout << "Number of unprocessed KeyFrames before=" << mpUnprocOrbKeyFrames.size() << ", ";
    for (auto it = mpUnprocOrbKeyFrames.begin(); it != mpUnprocOrbKeyFrames.end();)
    {

      unique_lock<mutex> lock(mMutexNewKF);
      bool bUnprocessed = false;
      auto& mtORKf = it->second;
      ORB_SLAM3::KeyFrame* mopKf = std::get<0>(mtORKf);
      orbslam3_interfaces::msg::KeyFrame::SharedPtr mrpKf = std::get<1>(mtORKf);
      Converter::KeyFrameConverter::FillKeyFrameData(mopKf, mrpKf, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

      if(!bUnprocessed)
      {
        it = mpUnprocOrbKeyFrames.erase(it);
        if(!bUnprocessed && mbLocalMappingIsIdle) mpLocalMapper_->InsertKeyframeFromRos(mopKf);
      } else {
        ++it;
      }
    }
    std::cout << "and after=" << mpUnprocOrbKeyFrames.size() << std::endl;
  }

  if(mpUnprocOrbMapPoints.size() > 0)
  {
    // Check map points
    std::cout << "Number of unprocessed MapPoints before=" << mpUnprocOrbMapPoints.size() << ", ";
    for (auto it = mpUnprocOrbMapPoints.begin(); it != mpUnprocOrbMapPoints.end();)
    {
      unique_lock<mutex> lock(mMutexNewKF);

      bool bUnprocessed = false;
      auto& mtORMP = it->second;
      ORB_SLAM3::MapPoint* mopMp = std::get<0>(mtORMP);
      orbslam3_interfaces::msg::MapPoint::SharedPtr mrpMp = std::get<1>(mtORMP);
      Converter::MapPointConverter::FillMapPointData(mopMp, mrpMp, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

      if(!bUnprocessed)
      {
        it = mpUnprocOrbMapPoints.erase(it);
      } else {
        ++it;
      }
    }
    std::cout << "and after=" << mpUnprocOrbMapPoints.size() << std::endl;
  }
 
}

/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  
  unique_lock<mutex> lock(mMutexNewKF);
  
  std::cout << "##### publish kf: number of map points " << pKf->GetMap()->GetAllMapPoints().size() << " and in kf " << pKf->GetMapPoints().size() << " and matches " << pKf->GetMapPointMatches().size() << std::endl;
  mpOrbKeyFrames[pKf->mnId] = pKf; 
  for(ORB_SLAM3::MapPoint* pMP : pKf->GetMapPoints())
  {
    mpOrbMapPoints[pMP->mnId] = pMP;
  }

  orbslam3_interfaces::msg::KeyFrame msgKf = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKf);

  msgKf.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  //RCLCPP_INFO(this->get_logger(), "Publishing keyframe with id: %d", pKf->mnId);
  keyframe_publisher_->publish(msgKf);
}

/*        Map        */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
  
  unique_lock<mutex> lock(mMutexUpdateMap);

  RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
  auto msgM = Converter::MapConverter::OrbMapToRosMap(pM);
  msgM.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  map_publisher_->publish(msgM);
  RCLCPP_INFO(this->get_logger(), "Map published with id: %d", msgM.mn_id);
  
}


/*        MapPoint        */
void SlamWrapperNode::publishMapPoint(ORB_SLAM3::MapPoint* pMp) {
  //RCLCPP_INFO(this->get_logger(), "Publishing a new mappoint with id: %d", pMp->mnId);
  mpOrbMapPoints[pMp->mnId] = pMp;
  orbslam3_interfaces::msg::MapPoint msgMp = Converter::MapPointConverter::ORBSLAM3MapPointToROS(pMp);
  msgMp.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  map_point_publisher_->publish(msgMp);
}

/*        Local Mapping - Active        */
void SlamWrapperNode::publishLMActivityChange(bool bActive) {
  RCLCPP_INFO(this->get_logger(), "Publishing Local Mapping Activity change=%d", bActive);
  std_msgs::msg::Bool msg = std_msgs::msg::Bool();
  msg.data = bActive;
  
  mbLocalMappingIsIdle = !bActive;

  lm_active_publisher_->publish(msg);
}






/* OBJECT SUBSCRIPTION CALLBACKS */

/*        KeyFrame        */
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf) {
  if(rKf->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
 
  std::lock_guard<std::mutex> lock(mMutexNewKF);


  RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d", rKf->mn_id);
  bool bUnprocessed = false;
  ORB_SLAM3::KeyFrame* oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf, mpOrbKeyFrames, mpOrbMaps, &bUnprocessed);
  
  
  oKf->SetORBVocabulary(m_SLAM->GetORBVocabulary());
  oKf->SetKeyFrameDatabase(mpKfDb_);
  oKf->UpdateMap(mpOrbMaps[rKf->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
  //oKf->mpImuPreintegrated = new ORB_SLAM3::IMU::Preintegrated();
  std::vector<ORB_SLAM3::GeometricCamera*> mpCameras = mpAtlas_->GetAllCameras();
  oKf->mpCamera = mpAtlas_->GetAllCameras()[0];
  if(mpCameras.size() > 1) oKf->mpCamera2 = mpAtlas_->GetAllCameras()[1];
  //std::set<ORB_SLAM3::MapPoint*> mvpMapPoints = oKf->GetMapPoints();
  
  if(bUnprocessed){
    RCLCPP_INFO(this->get_logger(), "*##*#*#*#*#*#*#*###**#*#*#* Keyframe with ID=%d is unprocessed. *#*####*#*#*#**#*#*#*#*#", oKf->mnId);
    mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
    //mpActionChecker->InsertUnprocessedKeyFrame(oKf, rKf);
  } 
  //else {
    //mpActionChecker->InsertKeyFrame(oKf);
  //}
  mpOrbKeyFrames[oKf->mnId] = oKf;
  //mpActionChecker->InsertKeyFrame(oKf);
  std::cout << "number of map points in kf " << rKf->mvp_map_points.size() << std::endl;
  ORB_SLAM3::Map* kfMap;
  for(size_t i = 0; i < rKf->mvp_map_points.size(); i++)
  {
    orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rKf->mvp_map_points[i]);
    ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);
    
    oMP->UpdateMap(mpOrbMaps[mp->mp_map_id]);
    //mpAtlas_->AddMapPoint(oMP);
    //mpOrbMaps[oKf->mnOriginMapId]->AddMapPoint(oMP);
    oKf->AddMapPoint(oMP, i);
    mpAtlas_->AddMapPoint(std::move(oMP)); 
    
    if(bUnprocessed) {
      RCLCPP_INFO(this->get_logger(), "*##*#*#*#*#*#*#*###**#*#*#* MapPoint with ID=%d is unprocessed. *#*####*#*#*#**#*#*#*#*#", oMP->mnId);
      mpUnprocOrbMapPoints[oMP->mnId] = std::make_tuple(oMP, mp);
      //mpActionChecker->InsertUnprocessedMapPoint(oMP, mp);
    }

    //else {
      //mpActionChecker->InsertMapPoint(oMP);
    //}
    mpOrbMapPoints[oMP->mnId] = oMP;
  }
  std::cout << "map points in map " << mpAtlas_->MapPointsInMap() << std::endl; 
  std::cout << "##################### number of map points in map " << mpOrbMaps[oKf->mnOriginMapId]->GetAllMapPoints().size() << ", " << mpOrbMaps[oKf->mnOriginMapId]->MapPointsInMap() << ", map id=" << oKf->mnOriginMapId  << std::endl; 
  RCLCPP_INFO(this->get_logger(), "Keyframe with ID '%d is converted, with map id=%d.", oKf->mnId, oKf->mnOriginMapId); 
  
  mpAtlas_->AddKeyFrame(oKf);  
  if(!bUnprocessed && mbLocalMappingIsIdle) mpLocalMapper_->InsertKeyframeFromRos(oKf);
  //if(mbLocalMappingIsIdle) mpLocalMapper_->InsertKeyframeFromRos(oKf);
  //mpAtlas_->AddKeyFrame(oKf, true);
  
  //RCLCPP_INFO(this->get_logger(), "KF added to atlas."); 
}

/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM) {
  if(rM->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  unique_lock<mutex> lock(mMutexUpdateMap);

  RCLCPP_INFO(this->get_logger(), "\n ¤%&¤%&¤%&¤%&¤%&¤%&%¤& Got a new map, id: %d", rM->mn_id);
  bool bUnprocessed = false;
  ORB_SLAM3::Map* opM = Converter::MapConverter::RosMapToOrbMap(rM, mpOrbKeyFrames, mpOrbMapPoints, mpOrbMaps, &bUnprocessed);
  
  //mpOrbMaps.insert(std::make_pair(opM->GetId(), opM));

  //RCLCPP_INFO(this->get_logger(), "Map with ID '%d is converted.", opM->GetId()); 
  //mpLocalMapper_->InsertKeyframeFromRos(oKf);
  //mpAtlas_->AddKeyFrame(oKf, true);
}

/*        MapPoint        */
void SlamWrapperNode::GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr rpMp) {
  if(rpMp->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  //RCLCPP_INFO(this->get_logger(), "Got a new mappoint, id: %d", rpMp->mn_id);
  bool bUnprocessed = false;
  ORB_SLAM3::MapPoint* opMp = Converter::MapPointConverter::RosMapPointToOrb(rpMp, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

  if(bUnprocessed) {
    RCLCPP_INFO(this->get_logger(), "*##*#*#*#*#*#*#*###**#*#*#* MapPoint with ID=%d is unprocessed. *#*####*#*#*#**#*#*#*#*#", opMp->mnId);
    mpUnprocOrbMapPoints[opMp->mnId] = std::make_tuple(opMp, rpMp);
    //mpActionChecker->InsertUnprocessedMapPoint(opMp, rpMp);
  } 
  //else {
  //  mpActionChecker->InsertMapPoint(opMp);
  //}

  mpOrbMapPoints.insert(std::make_pair(opMp->mnId, opMp));

  //RCLCPP_INFO(this->get_logger(), "MapPoint with ID '%d is converted.", opMp->mnId); 
}

/*        LocalMapping - Active        */
void SlamWrapperNode::GrabLMActive(const std_msgs::msg::Bool::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Local mapping is active=%d", msg->data);
}

void SlamWrapperNode::publishEndMsg() 
{
  std_msgs::msg::Bool endMsg;
  endMsg.data = true;
  
  end_publisher_->publish(endMsg);
  RCLCPP_INFO(this->get_logger(), "Publishing to /Destroy");
}

void SlamWrapperNode::endCallback(std_msgs::msg::Bool::SharedPtr bMsg) {
  RCLCPP_INFO(this->get_logger(), "Received msg from /destroy");
  rclcpp::shutdown();
}



/*    INIT PUBLISHERS    */

void SlamWrapperNode::CreatePublishers() {
  /* KEYFRAME */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame");
  keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
      "/KeyFrame", 
      500);
  
  /* MAP */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
  map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
      "/Map", 
      1);
  
  /* MAPPOINT */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /MapPoint");
  map_point_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapPoint>(
      "/MapPoint", 
      1000);
  
  /* LocalMapping Active*/
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /LocalMapping/Active");
  lm_active_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/LocalMapping/Active", 
      10);




  /* Destroy */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /destroy");
  end_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/destroy", 
      10);

}







/*   INIT SUBSCRIBERS   */

void SlamWrapperNode::CreateSubscribers() {
    
  /* KF */  
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
  m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
      "KeyFrame",
      100,
      std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));
  
  /* Map */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
  m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
      "Map",
      10,
      std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));

  /* MapPoint */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /MapPoint");
  m_map_point_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::MapPoint>(
      "MapPoint",
      1000,
      std::bind(&SlamWrapperNode::GrabMapPoint, this, std::placeholders::_1));
  

  /* LocalMapping active */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Active");
  m_lm_active_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "LocalMapping/Active",
      10,
      std::bind(&SlamWrapperNode::GrabLMActive, this, std::placeholders::_1));


  /* Destroy node */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /destroy");
  end_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "/destroy", 
      10, 
      std::bind(&SlamWrapperNode::endCallback, this, std::placeholders::_1));
}
