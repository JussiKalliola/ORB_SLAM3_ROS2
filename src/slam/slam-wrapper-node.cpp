#include "slam-wrapper-node.hpp"
#include <cstdlib>


// TODO:
// - From the functions which are used e.g., in loop closing or local mapping, only send the information about modified objects (DetectNBestCandidates->placerecognitionscore ++ etc.).
// - Create few different threads if the whole function needs to be executed
// - Send new keyframe from the constructor, not from Atlas::AddKeyFrame
// - Create MapPoint publisher/subscriber and actions
// - Checking in another thread? So one node is continuosuly receiving and storing data and another one is processing it.
// - Add headers to the messages so that timestamp can be used.
// - Use deque instead of vector?
// - Save data when destroyed in the subscription side (also print stuff)
  
}

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

  mpOrbKeyFrames=std::map<long unsigned int, ORB_SLAM3::KeyFrame*>();
  
  CreatePublishers();


  /* Init subscribers */
  if (subscribe_to_slam) { 
    CreateSubscribers();

    action_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SlamWrapperNode::checkForNewActions, this)
    );
  }
  
  //unsigned long int mnLastInitKFidMap = mpAtlas_->GetLastInitKFid();
  //publishNewMapId(mnLastInitKFidMap);
}


SlamWrapperNode::~SlamWrapperNode() {
  // Stop all threads
  RCLCPP_FATAL(this->get_logger(), "SlamWrapperNode is destroyed");
}





void SlamWrapperNode::checkForNewActions() {
  if (!mRosActionMap.empty()) {
    RCLCPP_INFO(get_logger(), "Action buffer contains items, try to perform actions.");
    
    // Perform action if there are new keyframes
    size_t numOfActions = mRosActionMap.size();
    
    std::vector<int> kfIdsToRemove;
    std::vector<int> kfdbIdsToRemove;
    std::vector<int> rosActionIdsToRemove;
    std::vector<int> atlasIdsToRemove;
    

    for(size_t i = 0; i < numOfActions; ++i) 
    { 
      int categoryIdx = std::get<0>(mRosActionMap[i]);
      int vecIdx = std::get<1>(mRosActionMap[i]);


      if(categoryIdx==0)
      {
        
        orbslam3_interfaces::msg::AtlasActions::SharedPtr mAtlasRosMsg = mvpAtlasRosActions[vecIdx];
        if(mAtlasRosMsg==nullptr) continue;
        bool success = PerformAtlasAction(mAtlasRosMsg);
          
        //RCLCPP_INFO(get_logger(), "%d %d %d", vecIdx, mAtlasRosMsg->add_kf_id, success);
        if(success) 
        {
          atlasIdsToRemove.push_back(vecIdx);
          rosActionIdsToRemove.push_back(i);
        }
        
      } else if(categoryIdx==1) {
        orbslam3_interfaces::msg::KeyFrameActions::SharedPtr mKfRosMsg = mvpKfRosActions[vecIdx];
        
        if(mKfRosMsg==nullptr) continue;
        
        if(mpOrbKeyFrames.find(mKfRosMsg->kf_id) != mpOrbKeyFrames.end()) {
          bool success = PerformKeyFrameAction(mKfRosMsg);
          
          if(success) 
          {
            kfIdsToRemove.push_back(vecIdx);
            rosActionIdsToRemove.push_back(i);
          }
        }
      } else if(categoryIdx==2) {
        orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr mKfdbRosMsg = mvpKFDBRosActions[vecIdx];
        
        if(mKfdbRosMsg==nullptr) continue;
        
        bool success = PerformKFDBAction(mKfdbRosMsg);
          
        if(success) 
        {
          kfdbIdsToRemove.push_back(vecIdx);
          rosActionIdsToRemove.push_back(i);
        }
      }

    }

    RCLCPP_INFO(get_logger(), "All : Action performed=%d, total number of actions=%d", rosActionIdsToRemove.size(), numOfActions);
    for(const auto& id : rosActionIdsToRemove)
    {
      mRosActionMap.erase(mRosActionMap.begin() + id);
    }

    RCLCPP_INFO(get_logger(), "Atlas : Action performed=%d, total number of actions=%d", atlasIdsToRemove.size(), mvpAtlasRosActions.size());
    for(const auto& id : atlasIdsToRemove)
    {
      mvpAtlasRosActions[id] = nullptr;
      //mvpAtlasRosActions.erase(mvpAtlasRosActions.begin() + id);
    }

    RCLCPP_INFO(get_logger(), "KeyFrame : Action performed=%d, total number of actions=%d", kfIdsToRemove.size(), mvpKfRosActions.size());
    for(const auto& id : kfIdsToRemove)
    {
      mvpKfRosActions[id] = nullptr;
      //mvpKfRosActions.erase(mvpKfRosActions.begin() + id);
    }

    RCLCPP_INFO(get_logger(), "KFDB : Action performed=%d, total number of actions=%d", kfdbIdsToRemove.size(), mvpKFDBRosActions.size());
    for(const auto& id : kfdbIdsToRemove)
    {
      mvpKFDBRosActions[id] = nullptr;
      //mvpKfRosActions.erase(mvpKfRosActions.begin() + id);
    }

  } else {
    RCLCPP_INFO(get_logger(), "No new keyframes. Checking again after delay...");
    action_check_timer_->reset(); // Reschedule the timer to check after a delay
    //keyframe_check_timer_->change_period(std::chrono::milliseconds(check_delay_ms_));
  }
}



/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  orbslam3_interfaces::msg::KeyFrame msgKf = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKf);

  msgKf.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  RCLCPP_INFO(this->get_logger(), "Publishing keyframe with id: %d", pKf->mnId);
  keyframe_publisher_->publish(msgKf);
}

/*        Map        */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
  RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
  orbslam3_interfaces::msg::Map msgM = Converter::MapConverter::OrbMapToRosMap(pM);
  msgM.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  map_publisher_->publish(msgM);
}



/* ACTION PUBLISHERS */

void SlamWrapperNode::publishMapAction() {
  RCLCPP_INFO(this->get_logger(), "Publishing a map action.");
  //map_action_publisher_->publish(msg);
}

void SlamWrapperNode::publishAtlasAction(orbslam3_interfaces::msg::AtlasActions aMsg) {
  //RCLCPP_INFO(this->get_logger(), "Publishing Atlas Action.");
  atlas_action_publisher_->publish(aMsg);
}

void SlamWrapperNode::publishKFAction(orbslam3_interfaces::msg::KeyFrameActions kfMsg) {
  //RCLCPP_INFO(this->get_logger(), "Publishing KeyFrame Action %d.", kfMsg.kf_id);
  kf_action_publisher_->publish(kfMsg);
}

void SlamWrapperNode::publishKFDBAction(orbslam3_interfaces::msg::KeyFrameDatabaseActions kfdbMsg) {
  RCLCPP_INFO(this->get_logger(), "Publishing KeyFrame Database Action.");
  kf_db_action_publisher_->publish(kfdbMsg);
}







/* OBJECT SUBSCRIPTION CALLBACKS */

/*        KeyFrame        */
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf) {
  if(rKf->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
 
  //RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d", rKf->mn_id);
  ORB_SLAM3::KeyFrame* oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf, mpOrbKeyFrames, mpOrbMaps);
  
  oKf->SetORBVocabulary(mpAtlas_->GetORBVocabulary());
  oKf->SetKeyFrameDatabase(mpKfDb_);
  oKf->UpdateMap(mpAtlas_->GetCurrentMap()); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
  oKf->mpImuPreintegrated = new ORB_SLAM3::IMU::Preintegrated();
  //std::set<ORB_SLAM3::MapPoint*> mvpMapPoints = oKf->GetMapPoints();
  
  for(size_t i; i < rKf->mvp_map_points.size(); ++i)
  {
    orbslam3_interfaces::msg::MapPoint mp = rKf->mvp_map_points[i];
    ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(&mp, oKf, mpOrbKeyFrames);
    oMP->UpdateMap(mpAtlas_->GetCurrentMap()); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();   
    oKf->AddMapPoint(oMP, i);
    mpOrbMapPoints[mp.mn_id] = oMP;
  }
  

  mpOrbKeyFrames.insert(std::make_pair(oKf->mnId, oKf));
  RCLCPP_INFO(this->get_logger(), "Keyframe with ID '%d is converted.", oKf->mnId); 
  //mpLocalMapper_->InsertKeyframeFromRos(oKf);
  //mpAtlas_->AddKeyFrame(oKf, true);
  
  //RCLCPP_INFO(this->get_logger(), "KF added to atlas."); 
}

/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM) {
  if(rM->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  RCLCPP_INFO(this->get_logger(), "Got a new map, id: %d", rM->mn_id);
  ORB_SLAM3::Map* opM = Converter::MapConverter::RosMapToOrbMap(rM, mpOrbKeyFrames);
  mpOrbMaps.insert(std::make_pair(opM->GetId(), opM));

  RCLCPP_INFO(this->get_logger(), "Map with ID '%d is converted.", opM->GetId()); 
  //mpLocalMapper_->InsertKeyframeFromRos(oKf);
  //mpAtlas_->AddKeyFrame(oKf, true);
}


/* ACTION SUBSCRIPTION CALLBACKS */

/*        ATLAS - ACTION        */
void SlamWrapperNode::GrabAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rAA) {
  if(rAA->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  mvpAtlasRosActions.push_back(rAA);
  mRosActionMap.push_back(std::make_tuple(0, mvpAtlasRosActions.size()-1));  
}

/*        KEYFRAME - ACTION        */
void SlamWrapperNode::GrabKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rKF) {
  
  if(rKF->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;

  //PerformKeyFrameAction(rKF);
  mvpKfRosActions.push_back(rKF);
  mRosActionMap.push_back(std::make_tuple(1, mvpKfRosActions.size()-1));
}

/*        KEYFRAME DATABASE - ACTION        */
void SlamWrapperNode::GrabKFDBAction(const orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr rKfdbA) { 
  //RCLCPP_INFO(this->get_logger(), "Received new KFDB action.");
  if(rKfdbA->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  mvpKFDBRosActions.push_back(rKfdbA);
  mRosActionMap.push_back(std::make_tuple(2, mvpKFDBRosActions.size()-1));
  
}



/* ORB ACTION EXECUTORS */

/*        ATLAS - ACTION        */
bool SlamWrapperNode::PerformAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rAA) {

  int actionId = Parser::Action::parseAtlasAction(rAA, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);

  
  if(actionId == -1) 
    return false;
  
  RCLCPP_INFO(this->get_logger(), "Got new atlas action %d.", actionId);
  
  if (actionId==0)  mpAtlas_->AddMapPoint(mpOrbMapPoints[rAA->add_map_point_id], true);
  if (actionId==1)  mpAtlas_->ChangeMap(mpOrbMaps[rAA->change_map_to_id], true);
  if (actionId==2)  mpAtlas_->AddKeyFrame(mpOrbKeyFrames[rAA->add_kf_id], true);
  if (actionId==3)  mpAtlas_->SetMapBad(mpOrbMaps[rAA->set_map_bad_id], true);
  if (actionId==4)  mpAtlas_->CreateNewMap(true);
  if (actionId==5)  mpAtlas_->InformNewBigChange(true);
  if (actionId==6)  mpAtlas_->clearMap(true);
  if (actionId==7)  mpAtlas_->clearAtlas(true);
  if (actionId==8)  mpAtlas_->RemoveBadMaps(true);
  if (actionId==9)  mpAtlas_->SetInertialSensor(true);  
  if (actionId==10) mpAtlas_->SetImuInitialized(true);
  
  return true;
}

/*        KEYFRAME - ACTION        */
bool SlamWrapperNode::PerformKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rKF) {

  int actionId = Parser::Action::parseKFAction(rKF, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);
  if(actionId == -1) 
    return false;
  
  RCLCPP_INFO(this->get_logger(), "Got new KeyFrame action=%d", actionId);
  
  ORB_SLAM3::KeyFrame* kf_ = mpOrbKeyFrames[rKF->kf_id];
  //RCLCPP_INFO(this->get_logger(), "kf id=%d", kf_->mnId);
  
  // Change to switch statement and create enum for the actionids
  if (actionId==0)  kf_->SetPose(Converter::RosToCpp::PoseToSophusSE3f(rKF->set_pose), true);
  if (actionId==1)  kf_->SetVelocity(Converter::RosToCpp::Vector3ToEigenVector3f(rKF->set_velocity), true);
  if (actionId==2)  kf_->AddConnection(mpOrbKeyFrames[rKF->add_connection_id], rKF->add_connection_weight, true);
  if (actionId==3)  kf_->AddMapPoint(mpOrbMapPoints[rKF->add_map_point_mp_id], rKF->add_map_point_vector_idx, true);
  if (actionId==4)  kf_->EraseMapPointMatch(rKF->erase_map_point_vector_idx, true);
  if (actionId==5)  kf_->EraseMapPointMatch(mpOrbMapPoints[rKF->erase_map_point_id], true);
  if (actionId==6)  kf_->ReplaceMapPointMatch(rKF->replace_map_point_vector_idx, mpOrbMapPoints[rKF->replace_map_point_id], true);
  if (actionId==7)  kf_->AddChild(mpOrbKeyFrames[rKF->add_child_id], true);
  if (actionId==8)  kf_->EraseChild(mpOrbKeyFrames[rKF->erase_child_id], true);
  if (actionId==9)  kf_->ChangeParent(mpOrbKeyFrames[rKF->change_parent_id], true);  
  if (actionId==10) kf_->AddLoopEdge(mpOrbKeyFrames[rKF->add_loop_edge_id], true);
  if (actionId==11) kf_->AddMergeEdge(mpOrbKeyFrames[rKF->add_merge_edge_id], true);
  if (actionId==12) kf_->EraseConnection(mpOrbKeyFrames[rKF->erase_connection_id], true);
  if (actionId==13) kf_->SetNewBias(Converter::RosToOrb::RosBiasToOrbImuBias(rKF->set_new_bias), true);
  if (actionId==14) kf_->UpdateMap(mpOrbMaps[rKF->update_map_id], true);
  if (actionId==15) kf_->ComputeBoW(true);
  if (actionId==16) kf_->UpdateBestCovisibles(true);
  if (actionId==17) kf_->UpdateConnections(true, true);
  if (actionId==18) kf_->SetFirstConnection(true, true);
  if (actionId==19) kf_->SetNotErase(true);
  if (actionId==20) kf_->SetErase(true);
  if (actionId==21) kf_->SetBadFlag(true);

  return true;
}

/*        KEYFRAME - ACTION        */
bool SlamWrapperNode::PerformKFDBAction(const orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr rKfdb) {

  int actionId = Parser::Action::parseKFDBAction(rKfdb, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);
  if(actionId == -1) 
    return false;
  
  RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d", actionId);
  
  if(actionId==0) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->add_kf_id];
    mpKfDb_->add(pKf, true);
  }

  if(actionId==1) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->erase_kf_id];
    mpKfDb_->erase(pKf, true);
  }

  if(actionId==2) {
    mpKfDb_->clear(true);
  }

  if(actionId==3) {
    ORB_SLAM3::Map* pMap = mpOrbMaps[rKfdb->clear_map_id]; 
    mpKfDb_->clearMap(pMap, true);
  }

  if(actionId==4) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->detect_candidates_kf_id];
    
    float minScore = rKfdb->detect_candidates_min_score;
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpLoopCands;
    for(size_t id : rKfdb->detect_candidates_loop_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpLoopCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }

    std::vector<ORB_SLAM3::KeyFrame*> mvpMergeCands;
    for(size_t id : rKfdb->detect_candidates_merge_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpMergeCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }

    mpKfDb_->DetectCandidates(pKf, minScore, mvpLoopCands, mvpMergeCands, true);
  }


  if(actionId==5) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->detect_best_candidates_kf_id];
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpLoopCands;
    for(size_t id : rKfdb->detect_best_candidates_loop_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpLoopCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }

    std::vector<ORB_SLAM3::KeyFrame*> mvpMergeCands;
    for(size_t id : rKfdb->detect_best_candidates_merge_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpMergeCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }
  
    int nMinWords = rKfdb->detect_best_candidates_n_min_words;

    mpKfDb_->DetectBestCandidates(pKf, mvpLoopCands, mvpMergeCands, nMinWords, true);
  }

  if(actionId==6) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->detect_n_best_candidates_kf_id];
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpLoopCands;
    for(size_t id : rKfdb->detect_n_best_candidates_loop_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpLoopCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }

    std::vector<ORB_SLAM3::KeyFrame*> mvpMergeCands;
    for(size_t id : rKfdb->detect_n_best_candidates_merge_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpMergeCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }
  
    int nNumCands = rKfdb->detect_n_best_candidates_n_num_cands;

    RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d Just before function", actionId);
    mpKfDb_->DetectNBestCandidates(pKf, mvpLoopCands, mvpMergeCands, nNumCands, true);
    RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d Just after function", actionId);
  }

  return true;
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
  
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrameDatabase/Actions");
  kf_db_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrameDatabaseActions>(
      "/KeyFrameDatabase/Actions", 
      10);
}

void SlamWrapperNode::CreateSubscribers() {
    
  /* KF */  
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
  m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
      "KeyFrame",
      50,
      std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));
  
  /* Map */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
  m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
      "Map",
      50,
      std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));

  /* KF Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame/Actions");
  m_kf_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrameActions>(
      "KeyFrame/Actions",
      500,
      std::bind(&SlamWrapperNode::GrabKeyFrameAction, this, std::placeholders::_1));
  
  /* KeyFrame Database Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrameDatabase/Actions");
  m_kfdb_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrameDatabaseActions>(
      "KeyFrameDatabase/Actions",
      500,
      std::bind(&SlamWrapperNode::GrabKFDBAction, this, std::placeholders::_1));
  
  /* Atlas Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Atlas/Actions");
  m_atlas_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::AtlasActions>(
      "Atlas/Actions",
      500,
      std::bind(&SlamWrapperNode::GrabAtlasAction, this, std::placeholders::_1));
  
}
