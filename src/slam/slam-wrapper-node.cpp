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


  // Initialize maps for the SLAM data, unprocessed and processed
  mpUnprocOrbKeyFrames=std::map<long unsigned int, std::tuple<ORB_SLAM3::KeyFrame*, orbslam3_interfaces::msg::KeyFrame::SharedPtr>>();
  mpUnprocOrbMapPoints=std::map<long unsigned int, std::tuple<ORB_SLAM3::MapPoint*, orbslam3_interfaces::msg::MapPoint::SharedPtr>>();
  mpUnprocOrbMaps=std::map<long unsigned int, std::tuple<ORB_SLAM3::Map*, orbslam3_interfaces::msg::Map::SharedPtr>>();

  mpOrbKeyFrames=std::map<long unsigned int, ORB_SLAM3::KeyFrame*>();
  mpOrbMapPoints=std::map<long unsigned int, ORB_SLAM3::MapPoint*>();
  mpOrbMaps=std::map<long unsigned int, ORB_SLAM3::Map*>();
 
  // Add the current map to the maps
  std::vector<ORB_SLAM3::Map*> mpAtlasMaps = mpAtlas_->GetAllMaps();
  for(const auto& map : mpAtlasMaps) {
    mpOrbMaps.insert(std::make_pair(map->GetId(), map));
  }

  CreatePublishers();


  /* Init subscribers */
  if (subscribe_to_slam) { 
    CreateSubscribers();

    action_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SlamWrapperNode::checkForNewActions, this)
    );
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
  if (!mRosActionMap.empty()) {
    RCLCPP_INFO(get_logger(), "Action buffer contains items, try to perform actions.");
    RCLCPP_INFO(get_logger(), "Number of maps=%d", mpOrbMaps.size());
    
    // Check if variables can be filled with new data
    std::cout << "Number of unprocessed KeyFrames before=" << mpUnprocOrbKeyFrames.size() << ", ";
    for (auto it = mpUnprocOrbKeyFrames.begin(); it != mpUnprocOrbKeyFrames.end();)
    {

      bool bUnprocessed = false;
      auto& mtORKf = it->second;
      ORB_SLAM3::KeyFrame* mopKf = std::get<0>(mtORKf);
      orbslam3_interfaces::msg::KeyFrame::SharedPtr mrpKf = std::get<1>(mtORKf);
      Converter::KeyFrameConverter::FillKeyFrameData(mopKf, mrpKf, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

      if(!bUnprocessed)
      {
        it = mpUnprocOrbKeyFrames.erase(it);
      } else {
        ++it;
      }
    }
    std::cout << "and after=" << mpUnprocOrbKeyFrames.size() << std::endl;
   
    // Check map points
    std::cout << "Number of unprocessed MapPoints before=" << mpUnprocOrbMapPoints.size() << ", ";
    for (auto it = mpUnprocOrbMapPoints.begin(); it != mpUnprocOrbMapPoints.end();)
    {

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


    // Perform action if there are new keyframes
    size_t numOfActions = mRosActionMap.size();
    
    std::vector<int> kfIdsToRemove;
    std::vector<int> kfdbIdsToRemove;
    std::vector<int> rosActionIdsToRemove;
    std::vector<int> atlasIdsToRemove;
    std::vector<int> mpIdsToRemove;
    

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
      } else if(categoryIdx==3) {
        orbslam3_interfaces::msg::MapPointActions::SharedPtr mMpRosMsg = mvpMpRosActions[vecIdx];
        
        if(mMpRosMsg==nullptr) continue;
        
        bool success = PerformMapPointAction(mMpRosMsg);
          
        if(success) 
        {
          mpIdsToRemove.push_back(vecIdx);
          rosActionIdsToRemove.push_back(i);
        }
      }
    }

    
    std::cout << std::endl;
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

    RCLCPP_INFO(get_logger(), "MapPoint : Action performed=%d, total number of actions=%d", mpIdsToRemove.size(), mvpMpRosActions.size());
    for(const auto& id : mpIdsToRemove)
    {
      mvpMpRosActions[id] = nullptr;
      //mvpKfRosActions.erase(mvpKfRosActions.begin() + id);
    }

    std::cout << std::endl;
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
  
  //RCLCPP_INFO(this->get_logger(), "Publishing keyframe with id: %d", pKf->mnId);
  keyframe_publisher_->publish(msgKf);
}

/*        Map        */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
  RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
  orbslam3_interfaces::msg::Map msgM = Converter::MapConverter::OrbMapToRosMap(pM);
  msgM.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  map_publisher_->publish(msgM);
}

/*        MapPoint        */
void SlamWrapperNode::publishMapPoint(ORB_SLAM3::MapPoint* pMp) {
  //RCLCPP_INFO(this->get_logger(), "Publishing a new mappoint with id: %d", pMp->mnId);
  orbslam3_interfaces::msg::MapPoint msgMp = Converter::MapPointConverter::ORBSLAM3MapPointToROS(pMp);
  msgMp.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  map_point_publisher_->publish(msgMp);
}



/* ACTION PUBLISHERS */

void SlamWrapperNode::publishMapAction() {
  RCLCPP_INFO(this->get_logger(), "Publishing a map action.");
  //map_action_publisher_->publish(msg);
}

void SlamWrapperNode::publishAtlasAction(orbslam3_interfaces::msg::AtlasActions::SharedPtr aMsg) {
  //RCLCPP_INFO(this->get_logger(), "Publishing Atlas Action.");
  atlas_action_publisher_->publish(*aMsg);
}

void SlamWrapperNode::publishKFAction(orbslam3_interfaces::msg::KeyFrameActions::SharedPtr kfMsg) {
  //RCLCPP_INFO(this->get_logger(), "Publishing KeyFrame Action %d.", kfMsg.kf_id);
  kf_action_publisher_->publish(*kfMsg);
}

void SlamWrapperNode::publishKFDBAction(orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr kfdbMsg) {
  //RCLCPP_INFO(this->get_logger(), "Publishing KeyFrame Database Action.");
  kf_db_action_publisher_->publish(*kfdbMsg);
}

void SlamWrapperNode::publishMapPointAction(orbslam3_interfaces::msg::MapPointActions::SharedPtr mpMsg) {
  //RCLCPP_INFO(this->get_logger(), "Publishing Atlas Action.");
  mp_action_publisher_->publish(*mpMsg);
}






/* OBJECT SUBSCRIPTION CALLBACKS */

/*        KeyFrame        */
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf) {
  if(rKf->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
 
  //RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d", rKf->mn_id);
  bool bUnprocessed = false;
  ORB_SLAM3::KeyFrame* oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf, mpOrbKeyFrames, mpOrbMaps, &bUnprocessed);
  
  oKf->SetORBVocabulary(m_SLAM->GetORBVocabulary());
  oKf->SetKeyFrameDatabase(mpKfDb_);
  oKf->UpdateMap(mpOrbMaps[oKf->mnOriginMapId]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
  oKf->mpImuPreintegrated = new ORB_SLAM3::IMU::Preintegrated();
  //std::set<ORB_SLAM3::MapPoint*> mvpMapPoints = oKf->GetMapPoints();
  
  if(bUnprocessed){
    RCLCPP_INFO(this->get_logger(), "*##*#*#*#*#*#*#*###**#*#*#* Keyframe with ID=%d is unprocessed. *#*####*#*#*#**#*#*#*#*#", oKf->mnId);
    mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
  } 
  
  mpOrbKeyFrames[oKf->mnId] = oKf;
  
  for(size_t i; i < rKf->mvp_map_points.size(); ++i)
  {
    bool bUnprocessed = false;
    orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rKf->mvp_map_points[i]);
    ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);
    oMP->UpdateMap(mpAtlas_->GetCurrentMap()); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();   
    oKf->AddMapPoint(oMP, i);
    if(bUnprocessed) {
      RCLCPP_INFO(this->get_logger(), "*##*#*#*#*#*#*#*###**#*#*#* MapPoint with ID=%d is unprocessed. *#*####*#*#*#**#*#*#*#*#", oMP->mnId);
      mpUnprocOrbMapPoints[oMP->mnId] = std::make_tuple(oMP, mp);
    }
    mpOrbMapPoints[oMP->mnId] = oMP;
  }
  

  RCLCPP_INFO(this->get_logger(), "Keyframe with ID '%d is converted, with map id=%d.", oKf->mnId, oKf->mnOriginMapId); 
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
  }
  mpOrbMapPoints.insert(std::make_pair(opMp->mnId, opMp));

  //RCLCPP_INFO(this->get_logger(), "MapPoint with ID '%d is converted.", opMp->mnId); 
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


/*        MapPoint - ACTION        */
void SlamWrapperNode::GrabMapPointAction(const orbslam3_interfaces::msg::MapPointActions::SharedPtr rMpA) { 
  //RCLCPP_INFO(this->get_logger(), "Received new MapPoint action.");
  if(rMpA->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  mvpMpRosActions.push_back(rMpA);
  mRosActionMap.push_back(std::make_tuple(3, mvpMpRosActions.size()-1));
  
}

/* ORB ACTION EXECUTORS */

/*        ATLAS - ACTION        */
bool SlamWrapperNode::PerformAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rAA) {

  int actionId = Parser::Action::parseAtlasAction(rAA, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);

  
  if(actionId == -1) 
    return false;
  
  //RCLCPP_INFO(this->get_logger(), "Got new atlas action %d.", actionId);
  
  if (actionId==0)  mpAtlas_->AddMapPoint(mpOrbMapPoints[rAA->add_map_point_id], true);
  if (actionId==1)  mpAtlas_->ChangeMap(mpOrbMaps[rAA->change_map_to_id], true);
  if (actionId==2)  mpAtlas_->AddKeyFrame(mpOrbKeyFrames[rAA->add_kf_id], true);
  if (actionId==3)  mpAtlas_->SetMapBad(mpOrbMaps[rAA->set_map_bad_id], true);
  //if (actionId==4)  mpAtlas_->CreateNewMap(true);
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
  
  //RCLCPP_INFO(this->get_logger(), "Got new KeyFrame action=%d", actionId);
  
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
  
  //RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d", actionId);
  
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

    //RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d Just before function", actionId);
    mpKfDb_->DetectNBestCandidates(pKf, mvpLoopCands, mvpMergeCands, nNumCands, true);
    //RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d Just after function", actionId);
  }

  return true;
}


/*    MAPPOINT - ACTION    */
bool SlamWrapperNode::PerformMapPointAction(const orbslam3_interfaces::msg::MapPointActions::SharedPtr rMpA) {

  int actionId = Parser::Action::parseMapPointAction(rMpA, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);
  if(actionId == -1) 
    return false;
  
  //RCLCPP_INFO(this->get_logger(), "Got new MapPoint action=%d", actionId);
  
  ORB_SLAM3::MapPoint* pMp = mpOrbMapPoints[rMpA->mp_id];
  //RCLCPP_INFO(this->get_logger(), "kf id=%d", kf_->mnId);
  
  // Change to switch statement and create enum for the actionids
  if (actionId==0)  pMp->SetWorldPos(Converter::RosToCpp::Vector3ToEigenVector3f(rMpA->set_pose), true);
  if (actionId==1)  pMp->AddObservation(mpOrbKeyFrames[rMpA->add_observation_kf_id], rMpA->add_observation_vector_idx, true);
  if (actionId==2)  pMp->EraseObservation(mpOrbKeyFrames[rMpA->erase_observation_kf_id], true);
  if (actionId==3)  pMp->SetBadFlag(true);
  if (actionId==4)  pMp->Replace(mpOrbMapPoints[rMpA->replace_id], true);
  if (actionId==5)  pMp->IncreaseVisible(rMpA->increase_visibility_n, true);
  if (actionId==6)  pMp->IncreaseFound(rMpA->increase_found_n, true);
  if (actionId==7)  pMp->ComputeDistinctiveDescriptors(true);
  if (actionId==8)  pMp->UpdateNormalAndDepth(true);
  if (actionId==9)  pMp->SetNormalVector(Converter::RosToCpp::Vector3ToEigenVector3f(rMpA->set_normal_vector), true);
  if (actionId==10) pMp->UpdateMap(mpOrbMaps[rMpA->update_map_id], true);  

  return true;


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
      10);
  
  /* MAPPOINT */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /MapPoint");
  map_point_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapPoint>(
      "/MapPoint", 
      1000);
  



  /* MAP ACTIONS */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map/Actions");
  map_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapActions>(
      "/Map/Actions", 
      10);

  /* ATLAS ACTIONS */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Atlas/Actions");
  atlas_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::AtlasActions>(
      "/Atlas/Actions", 
      500);
  
  /* KEYFRAME ACTIONS */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame/Actions");
  kf_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrameActions>(
      "/KeyFrame/Actions", 
      2000);
  
  /* KEYFRAME DATABASE ACTIONS */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrameDatabase/Actions");
  kf_db_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrameDatabaseActions>(
      "/KeyFrameDatabase/Actions", 
      500);
  
  /* MAP POINT ACTIONS */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /MapPoint/Actions");
  mp_action_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapPointActions>(
      "/MapPoint/Actions", 
      5000);


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

  /* Map */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /MapPoint");
  m_map_point_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::MapPoint>(
      "MapPoint",
      1000,
      std::bind(&SlamWrapperNode::GrabMapPoint, this, std::placeholders::_1));
  



  /* KF Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame/Actions");
  m_kf_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrameActions>(
      "KeyFrame/Actions",
      1000,
      std::bind(&SlamWrapperNode::GrabKeyFrameAction, this, std::placeholders::_1));
  
  /* KeyFrame Database Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrameDatabase/Actions");
  m_kfdb_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrameDatabaseActions>(
      "KeyFrameDatabase/Actions",
      50,
      std::bind(&SlamWrapperNode::GrabKFDBAction, this, std::placeholders::_1));
  
  /* Atlas Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Atlas/Actions");
  m_atlas_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::AtlasActions>(
      "Atlas/Actions",
      500,
      std::bind(&SlamWrapperNode::GrabAtlasAction, this, std::placeholders::_1));
  
  /* MapPoint Actions */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /MapPoint/Actions");
  m_mp_action_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::MapPointActions>(
      "MapPoint/Actions",
      5000,
      std::bind(&SlamWrapperNode::GrabMapPointAction, this, std::placeholders::_1));
  


  /* Destroy node */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /destroy");
  end_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "/destroy", 
      10, 
      std::bind(&SlamWrapperNode::endCallback, this, std::placeholders::_1));
}
