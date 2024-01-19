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
// - If map not found when kf received, create it.
SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam) : Node("SlamWrapperNode") {
  
  const char* systemId = std::getenv("SLAM_SYSTEM_ID");
  
  if (systemId != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Initializing SLAM Wrapper Node. System ID=%s", systemId);
  } else {
    
    RCLCPP_FATAL(this->get_logger(), "Initializing SLAM Wrapper Node. System ID not found. Set up SLAM_SYSTEM_ID before continuing, exitting...");
    exit(1);
  }

  m_SLAM = pSLAM;
  mpTracker_ = m_SLAM->GetTrackerPtr();
  mpLocalMapper_ = m_SLAM->GetMapperPtr();
  mpAtlas_ = m_SLAM->GetAtlas();
  mpKfDb_ = m_SLAM->GetKeyFrameDatabase();
  
  for(const auto& mpCam : mpAtlas_->GetAllCameras())
  {
    mpOrbCameras[mpCam->GetId()] = mpCam;
  }

  
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
    
    action_check_timer_->cancel(); 
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
  //m_SLAM->Shutdown();
  // Stop all threads
//  std_msgs::msg::Bool endMsg;
  //endMsg.data = true;
  //end_publisher_->publish(endMsg);
  //RCLCPP_FATAL(this->get_logger(), "Sent msg to destroy other nodes in the network... Shutting down..");

}


void SlamWrapperNode::checkForNewActions() {
  

  //if(mpUnprocOrbMapPoints.size() > 0)
  //{
  //  // Check map points
  //  std::cout << "Number of unprocessed MapPoints before=" << mpUnprocOrbMapPoints.size() << ", ";
  //  for (auto it = mpUnprocOrbMapPoints.begin(); it != mpUnprocOrbMapPoints.end();)
  //  {
  //    unique_lock<mutex> lock(mMutexNewKF);

  //    bool bUnprocessed = false;
  //    auto& mtORMP = it->second;
  //    ORB_SLAM3::MapPoint* mopMp = std::get<0>(mtORMP);
  //    orbslam3_interfaces::msg::MapPoint::SharedPtr mrpMp = std::get<1>(mtORMP);
  //    Converter::MapPointConverter::FillMapPointData(mopMp, mrpMp, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

  //    if(!bUnprocessed)
  //    {
  //      it = mpUnprocOrbMapPoints.erase(it);
  //    } else {
  //      ++it;
  //    }
  //  }
  //  std::cout << "and after=" << mpUnprocOrbMapPoints.size() << std::endl;
  //}


  if(!mpUnprocOrbKeyFrames.empty()) {
    
    std::mutex mMutex;
    std::lock_guard<std::mutex> lock(mMutex);
    // Check if variables can be filled with new data
    std::cout << "STATS: # KF ptrs=" << mpOrbKeyFrames.size() << ", # MP ptrs=" << mpOrbMapPoints.size() << ", # CAM ptrs=" << mpOrbCameras.size()  << ", #Map ptrs=" << mpOrbMaps.size() << std::endl;
    std::cout << "Current Map: ID=" << mpAtlas_->GetCurrentMap()->GetId() << ", #KFs=" << mpAtlas_->GetCurrentMap()->KeyFramesInMap() << ", #MPs=" << mpAtlas_->GetCurrentMap()->MapPointsInMap() << std::endl;
    std::cout << std::endl;

    std::cout << "Number of unprocessed KeyFrames before=" << mpUnprocOrbKeyFrames.size() << ", ";
    for (auto it = mpUnprocOrbKeyFrames.begin(); it != mpUnprocOrbKeyFrames.end();)
    {

      //unique_lock<mutex> lock(mMutexNewKF);
      bool bKFUnprocessed = false;
      auto& mtORKf = it->second;
      ORB_SLAM3::KeyFrame* mopKf = std::get<0>(mtORKf);
      orbslam3_interfaces::msg::KeyFrame::SharedPtr mrpKf = std::get<1>(mtORKf);
      
      //std::cout << "Before mp postloads" << std::endl; 
      for(const long int mnId : mrpKf->mv_backup_map_points_id)
      {
        if(mnId == -1) continue; 
        mpOrbMapPoints[static_cast<unsigned long int>(mnId)]->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, &bKFUnprocessed);
        if(bKFUnprocessed) {
          std::cout << "Map Point is unprocessed, continuing..." << std::endl;
          //mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
          break;
        }
      }

      if(bKFUnprocessed) 
      {
        ++it;
        continue;
      }
      
      mopKf->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed);
      if(!bKFUnprocessed){
        //mpAtlas_->AddKeyFrame(mopKf);
        mpKfDb_->add(mopKf);

        //for(const auto& mpMP : mopKf->GetMapPoints()) {
        //  mpAtlas_->AddMapPoint(mpMP);
        //}

        mspKFsReadyForLM.insert(mopKf->mnId);
        it = mpUnprocOrbKeyFrames.erase(it);
        
      } else {
        std::cout << "KeyFrame is unprocessed (All MPs are good), continuing..." << std::endl;
        ++it;
      }
      //GrabKeyFrame(mrpKf);
      //Converter::KeyFrameConverter::FillKeyFrameData(mopKf, mrpKf, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

    }
    std::cout << "and after=" << mpUnprocOrbKeyFrames.size() << std::endl;
  }

  //TODO: Add somekind of logic so that there can be unprocessed keyframes but not any relevant ones.
  if(!mspKFsReadyForLM.empty() && mpLocalMapper_->AcceptKeyFrames() && mpUnprocOrbKeyFrames.empty())
  {
    std::mutex mMutex;
    std::lock_guard<std::mutex> lock(mMutex);
    
    while(!mspKFsReadyForLM.empty())
    {
      if(!mpLocalMapper_->AcceptKeyFrames())
        break;
      long unsigned int id=*mspKFsReadyForLM.begin();
      mpLocalMapper_->InsertKeyframeFromRos(mpOrbKeyFrames[id]);
      mspKFsReadyForLM.erase(mspKFsReadyForLM.begin());

    }
  }

  if(mspKFsReadyForLM.empty() && mpUnprocOrbKeyFrames.empty()) 
    action_check_timer_->cancel();

 
}

/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  
  unique_lock<mutex> lock(mMutexNewKF);
  
  RCLCPP_INFO(this->get_logger(), "Publishing a new KeyFrame with id: %d. Stats: #MPs=%d, Map ID=%d", pKf->mnId, pKf->GetMapPoints().size(), pKf->GetMap()->GetId());

  for(const auto& cam : mpAtlas_->GetAllCameras())
  {
    if(cam && cam != nullptr)
      mspCameras.insert(cam);
  }
  
  pKf->GetMap()->PreSave(mspCameras);
  
  mpOrbKeyFrames[pKf->mnId] = pKf;

  for(ORB_SLAM3::MapPoint* pMP : pKf->GetMapPoints())
  {
    mpOrbMapPoints[pMP->mnId] = pMP;
  }

  orbslam3_interfaces::msg::KeyFrame msgKf = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKf);
  
  msgKf.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  RCLCPP_INFO(this->get_logger(), "Publishing keyframe with id: %d. KF Map Id=%d == Current Map Id=%d", pKf->mnId, pKf->GetMap()->GetId(), mpAtlas_->GetCurrentMap()->GetId());
  keyframe_publisher_->publish(msgKf);
}

/*        Map        */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
  
  unique_lock<mutex> lock(mMutexUpdateMap);

  RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
  RCLCPP_INFO(this->get_logger(), "Map Stats : #KFs=%d, #MPs=%d, #RefMPs=%d", pM->KeyFramesInMap(), pM->MapPointsInMap(), pM->GetReferenceMapPoints().size());
  
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
  delete pMp;
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
  std::lock_guard<std::mutex> lock(mMutexNewKF);
  
  if(rKf->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;

  //if(!mpLocalMapper_->AcceptKeyFrames())
  //  return;

  RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d, for a map: %d", rKf->mn_id, rKf->mp_map_id);
  
  if(mpOrbMaps.find(rKf->mp_map_id) == mpOrbMaps.end())
  {
    RCLCPP_INFO(this->get_logger(), "NEW map created w/ KF.");
    
    mpAtlas_->CreateNewMap();
    for(ORB_SLAM3::Map* pM : mpAtlas_->GetAllMaps())
    {
      mpOrbMaps[pM->GetId()] = pM;
    }
  }

  ORB_SLAM3::KeyFrame* oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf); 
  
  oKf->UpdateMap(mpOrbMaps[rKf->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
  
  //if((oKf->mnId >= 1 || mpOrbMaps[rKf->mp_map_id]->KeyFramesInMap() >= 1))
  //{
  //  if(!mpLocalMapper_->NeedNewKeyFrame(oKf))
  //  {
  //    delete oKf;
  //    return;
  //  }
  //}

  mpOrbKeyFrames[oKf->mnId] = oKf;

  
  for(size_t i = 0; i < rKf->mvp_map_points.size(); i++)
  {
    orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rKf->mvp_map_points[i]);
    if(mpOrbMapPoints.find(mp->mn_id) == mpOrbMapPoints.end())
    {
      ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, static_cast<long int>(oKf->mnId));
      
      mpOrbMapPoints[oMP->mnId] = oMP;
      oMP->UpdateMap(mpOrbMaps[mp->mp_map_id]);
    }
  }

  //std::cout << "after converting" << std::endl; 
  
  oKf->SetORBVocabulary(m_SLAM->GetORBVocabulary());
  oKf->SetKeyFrameDatabase(mpKfDb_);
  oKf->UpdateMap(mpOrbMaps[rKf->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
  
  bool bKFUnprocessed = false;

  //std::cout << "Before mp postloads #MPs=" << rKf->mv_backup_map_points_id.size() << ", [0]=" << rKf->mv_backup_map_points_id[0]<< std::endl; 
  for(const auto& mnId : rKf->mv_backup_map_points_id)
  {
    if(mnId == -1) continue;
    mpOrbMapPoints[static_cast<unsigned long int>(mnId)]->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, &bKFUnprocessed);
    if(bKFUnprocessed) {
      std::cout << "Map Point if unprocessed, continuing..." << std::endl;
      
      if (action_check_timer_->is_canceled())
      {
        // Timer is not active
        // Perform actions or start the timer if needed
        action_check_timer_->reset();
      
      }
      mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
      return;
    }
  }

  //std::cout << "after mp postloads" << std::endl;
  
  oKf->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed);
  
  if(!bKFUnprocessed){
    //mpAtlas_->AddKeyFrame(oKf);
    mpKfDb_->add(oKf);

    //for(const auto& mpMP : oKf->GetMapPoints()) {
    //  if(!mpAtlas_->CheckIfMapPointInMap(mpMP) && !mpMP->isBad()) mpAtlas_->AddMapPoint(mpMP);
    //}

    if (action_check_timer_->is_canceled())
    {
      // Timer is not active
      // Perform actions or start the timer if needed
      action_check_timer_->reset();
    }
    mspKFsReadyForLM.insert(oKf->mnId);
  } else {
    std::cout << "KeyFrame is unprocessed (All MPs are good), continuing..." << std::endl;
    
    if (action_check_timer_->is_canceled())
    {
      // Timer is not active
      // Perform actions or start the timer if needed
      action_check_timer_->reset();
    }
    mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Got a new keyframe. ID=%d. Stats: #MPs=%d, Map ID=%d == Current Map ID=%d", oKf->mnId, oKf->GetNumberMPs(), oKf->GetMap()->GetId(), mpAtlas_->GetCurrentMap()->GetId());
  //std::set<ORB_SLAM3::MapPoint*> mvpMapPoints = oKf->GetMapPoints();
  //std::cout << "after cameras" << std::endl; 


  //auto itLowerBound = mpUnprocOrbKeyFrames.lower_bound(oKf->mnId); 

  //if(bKFUnprocessed || itLowerBound != mpUnprocOrbKeyFrames.begin()){
  //  RCLCPP_INFO(this->get_logger(), "*##*#*#*#*#*#*#*###**#*#*#* Keyframe with ID=%d is unprocessed. *#*####*#*#*#**#*#*#*#*#", oKf->mnId);
  //  mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
  //  //mpActionChecker->InsertUnprocessedKeyFrame(oKf, rKf);
  //} else {

  //  bool bMPUnprocessed = false;

  //  
  //  std::cout << "number of map points in kf " << rKf->mvp_map_points.size() << std::endl;
  //  ORB_SLAM3::Map* kfMap;
  //  for(size_t i = 0; i < rKf->mvp_map_points.size(); i++)
  //  {
  //    orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rKf->mvp_map_points[i]);
  //    ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bMPUnprocessed);
  //    
  //    oMP->UpdateMap(mpOrbMaps[mp->mp_map_id]);
  //    oKf->AddMapPoint(oMP, i);
  //    
  //    if(bMPUnprocessed) {
  //      RCLCPP_INFO(this->get_logger(), "*##*#*#*#*#*#*#*###**#*#*#* MapPoint with ID=%d is unprocessed. *#*####*#*#*#**#*#*#*#*#", oMP->mnId);
  //      mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
  //      bKFUnprocessed=true;
  //    }

  //    mpOrbMapPoints[oMP->mnId] = oMP;
  //  }

  //  
  //  if(!bKFUnprocessed) 
  //  {
  //    //mpAtlas_->AddKeyFrame(oKf); 
  //    for(const auto& mp : oKf->GetMapPoints())
  //    {
  //      mpAtlas_->AddMapPoint(mp);
  //    }

  //    std::cout << "##################### number of map points in map " << mpOrbMaps[oKf->mnOriginMapId]->GetAllMapPoints().size() << ", " << mpOrbMaps[oKf->mnOriginMapId]->MapPointsInMap() << ", map id=" << oKf->mnOriginMapId  << std::endl; 
  //    std::cout << "map points in map " << mpAtlas_->MapPointsInMap() << std::endl; 
  //    RCLCPP_INFO(this->get_logger(), "Keyframe with ID '%d is converted, with map id=%d.", oKf->mnId, oKf->mnOriginMapId); 
  //    mpUnprocOrbKeyFrames.erase(oKf->mnId);

  //    if(mpLocalMapper_->AcceptKeyFrames()) {
  //      mpLocalMapper_->InsertKeyframeFromRos(oKf);
  //    }
  //
  //  } 

  //}
}
/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM) {
  if(rM->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  unique_lock<mutex> lock(mMutexUpdateMap);

  RCLCPP_INFO(this->get_logger(), "Got a new map, id=%d. Stats: #KFs=%d, #MPs=%d", rM->mn_id, rM->msp_keyframes.size(), rM->msp_map_points.size());
  
  if(mpTracker_->IsMapUpToDate())
  {
    std::cout << "Map is up to date, skipping update.." << std::endl;
    return;
  }
  
  
  RCLCPP_INFO(this->get_logger(), "Inform tracking that Map update is being executed.");
  mpTracker_->LocalMapIsUpdating(true); 
  //if(mpOrbMaps[rM->mn_id]->KeyFramesInMap() < 2)
  //{
  //  std::cout << "Map is still initializing, skipping update.." << std::endl;
  //  return;
  //}
  
  //bool bUnprocessed = false;
  ORB_SLAM3::Map* opM = Converter::MapConverter::RosMapToOrbMap(rM);

  std::cout << "After creating map" << std::endl;
  std::vector<unsigned long int> mvpMPsDone;
  std::cout << "Before loop" << std::endl;
  for(size_t j = 0; j < rM->msp_keyframes.size(); j++)
  {
    orbslam3_interfaces::msg::KeyFrame::SharedPtr rKF = std::make_shared<orbslam3_interfaces::msg::KeyFrame>(rM->msp_keyframes[j]);
    std::cout << "Processing KF=" << rKF->mn_id << std::endl; 
    ORB_SLAM3::KeyFrame* oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKF); 
    
    if(mpOrbKeyFrames.find(oKf->mnId) != mpOrbKeyFrames.end())
      mpOrbKeyFrames.erase(oKf->mnId);
    
    mpOrbKeyFrames[oKf->mnId] = oKf;
    
    for(size_t i = 0; i < rKF->mvp_map_points.size(); i++)
    {
      orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rKF->mvp_map_points[i]);
      //std::cout << "Processing MP=" << mp->mn_id << std::endl; 
      ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, static_cast<long int>(oKf->mnId));
      
      if(mpOrbMapPoints.find(oMP->mnId) != mpOrbMapPoints.end())
        mpOrbMapPoints.erase(oMP->mnId);
      
      mpOrbMapPoints[oMP->mnId] = oMP;
      oMP->UpdateMap(opM);
      mvpMPsDone.push_back(oMP->mnId);
    }
    
    std::cout << "Done with MPs" << std::endl; 
    oKf->SetORBVocabulary(m_SLAM->GetORBVocabulary());
    oKf->SetKeyFrameDatabase(mpKfDb_);
    oKf->UpdateMap(opM); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();  
  }
  std::cout << "Done with KFs" << std::endl; 

  for(size_t j = 0; j < rM->msp_map_points.size(); j++)
  {
    unsigned long int targetId = rM->msp_map_points[j].mn_id;
    
    auto it = std::find(mvpMPsDone.begin(), mvpMPsDone.end(), targetId);

    if(it != mvpMPsDone.end())
    {
      //std::cout << "MapPoint is not processed." << std::endl;
      orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rM->msp_map_points[j]);
      
      ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(mp);
      
      if(mpOrbMapPoints.find(oMP->mnId) != mpOrbMapPoints.end())
        mpOrbMapPoints.erase(oMP->mnId);
      
      mpOrbMapPoints[oMP->mnId] = oMP;
      oMP->UpdateMap(opM);
      mvpMPsDone.push_back(oMP->mnId);
    } else {
      std::cout << "MapPoint is already processed." << std::endl;
    }

  }
  
  if(mpOrbMaps.find(opM->GetId()) != mpOrbMaps.end())
  {
    ORB_SLAM3::Map* mpPrevMap = mpOrbMaps[opM->GetId()];
    
    std::cout << "Reseting Database..." << std::endl;;
    mpKfDb_->clearMap(mpPrevMap);
  }
  
  bool bKFUnprocessed = false;
  opM->PostLoad(mpKfDb_, m_SLAM->GetORBVocabulary(), mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed);
  
  if(bKFUnprocessed) std::cout << "Map with ID=" << opM->GetId() << " is unprocessed!!" << std::endl;
  
  if(mpOrbMaps.find(opM->GetId()) != mpOrbMaps.end())
  {
    std::cout << "Map found, replacing it .. " << std::endl;
    mpAtlas_->ReplaceMap(opM);
    
    mpOrbMaps[opM->GetId()] = opM;
  }

  std::cout << "Done with GrabMap" << std::endl;


  mpTracker_->UpdateFromLocalMapping(opM, mpOrbKeyFrames);

  
}

/*        MapPoint        */
void SlamWrapperNode::GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr rpMp) {
  if(rpMp->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  //RCLCPP_INFO(this->get_logger(), "Got a new mappoint, id: %d", rpMp->mn_id);
  bool bUnprocessed = false;
  ORB_SLAM3::MapPoint* opMp = Converter::MapPointConverter::RosMapPointToOrb(rpMp);

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
      10);
  
  /* MAP */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
  map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
      "/Map", 
      10);
  
  /* MAPPOINT */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /MapPoint");
  map_point_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapPoint>(
      "/MapPoint", 
      100);
  
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
