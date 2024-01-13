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
        std::chrono::milliseconds(500),
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
    // Check if variables can be filled with new data
    std::cout << "# KF ptrs=" << mpOrbKeyFrames.size() << ", # MP ptrs=" << mpOrbMapPoints.size() << ", # CAM ptrs=" << mpOrbCameras.size() << std::endl;
    std::cout << "Number of unprocessed KeyFrames before=" << mpUnprocOrbKeyFrames.size() << ", ";
    for (auto it = mpUnprocOrbKeyFrames.begin(); it != mpUnprocOrbKeyFrames.end();)
    {

      //unique_lock<mutex> lock(mMutexNewKF);
      bool bKFUnprocessed = false;
      auto& mtORKf = it->second;
      ORB_SLAM3::KeyFrame* mopKf = std::get<0>(mtORKf);
      orbslam3_interfaces::msg::KeyFrame::SharedPtr mrpKf = std::get<1>(mtORKf);
      
      std::cout << "Before mp postloads" << std::endl; 
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
        mpAtlas_->AddKeyFrame(mopKf);
        mpKfDb_->add(mopKf);

        for(const auto& mpMP : mopKf->GetMapPoints()) {
          mpAtlas_->AddMapPoint(mpMP);
        }

        if(mpLocalMapper_->AcceptKeyFrames()) {
          mpLocalMapper_->InsertKeyframeFromRos(mopKf);
          it = mpUnprocOrbKeyFrames.erase(it);
        } else {
          ++it;
        }
        
      } else {
        std::cout << "KeyFrame is unprocessed (All MPs are good), continuing..." << std::endl;
        ++it;
      }
      //GrabKeyFrame(mrpKf);
      //Converter::KeyFrameConverter::FillKeyFrameData(mopKf, mrpKf, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

    }
    std::cout << "and after=" << mpUnprocOrbKeyFrames.size() << std::endl;
  }

 
}

/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  
  unique_lock<mutex> lock(mMutexNewKF);
  
  std::cout << "##### publish kf: number of map points " << pKf->GetMap()->GetAllMapPoints().size() << " and in kf " << pKf->GetMapPoints().size() << " and matches " << pKf->GetMapPointMatches().size() << std::endl;
  

  std::cout << "#KFs=" << mpAtlas_->GetAllKeyFrames().size() << ", #MPs=" << mpAtlas_->GetAllMapPoints().size() << ", #CAMs=" << mpAtlas_->GetAllCameras().size() << std::endl;

  for(const auto& kf : mpAtlas_->GetAllKeyFrames())
  {
    mspKeyFrames.insert(kf);
  }
  
  for(const auto& mp : mpAtlas_->GetAllMapPoints())
  {
    mspMapPoints.insert(mp);
  }
  
  for(const auto& cam : mpAtlas_->GetAllCameras())
  {
    mspCameras.insert(cam);
  }
  
  pKf->PreSave(mspKeyFrames, mspMapPoints, mspCameras);
  
  mpOrbKeyFrames[pKf->mnId] = pKf;

  for(ORB_SLAM3::MapPoint* pMP : pKf->GetMapPoints())
  {
    pMP->PreSave(mspKeyFrames, mspMapPoints);
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
  std::lock_guard<std::mutex> lock(mMutexNewKF);
  
  if(rKf->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;

  if(!mpLocalMapper_->AcceptKeyFrames())
    return;

  RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d", rKf->mn_id);
  

  ORB_SLAM3::KeyFrame* oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf); 
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

  std::cout << "after converting" << std::endl; 
  
  oKf->SetORBVocabulary(m_SLAM->GetORBVocabulary());
  oKf->SetKeyFrameDatabase(mpKfDb_);
  oKf->UpdateMap(mpOrbMaps[rKf->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
  
  bool bKFUnprocessed = false;

  std::cout << "Before mp postloads #MPs=" << rKf->mv_backup_map_points_id.size() << ", [0]=" << rKf->mv_backup_map_points_id[0]<< std::endl; 
  for(const auto& mnId : rKf->mv_backup_map_points_id)
  {
    if(mnId == -1) continue;
    mpOrbMapPoints[static_cast<unsigned long int>(mnId)]->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, &bKFUnprocessed);
    if(bKFUnprocessed) {
      std::cout << "Map Point if unprocessed, continuing..." << std::endl;
      mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
      return;
    }
  }

  std::cout << "after mp postloads" << std::endl;
  
  oKf->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed);
  if(!bKFUnprocessed){
    mpAtlas_->AddKeyFrame(oKf);
    mpKfDb_->add(oKf);

    for(const auto& mpMP : oKf->GetMapPoints()) {
      if(!mpAtlas_->CheckIfMapPointInMap(mpMP)) mpAtlas_->AddMapPoint(mpMP);
    }
  } else {
    std::cout << "KeyFrame is unprocessed (All MPs are good), continuing..." << std::endl;

  }

  //std::set<ORB_SLAM3::MapPoint*> mvpMapPoints = oKf->GetMapPoints();
  std::cout << "after cameras" << std::endl; 


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

  RCLCPP_INFO(this->get_logger(), "Got a new map, id=%d", rM->mn_id);
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
    mpOrbKeyFrames[oKf->mnId] = oKf;
    
    for(size_t i = 0; i < rKF->mvp_map_points.size(); i++)
    {
      orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rKF->mvp_map_points[i]);
      std::cout << "Processing MP=" << mp->mn_id << std::endl; 
      ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, static_cast<long int>(oKf->mnId));
      
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
      std::cout << "MapPoint is not processed." << std::endl;
      orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rM->msp_map_points[j]);
      
      ORB_SLAM3::MapPoint* oMP = Converter::MapPointConverter::RosMapPointToOrb(mp);
      
      mpOrbMapPoints[oMP->mnId] = oMP;
      oMP->UpdateMap(opM);
      mvpMPsDone.push_back(oMP->mnId);
    } else {
      std::cout << "MapPoint is already processed." << std::endl;
    }

  }

  opM->PostLoad(mpKfDb_, m_SLAM->GetORBVocabulary(), mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras);
  
  std::cout << "Map ID=" << opM->GetId() << std::endl;

  if(mpOrbMaps.find(opM->GetId()) != mpOrbMaps.end())
  {
    std::cout << "Map found, replacing it .. " << std::endl;
    mpAtlas_->ReplaceMap(opM);
    mpOrbMaps[opM->GetId()] = opM;
  }
  std::cout << "Done with GrabMap" << std::endl;


  mpTracker_->UpdateFromLocalMapping(opM);

  
  // Get Map Mutex -> Map cannot be changed
  //unique_lock<mutex> lock2(mpMap->mMutexCallBackUpdate);

  //if(!msRelocStatus)
  //{
  //    if(mpMap->KeyFramesInMap() < LOCAL_MAP_SIZE)
  //    {
  //        cout << "log,Tracking::mapCallback,map is still initializing. skip update" << endl;
  //        return;
  //    }

  //    if(mapUpToDate)
  //    {
  //        cout << "log,Tracking::mapCallback,map is up to date. skip update" << endl;
  //        return;
  //    }

  //    // Check if high rate of keyframes are being created
  //    {
  //        // Edge-SLAM: measure
  //        msLastKeyFrameStop = std::chrono::high_resolution_clock::now();
  //        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(msLastKeyFrameStop - msLastKeyFrameStart);
  //        auto dCount = duration.count();

  //        // As the map adds more than LOCAL_MAP_SIZE keyframes, decrease TIME_KF to accept the update sooner
  //        unsigned int divRate = (unsigned)(TIME_KF/((mpMap->KeyFramesInMap()/LOCAL_MAP_SIZE)+1));
  //        if(divRate < 50)
  //            divRate = 0;

  //        if(dCount < divRate)
  //        {
  //            cout << "log,Tracking::mapCallback,map update rejected due to high keyframe rate " << dCount << "ms; KFs in map " << mpMap->KeyFramesInMap() << "; divRate " << divRate << "ms" << endl;
  //            return;
  //        }
  //    }
  //}

  // Receive local-map update from server
  //mapVec.clear();
  //try
  //{
  //    std::stringstream is(msg);
  //    boost::archive::text_iarchive ia(is);
  //    ia >> mapVec;
  //    is.clear();
  //}
  //catch(boost::archive::archive_exception e)
  //{
  //    cout << "log,Tracking::mapCallback,map error: " << e.what() << endl;
  //    return;
  //}

  //// If not in relocalization mode, then discard relocalization map update
  //if((!msRelocStatus) && (mpMap->KeyFramesInMap()>=LOCAL_MAP_SIZE) && (mnMapUpdateLastKFId>10) && (mapVec.size()!=LOCAL_MAP_SIZE))
  //{
  //    cout << "log,Tracking::mapCallback,relocalization map received after successfully relocalizing. map rejected. keyframes in map " << mpMap->KeyFramesInMap() << ". last MU KF id " << mnMapUpdateLastKFId << ". map size " << mapVec.size() << endl;
  //    return;
  //}

  //// Edge-SLAM: debug
  //cout << "log,Tracking::mapCallback,accept and process map update " << ++mapCallbackCount << endl;




  /* ############################################################ */





  //// Keep current reference keyframe Id, and reset refKFSet
  //long unsigned int refId = mpReferenceKF->mnId; // mpReferenceKF is from tarcking
  //// refKFSet = false; Edge-slam variable

  //// Before clearing the map, we should retrieve the ids of map points within last frame
  //vector<unsigned long int> lastFrame_points_ids;
  //vector<bool> lastFrame_points_availability;
  //vector<unsigned long int> mvpLocalMapPoints_ids;
  //for(int i =0; i<mLastFrame.N; i++) // mLastFrame is from tracking
  //{
  //    MapPoint* pMP = mLastFrame.mvpMapPoints[i];

  //    if(pMP)
  //    {
  //        lastFrame_points_ids.push_back(pMP->mnId);
  //        lastFrame_points_availability.push_back(true);
  //    }
  //    else
  //    {
  //        lastFrame_points_availability.push_back(false);
  //    }
  //}
  //for (unsigned int i = 0 ; i < mvpLocalMapPoints.size(); i++)
  //{
  //    mvpLocalMapPoints_ids.push_back(mvpLocalMapPoints[i]->mnId);
  //}

  //

  //// Reset tracking thread to update it using a new local-map
  //MUReset(); // From tracking

  //// Reconstruct keyframes loop
  //// For every keyframe, check its mappoints, then add them to tracking local-map
  //// Add keyframe to tracking local-map
  //for(int i=0; i<(int)mapVec.size(); i++) // mapVec is the data, in this case ros2 msg
  //{
  //    // Reconstruct keyframe
  //    KeyFrame *tKF = new KeyFrame();
  //    //{
  //    //    try
  //    //    {
  //    //        std::stringstream iis(mapVec[i]);
  //    //        boost::archive::text_iarchive iia(iis);
  //    //        iia >> tKF;
  //    //        iis.clear();
  //    //    }
  //    //    catch(boost::archive::archive_exception e)
  //    //    {
  //    //        cout << "log,Tracking::mapCallback,keyframe error: " << e.what() << endl;

  //    //        // Clear
  //    //        tKF = static_cast<KeyFrame*>(NULL);
  //    //        continue;
  //    //    }
  //    //}

  //    // Set keyframe fields
  //    tKF->setORBVocab(mpORBVocabulary);
  //    tKF->setMapPointer(mpMap);
  //    tKF->setKeyFrameDatabase(mpKeyFrameDB);
  //    tKF->ComputeBoW();

  //    // Get keyframe's mappoints
  //    vector<MapPoint*> vpMapPointMatches = tKF->GetMapPointMatches();

  //    // Iterate through current keyframe's mappoints and double check them
  //    for(size_t i=0; i<vpMapPointMatches.size(); i++)
  //    {
  //        MapPoint* pMP = vpMapPointMatches[i];
  //        if(pMP)
  //        {
  //            if(!pMP->isBad())
  //            {
  //                // If tracking id is set
  //                if(pMP->trSet)
  //                {
  //                    MapPoint* pMPMap = mpMap->RetrieveMapPoint(pMP->mnId, true);

  //                    if(pMPMap != NULL)
  //                    {
  //                        // Replace keyframe's mappoint pointer to the existing one in tracking local-map
  //                        tKF->AddMapPoint(pMPMap, i);

  //                        // Add keyframe observation to the mappoint
  //                        pMPMap->AddObservation(tKF, i);

  //                        // Delete duplicate mappoint
  //                        delete pMP;
  //                    }
  //                    else
  //                    {
  //                        // Add keyframe's mappoint to tracking local-map
  //                        mpMap->AddMapPoint(pMP); // mpMap needs to be selected from Atlas

  //                        // Add keyframe observation to the mappoint
  //                        pMP->AddObservation(tKF, i);
  //                        pMP->setMapPointer(mpMap); // We are not sending the map pointer in marshalling
  //                        pMP->SetReferenceKeyFrame(tKF);
  //                    }
  //                }
  //                else if(pMP->lmSet)     // If tracking id is not set, but local-mapping id is set
  //                {
  //                    MapPoint* pMPMap = mpMap->RetrieveMapPoint(pMP->lmMnId, false);

  //                    if(pMPMap != NULL)
  //                    {
  //                        // Replace keyframe's mappoint pointer to the existing one in tracking local-map
  //                        tKF->AddMapPoint(pMPMap, i);

  //                        // Add keyframe observation to the mappoint
  //                        pMPMap->AddObservation(tKF, i);

  //                        // Delete duplicate mappoint
  //                        delete pMP;
  //                    }
  //                    else
  //                    {
  //                        // Assign tracking id
  //                        pMP->AssignId(true);

  //                        // Add keyframe's mappoint to tracking local-map
  //                        mpMap->AddMapPoint(pMP);

  //                        // Add keyframe observation to the mappoint
  //                        pMP->AddObservation(tKF, i);
  //                        pMP->setMapPointer(mpMap); // We are not sending the map pointer in marshalling
  //                        pMP->SetReferenceKeyFrame(tKF);
  //                    }
  //                }
  //            }
  //        }
  //    }

  //    // Add keyframe to tracking local-map
  //    mpMap->AddKeyFrame(tKF);

  //    // Add Keyframe to database
  //    mpKeyFrameDB->add(tKF);

  //    // Set RefKF to previous RefKF if it is part of the map update
  //    if(tKF->mnId == refId)
  //    {
  //        mpReferenceKF = tKF; //mpReferenceKF is from tracking
  //        mLastFrame.mpReferenceKF = tKF; // mLastFrame is from tracking
  //        refKFSet = true; // Edge-slam var in tracking
  //    }

  //    // Clear
  //    tKF = static_cast<KeyFrame*>(NULL);
  //    vpMapPointMatches.clear();
  //}

  //// Get all keyframes in tracking local-map
  //vector<KeyFrame*> vpKeyFrames = mpMap->GetAllKeyFrames();

  //// Initialize Reference KeyFrame and other KF variables
  //if(vpKeyFrames.size() > 0)
  //{
  //    if(!refKFSet)
  //        mpReferenceKF = vpKeyFrames[0]; // from tracking
  //    else
  //    {
  //        if(mpReferenceKF->mnId < vpKeyFrames[0]->mnId) // from tracking
  //        {
  //            mpReferenceKF = vpKeyFrames[0]; // from tracking
  //            refKFSet = false; // edge-slam tracking
  //        }
  //    }

  //    mnLastKeyFrameId = vpKeyFrames[0]->mnFrameId; // from tracking
  //    mpLastKeyFrame = vpKeyFrames[0]; // from tracking 
  //    mnMapUpdateLastKFId = vpKeyFrames[0]->mnId; // edge-slam tracking
  //}

  //// Edge-SLAM: debug
  //cout << "log,Tracking::mapCallback,keyframes in update: ";

  //// Iterate through keyframes and reconstruct connections
  //for (std::vector<KeyFrame*>::iterator it=vpKeyFrames.begin(); it!=vpKeyFrames.end(); ++it)
  //{
  //    KeyFrame* pKFCon = *it;

  //    pKFCon->ReconstructConnections();

  //    // Edge-SLAM: debug
  //    cout << pKFCon->mnId << " ";

  //    // If RefKF has lower id than current KF, then set it to that KF
  //    if(mpReferenceKF->mnId < pKFCon->mnId)
  //    {
  //        mpReferenceKF = pKFCon;
  //        refKFSet = false;
  //    }

  //    // Update other KF variables
  //    if(mnMapUpdateLastKFId < pKFCon->mnId)
  //    {
  //        mnLastKeyFrameId = pKFCon->mnFrameId;
  //        mpLastKeyFrame = pKFCon;
  //        mnMapUpdateLastKFId = pKFCon->mnId;
  //    }
  //}

  //// Edge-SLAM: debug
  //cout << endl;

  //// Set current-frame RefKF
  //mCurrentFrame.mpReferenceKF = mpReferenceKF;

  //// Get all map points in tracking local-map
  //vector<MapPoint*> vpMapPoints = mpMap->GetAllMapPoints();

  //// Iterate through all mappoints and SetReferenceKeyFrame
  //for (std::vector<MapPoint*>::iterator it=vpMapPoints.begin(); it!=vpMapPoints.end(); ++it)
  //{
  //    MapPoint* rMP = *it;

  //    if((unsigned)rMP->mnFirstKFid == rMP->GetReferenceKeyFrame()->mnId)
  //        continue;

  //    KeyFrame* rKF = mpMap->RetrieveKeyFrame(rMP->mnFirstKFid);

  //    if(rKF)
  //        rMP->SetReferenceKeyFrame(rKF);
  //}

  //// Updating mLastFrame
  //for(int i =0; i<mLastFrame.N; i++)
  //{
  //    if(lastFrame_points_availability[i])
  //    {
  //        MapPoint* newpMP = mpMap->RetrieveMapPoint(lastFrame_points_ids[i], true);

  //        if (newpMP)
  //        {
  //            mLastFrame.mvpMapPoints[i] = newpMP;
  //        }
  //        else
  //        {
  //            mLastFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
  //        }
  //    }
  //}

  //// We should update mvpLocalMapPoints for viewer
  //mvpLocalMapPoints.clear(); // from tracking = mpAtlas->GetAllMapPoints()
  //for (unsigned int i = 0 ; i < mvpLocalMapPoints_ids.size(); i++)// from tracking = mpAtlas->GetAllMapPoints()
  //{
  //    MapPoint* pMP = mpMap->RetrieveMapPoint(mvpLocalMapPoints_ids[i], true);// from tracking = mpAtlas->GetAllMapPoints()
  //    if (pMP)
  //    {
  //        mvpLocalMapPoints.push_back(pMP);// from tracking = mpAtlas->GetAllMapPoints()
  //    }
  //}

  //if(mpViewer)
  //    mpViewer->Release();

  //// Edge-SLAM: measure
  //msRelocLastMapUpdateStart = std::chrono::high_resolution_clock::now();

  //mapUpToDate = true;
  //msRelocStatus = false;

  //// Edge-SLAM: debug
  //cout << "log,Tracking::mapCallback,local map update is done" << endl;





  /* ############################################################ */




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
