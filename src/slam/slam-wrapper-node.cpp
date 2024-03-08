#include "slam-wrapper-node.hpp"
#include <cstdlib>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
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
// - Maybe dont add completely new map but just add the keyframes and mappoints to existing map (if available) in a same way as in edgeslam. OR Check if there is something wrong with the current map since map points are not found from that one.
SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam, const std::string path, const std::string strResultFilename ) : Node("SlamWrapperNode") {
  
  const char* systemId = std::getenv("SLAM_SYSTEM_ID");
  
  if (systemId != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Initializing SLAM Wrapper Node. System ID=%s", systemId);
  } else {
    
    RCLCPP_FATAL(this->get_logger(), "Initializing SLAM Wrapper Node. System ID not found. Set up SLAM_SYSTEM_ID before continuing, exitting...");
    exit(1);
  }

  savePath=path;
  mstrResultFilename=strResultFilename;

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

  //mpTracker_->SetStepByStep(true);
   
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
    
    stopTimer();
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
  
  //RCLCPP_INFO(this->get_logger(),  "Saving data to the path=" + savePath + mstrResultFilename);
  //m_SLAM->SaveKeyFrameTrajectoryTUM(savePath + mstrResultFilename);
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
    
    std::lock_guard<std::mutex> lock(mMutexChecker);
    // Check if variables can be filled with new data
    std::cout << "STATS: # KF ptrs=" << mpOrbKeyFrames.size() << ", # MP ptrs=" << mpOrbMapPoints.size() << ", # CAM ptrs=" << mpOrbCameras.size()  << ", #Map ptrs=" << mpOrbMaps.size() << std::endl;
    //std::cout << "Current Map: ID=" << mpAtlas_->GetCurrentMap()->GetId() << ", #KFs=" << mpAtlas_->GetCurrentMap()->KeyFramesInMap() << ", #MPs=" << mpAtlas_->GetCurrentMap()->MapPointsInMap() << std::endl;
    //std::cout << std::endl;

    std::cout << "Number of unprocessed KeyFrames before=" << mpUnprocOrbKeyFrames.size() << ", ";
    for (auto it = mpUnprocOrbKeyFrames.begin(); it != mpUnprocOrbKeyFrames.end();)
    {
      
      if(!mpLocalMapper_->AcceptKeyFrames())
        break;
      //unique_lock<mutex> lock(mMutexNewKF);
      bool bKFUnprocessed = false;
      //ORB_SLAM3::KeyFrame* mopKf = it->second;
      //ORB_SLAM3::KeyFrame* mopKf = std::get<0>(mtORKf);
      orbslam3_interfaces::msg::KeyFrame::SharedPtr mrpKf = it->second;
      ORB_SLAM3::KeyFrame* mopKf = mpOrbKeyFrames[mrpKf->mn_id];

      //std::cout << "Before mp postloads" << std::endl; 
      for(const std::string mstrId : mrpKf->mv_backup_map_points_id)
      {
        if(mstrId == "" || mstrId.length() < 6) continue; 
        mpOrbMapPoints[mstrId]->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, &bKFUnprocessed);
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
     
      mopKf->SetORBVocabulary(m_SLAM->GetORBVocabulary());
      mopKf->SetKeyFrameDatabase(mpKfDb_);
      mopKf->UpdateMap(mpOrbMaps[mrpKf->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
      mopKf->ComputeBoW(); 
      
      mopKf->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed);
      if(!bKFUnprocessed){
        //mpAtlas_->AddKeyFrame(mopKf);

        //for(const auto& mpMP : mopKf->GetMapPoints()) {
        //  mpAtlas_->AddMapPoint(mpMP);
        //}

        mspKFsReadyForLM.insert(mopKf->mnId);
        it = mpUnprocOrbKeyFrames.erase(it);
        mspUnprocKFids.erase(mopKf->mnId);   
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
    std::lock_guard<std::mutex> lock(mMutexChecker);
    std::cout << "add keyframe to local mapping" << std::endl;
    while(!mspKFsReadyForLM.empty())
    {
      long unsigned int id=*mspKFsReadyForLM.begin(); 
      ORB_SLAM3::KeyFrame* pKF = mpOrbKeyFrames[id];
      
      
      if(!mpLocalMapper_->NeedNewKeyFrame(pKF))
      {
        mspKFsReadyForLM.erase(mspKFsReadyForLM.begin());
        break;
      }
      
        

      
      //pKF->UpdateConnections();
      //mpKfDb_->add(pKF);
      //mpAtlas_->AddKeyFrame(pKF);
      
      //for(ORB_SLAM3::MapPoint* pMP : pKF->GetMapPoints())
      //{
      //  mpAtlas_->AddMapPoint(pMP);
      //}
      
      std::cout << pKF->GetPose().translation()[0] << " " << pKF->GetPose().translation()[1] << " " << pKF->GetPose().translation()[2]<< std::endl;
      std::cout << pKF->GetPose().unit_quaternion().x() << " " << pKF->GetPose().unit_quaternion().y() << " " << pKF->GetPose().unit_quaternion().z() << " " << pKF->GetPose().unit_quaternion().w()<< std::endl;

      RCLCPP_INFO(this->get_logger(), "Adding a new KeyFrame with id: %d. Stats: #MPs=%d, Map ID=%d, #MP matches=%d, #mspChildrens=%d, #msploopedges=%d, #mspMergeEdges=%d, sizeOf=%d", pKF->mnId, pKF->GetMapPoints().size(), pKF->GetMap()->GetId(), pKF->GetMapPointMatches().size(), pKF->GetChilds().size(), pKF->GetLoopEdges().size(), pKF->GetMergeEdges().size());
      //std::cout << std::hash<ORB_SLAM3::KeyFrame*>()(pKF) << std::endl;
      //if(!mpLocalMapper_->NeedNewKeyFrame(pKF))
      //  std::cout << "No need for KF, dropping keyframe." << std::endl; 
      mpLocalMapper_->InsertKeyframeFromRos(pKF);
      mspKFsReadyForLM.erase(mspKFsReadyForLM.begin());

    }
  }

  if(mspKFsReadyForLM.empty() && mpUnprocOrbKeyFrames.empty()) 
    stopTimer();


}

void SlamWrapperNode::startTimer()
{
  if (action_check_timer_->is_canceled())
  {
    // Timer is not active
    // Perform actions or start the timer if needed
    action_check_timer_->reset(); 
  }
}

void SlamWrapperNode::stopTimer()
{
  action_check_timer_->cancel();
}

/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {

  std::lock_guard<std::mutex> lock(mMutexPublishKF);

  // Calculate the size of the object using sizeof
  RCLCPP_INFO(this->get_logger(), "Publishing a new KeyFrame with id: %d. Stats: #MPs=%d, Map ID=%d, #MP matches=%d, #mspChildrens=%d, #msploopedges=%d, #mspMergeEdges=%d, sizeOf=%d", pKf->mnId, pKf->GetMapPoints().size(), pKf->GetMap()->GetId(), pKf->GetMapPointMatches().size(), pKf->GetChilds().size(), pKf->GetLoopEdges().size(), pKf->GetMergeEdges().size());

  for(const auto& cam : mpAtlas_->GetAllCameras())
  {
    if(cam && cam != nullptr)
      mspCameras.insert(cam);
  }
  

  
  std::vector<ORB_SLAM3::KeyFrame*> mvpKeyFrames = pKf->GetMap()->GetAllKeyFrames();
  std::vector<ORB_SLAM3::MapPoint*> mvpMapPoints = pKf->GetMap()->GetAllMapPoints();
  std::set<ORB_SLAM3::KeyFrame*> mspKeyFrames(mvpKeyFrames.begin(), mvpKeyFrames.end());
  std::set<ORB_SLAM3::MapPoint*> mspMapPoints(mvpMapPoints.begin(), mvpMapPoints.end());
  

  pKf->PreSave(mspKeyFrames, mspMapPoints, mspCameras);
  
  mpOrbKeyFrames[pKf->mnId] = pKf;
  mpOrbMaps[pKf->GetMap()->GetId()] = pKf->GetMap();
  
  for(ORB_SLAM3::MapPoint* pMP : pKf->GetMapPoints())
  {
    pMP->PreSave(mspKeyFrames, mspMapPoints);
    mpOrbMapPoints[pMP->mstrHexId] = pMP;
  }
  //publishMap(pKf->GetMap());


  std::cout << pKf->GetPose().translation()[0] << " " << pKf->GetPose().translation()[1] << " " << pKf->GetPose().translation()[2]<< std::endl;
  std::cout << pKf->GetPose().unit_quaternion().x() << " " << pKf->GetPose().unit_quaternion().y() << " " << pKf->GetPose().unit_quaternion().z() << " " << pKf->GetPose().unit_quaternion().w()<< std::endl;
  orbslam3_interfaces::msg::KeyFrame msgKf = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKf);

  //ORB_SLAM3::KeyFrame* tempKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(std::make_shared<orbslam3_interfaces::msg::KeyFrame>(msgKf), static_cast<ORB_SLAM3::KeyFrame*>(NULL));
  //Utility::CompareTwoSerializedObjects(tempKF, pKf);
  
  msgKf.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  RCLCPP_INFO(this->get_logger(), "Publishing keyframe with id: %d. KF Map Id=%d == Current Map Id=%d", pKf->mnId, pKf->GetMap()->GetId(), mpAtlas_->GetCurrentMap()->GetId());
  keyframe_publisher_->publish(msgKf);
}

/*        Map        */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
  
  std::lock_guard<std::mutex> lock(mMutexPublishMap);
  RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
  RCLCPP_INFO(this->get_logger(), "Map Stats : #KFs=%d, #MPs=%d, #RefMPs=%d", pM->KeyFramesInMap(), pM->MapPointsInMap(), pM->GetReferenceMapPoints().size());

  
  for(ORB_SLAM3::KeyFrame* pKF : pM->GetAllKeyFrames())
  {
    if(mpOrbKeyFrames.find(pKF->mnId) == mpOrbKeyFrames.end())
      mpOrbKeyFrames[pKF->mnId] = pKF;
  }

  for(ORB_SLAM3::MapPoint* pMP : pM->GetAllMapPoints())
  {
    if(mpOrbMapPoints.find(pMP->mstrHexId) == mpOrbMapPoints.end())
      mpOrbMapPoints[pMP->mstrHexId] = pMP;
  }
  
  mpLocalMapper_->SetAcceptKeyFrames(false);
  mpLocalMapper_->AllowLocalMapping(false);
  
  pM->PreSave(mspCameras);
  
  std::cout << "before map conversion" << std::endl; 
  auto msgM = Converter::MapConverter::OrbMapToRosMap(pM);
  std::cout << "after map conversion" << std::endl; 
  //ORB_SLAM3::Map* newMap = Converter::MapConverter::RosMapToOrbMap(std::make_shared<orbslam3_interfaces::msg::Map>(msgM), static_cast<ORB_SLAM3::Map*>(NULL));
  //bool bKFUnprocessed = false;
  //newMap->PostLoad(mpKfDb_, m_SLAM->GetORBVocabulary(), mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed);
  msgM.system_id = std::getenv("SLAM_SYSTEM_ID");



  map_publisher_->publish(msgM);
  RCLCPP_INFO(this->get_logger(), "Map published with id: %d, delete MPs=%d, update MPs=%d, delete KFs=%d, update KFs=%d", msgM.mn_id, pM->GetErasedMPIds().size(), pM->GetUpdatedMPIds().size(), pM->GetErasedKFIds().size(), pM->GetUpdatedKFIds().size());
  
  pM->ClearErasedData();
  pM->ClearUpdatedKFIds();
  pM->ClearUpdatedMPIds();
  mpLocalMapper_->SetAcceptKeyFrames(true);
  mpLocalMapper_->AllowLocalMapping(true);
}


/*        MapPoint        */
void SlamWrapperNode::publishMapPoint(ORB_SLAM3::MapPoint* pMp) {
  //RCLCPP_INFO(this->get_logger(), "Publishing a new mappoint with id: %d", pMp->mnId);
  mpOrbMapPoints[pMp->mstrHexId] = pMp;
  orbslam3_interfaces::msg::MapPoint msgMp = Converter::MapPointConverter::ORBSLAM3MapPointToROS(pMp);
  msgMp.system_id = std::getenv("SLAM_SYSTEM_ID");
  
  map_point_publisher_->publish(msgMp);
  //delete pMp;
}

/*        Local Mapping - Active        */
void SlamWrapperNode::publishLMActivityChange(bool bActive) {
  RCLCPP_INFO(this->get_logger(), "Publishing Local Mapping Activity change=%d", bActive);
  orbslam3_interfaces::msg::Bool msg = orbslam3_interfaces::msg::Bool();
  
  msg.system_id = std::getenv("SLAM_SYSTEM_ID");
  msg.data = bActive;
  
  mbLocalMappingIsIdle = !bActive;

  lm_active_publisher_->publish(msg);
}

/*        Local Mapping - Reset Requested        */
void SlamWrapperNode::publishLMResetRequested() {
  RCLCPP_INFO(this->get_logger(), "Publishing Local Mapping Reset Requested=true");
  std_msgs::msg::Bool msg = std_msgs::msg::Bool();
  msg.data = true;
  
  lm_reset_requested_publisher_->publish(msg);
}

void SlamWrapperNode::publishResetActiveMap(unsigned long int mnMapId) {
  RCLCPP_INFO(this->get_logger(), "Publishing Reset Active map, id=%d", mnMapId);
  orbslam3_interfaces::msg::Int64 msg = orbslam3_interfaces::msg::Int64();
  msg.system_id = std::getenv("SLAM_SYSTEM_ID");
  msg.data = static_cast<long int>(mnMapId);
  
  sys_reset_active_map_publisher_->publish(msg);
}


/* OBJECT SUBSCRIPTION CALLBACKS */

/*        KeyFrame        */
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf) {
  
  if(rKf->system_id == std::getenv("SLAM_SYSTEM_ID")) 
      return;
  
  std::lock_guard<std::mutex> lock(mMutexNewKF);
  
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

  ORB_SLAM3::KeyFrame* oKf = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
  if(mpOrbKeyFrames.find(rKf->mn_id) == mpOrbKeyFrames.end())
  {
    
    RCLCPP_INFO(this->get_logger(), "KF not found, creating a new object.");
    oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf, static_cast<ORB_SLAM3::KeyFrame*>(NULL));
  } else
  {
    RCLCPP_INFO(this->get_logger(), "KF found, updating existing object.");
    oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKf, mpOrbKeyFrames[rKf->mn_id]);
  }
  
  mpOrbKeyFrames[oKf->mnId] = oKf;

  for(size_t i = 0; i < rKf->mvp_map_points.size(); i++)
  {
    orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rKf->mvp_map_points[i]);
    
    ORB_SLAM3::MapPoint* oMP;
    if(mpOrbMapPoints.find(mp->m_str_hex_id) != mpOrbMapPoints.end())
      oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, mpOrbMapPoints[mp->m_str_hex_id]);
    else
      oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, static_cast<ORB_SLAM3::MapPoint*>(NULL));
    

    mpOrbMapPoints[oMP->mstrHexId] = oMP;
    oMP->UpdateMap(mpOrbMaps[mp->mp_map_id]);
  }
  
  //m_SLAM->Shutdown();
  //std::cout << "after converting" << std::endl; 
  
  oKf->SetORBVocabulary(m_SLAM->GetORBVocabulary());
  oKf->SetKeyFrameDatabase(mpKfDb_);
  oKf->UpdateMap(mpOrbMaps[rKf->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
  oKf->ComputeBoW(); 
  

  bool bKFUnprocessed = false;

  //std::cout << "Before mp postloads #MPs=" << rKf->mv_backup_map_points_id.size() << ", [0]=" << rKf->mv_backup_map_points_id[0]<< std::endl; 
  for(const auto& mstrId : rKf->mv_backup_map_points_id)
  {
    if(mstrId == "" || mstrId.length() < 6) continue;
    mpOrbMapPoints[mstrId]->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, &bKFUnprocessed, mspUnprocKFids);
    if(bKFUnprocessed) {
      std::cout << "Map Point if unprocessed, continuing..." << std::endl;
      
      startTimer();
      mpUnprocOrbKeyFrames[oKf->mnId] = rKf;
      mspUnprocKFids.insert(oKf->mnId);   
      return;
    }
  }

  //std::cout << "after mp postloads" << std::endl;
  
  oKf->PostLoad(mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed, mspUnprocKFids );
  std::cout << oKf->GetMap()->GetAllMapPoints().size() << std::endl;
  
  if(!bKFUnprocessed){
    //mpAtlas_->AddKeyFrame(oKf);

    //for(const auto& mpMP : oKf->GetMapPoints()) {
    //  if(!mpAtlas_->CheckIfMapPointInMap(mpMP) && !mpMP->isBad()) mpAtlas_->AddMapPoint(mpMP);
    //}
    
    if(mpLocalMapper_->NeedNewKeyFrame(oKf))
    {
      //mpKfDb_->add(oKf);
      mpLocalMapper_->InsertKeyframeFromRos(oKf);
      //return;
    } else {
      oKf->SetBadFlag();
    }

    //if (action_check_timer_->is_canceled())
    //{
    // Timer is not active
    // Perform actions or start the timer if needed
    //action_check_timer_->reset();
    //}
    //mspKFsReadyForLM.insert(oKf->mnId);
  } else {
    std::cout << "KeyFrame is unprocessed (All MPs are good), continuing..." << std::endl;

    startTimer();
    mpUnprocOrbKeyFrames[oKf->mnId] = rKf;
    mspUnprocKFids.insert(oKf->mnId);   
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Got a new keyframe. ID=%d. Stats: #MPs=%d, Map ID=%d == Current Map ID=%d", oKf->mnId, oKf->GetNumberMPs(), oKf->GetMap()->GetId(), mpAtlas_->GetCurrentMap()->GetId());
}
/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM) {
  if(rM->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  std::chrono::system_clock::time_point mcMapReceiveTime = std::chrono::system_clock::now();
  if(mpTracker_->mcLastResetTimeStamp < mcMapReceiveTime)
  {
    if(rM->msp_keyframes.size() > mpAtlas_->GetCurrentMap()->KeyFramesInMap())
    {
      RCLCPP_INFO(this->get_logger(), "Map was resetted before receiving update, do not accept it. receive time=%d, last reset time=%d", mcMapReceiveTime, mpTracker_->mcLastResetTimeStamp);
      return;
    }
  }

  if(mpOrbMaps.find(rM->mn_id) != mpOrbMaps.end())
  {
    RCLCPP_INFO(this->get_logger(), "Got an update for a map, id=%d. Stats: #KFs=%d, #MPs=%d", rM->mn_id, rM->msp_keyframes.size(), rM->msp_map_points.size());
  } else { 
    RCLCPP_INFO(this->get_logger(), "Got a new map, id=%d. Stats: #KFs=%d, #MPs=%d", rM->mn_id, rM->msp_keyframes.size(), rM->msp_map_points.size());
  }

  //mpOrbMaps.clear();
  //mpOrbKeyFrames.clear();
  //mpOrbMapPoints.clear();
  //
  {
    unique_lock<mutex> lock(mMutexUpdateMap);
    for(ORB_SLAM3::Map* pM : mpAtlas_->GetAllMaps())
    {
    //  mpOrbMaps[pM->GetId()] = pM;

      for(ORB_SLAM3::KeyFrame* pKF : pM->GetAllKeyFrames())
      {
        if(pKF)
        {
          if(mpOrbKeyFrames.find(pKF->mnId) == mpOrbKeyFrames.end())
            mpOrbKeyFrames[pKF->mnId] = pKF;
        }
      }

      for(ORB_SLAM3::MapPoint* pMP : pM->GetAllMapPoints())
      {
        if(pMP)
        {
          if(mpOrbMapPoints.find(pMP->mstrHexId) == mpOrbMapPoints.end())
            mpOrbMapPoints[pMP->mstrHexId] = pMP;
        }
      }
    }
  }


  RCLCPP_INFO(this->get_logger(), "UPDATED STATS. #Maps=%d, #KFs=%d, #MPs=%d", mpOrbMaps.size(), mpOrbKeyFrames.size(), mpOrbMapPoints.size());
  
  //if(mpTracker_->IsMapUpToDate())
  //{
  //  std::cout << "Map is up to date, skipping update.." << std::endl;
  //  return;
  //}
  
  
  RCLCPP_INFO(this->get_logger(), "Inform tracking that Map update is being executed. Number of KFs in old map=%d, and MPs=%d", mpAtlas_->GetCurrentMap()->GetAllKeyFrames().size(), mpAtlas_->GetCurrentMap()->GetAllMapPoints().size() );
  
  //bool bUnprocessed = false;
  ORB_SLAM3::Map* opM = static_cast<ORB_SLAM3::Map*>(NULL);
  if(mpOrbMaps.find(rM->mn_id) != mpOrbMaps.end())
  {
    unique_lock<mutex> lock(mMutexUpdateMap);
    std::cout << "Updating map" << std::endl;
    opM = mpOrbMaps[rM->mn_id];
    opM = Converter::MapConverter::RosMapToOrbMap(rM, opM);
    
    //std::cout << "Reseting Database..." << std::endl;;
    //mpKfDb_->clearMap(mpPrevMap);
    //mpAtlas_->clearMap();
    //mpAtlas_->AddMap(opM);
  }
  
  RCLCPP_INFO(this->get_logger(), "After update: KFs=%d, and MPs=%d", mpAtlas_->GetCurrentMap()->GetAllKeyFrames().size(), mpAtlas_->GetCurrentMap()->GetAllMapPoints().size() );

  std::cout << "After creating map" << std::endl;
  std::vector<std::string> mvpMPsDone;
  std::cout << "Before loop" << std::endl;
  
  //usleep(3000);
  std::cout << "#KFs=" << rM->mvp_backup_keyframes_ids.size() << ", #updates=" << rM->msp_keyframes.size() << ", erased=" << rM->mvp_erased_keyframe_ids.size() << std::endl;
  std::cout << "#MPs=" << rM->mvp_backup_map_points_ids.size() << ", #updates=" << rM->msp_map_points.size() << ", erased=" << rM->mvp_erased_mappoint_ids.size() << std::endl;
  

  size_t kfFound=0;
  size_t kfNotFound=0;
  size_t mpFound=0;
  size_t mpNotFound=0;
  


  mpTracker_->LocalMapIsUpdating(true); 
  for(size_t j = 0; j < rM->msp_keyframes.size(); j++)
  {
    unique_lock<mutex> lock(mMutexUpdateMap);
    
    orbslam3_interfaces::msg::KeyFrame::SharedPtr rKF = std::make_shared<orbslam3_interfaces::msg::KeyFrame>(rM->msp_keyframes[j]);

    //auto it = std::find(rM->mvp_erased_keyframe_ids.begin(), rM->mvp_erased_keyframe_ids.end(), rKF->mn_id);

    ORB_SLAM3::KeyFrame* oKf = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
    if(mpOrbKeyFrames.find(rKF->mn_id) == mpOrbKeyFrames.end())
    {
      //if (it != rM->mvp_erased_keyframe_ids.end()) {
      //  std::cout << "Do not add KF=" << rKF->mn_id << " since it was removed in LM." << std::endl;
      //  continue;
      //}
      
      //std::cout << " - KF=" << rKF->mn_id << " not found, processing..."<< std::endl;
      oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKF, static_cast<ORB_SLAM3::KeyFrame*>(NULL));    
      oKf->SetORBVocabulary(m_SLAM->GetORBVocabulary());
      oKf->SetKeyFrameDatabase(mpKfDb_);
      oKf->ComputeBoW();
      mpOrbKeyFrames[oKf->mnId] = oKf;
      kfNotFound+=1;
    } else {
      //if (it != rM->mvp_erased_keyframe_ids.end()) {
      //  std::cout << "remove KF=" << rKF->mn_id << " since it was removed in LM." << std::endl;
      //  mpOrbMaps[rKF->mp_map_id]->EraseKeyFrame(mpOrbKeyFrames[rKF->mn_id]);
      //  mpKfDb_->erase(mpOrbKeyFrames[rKF->mn_id]);
      //  mpOrbKeyFrames.erase(rKF->mn_id);
      //}
      
      //std::cout << " - KF=" << rKF->mn_id << " found, processing..."<< std::endl;
      oKf = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(rKF, mpOrbKeyFrames[rKF->mn_id]);
      kfFound+=1;
    }

    //mpOrbMaps[rM->mn_id]->AddKeyFrame(oKf);
    
    for(size_t i = 0; i < rKF->mvp_map_points.size(); i++)
    {
      orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rKF->mvp_map_points[i]);
      //std::cout << "Processing MP=" << mp->mn_id << std::endl; 
      ORB_SLAM3::MapPoint* oMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
      if(mpOrbMapPoints.find(mp->m_str_hex_id) == mpOrbMapPoints.end())
      {
        //std::cout << " - MP=" << mp->m_str_hex_id << " not found, processing..."<< std::endl;
        oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, static_cast<ORB_SLAM3::MapPoint*>(NULL));
        mpOrbMapPoints[oMP->mstrHexId] = oMP;
        mpNotFound+=1;
      } else {
        //std::cout << " - MP=" << mp->m_str_hex_id << " found, processing..."<< std::endl;
        oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, mpOrbMapPoints[mp->m_str_hex_id]);
        mpNotFound+=1;
      }
      
      //mpOrbMaps[rKF->mp_map_id]->AddMapPoint(oMP);
      mvpMPsDone.push_back(oMP->mstrHexId);
    }
    
    //std::cout << "Done with MPs" << std::endl; 
 

  }
  //std::cout << "Done with KFs" << std::endl; 

  
  std::cout << "Checking if map contains map points which does not belong to any keyframes.."<< std::endl;
  for(size_t j = 0; j < rM->msp_map_points.size(); j++)
  {
    unique_lock<mutex> lock(mMutexUpdateMap);
    std::string targetId = rM->msp_map_points[j].m_str_hex_id;
    
    auto it = std::find(mvpMPsDone.begin(), mvpMPsDone.end(), targetId);

    //std::cout << *it << ", " << targetId << std::endl;

    if(it == mvpMPsDone.end())
    {
      //std::cout << "MapPoint is in map but not in KF." << std::endl;
      orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(rM->msp_map_points[j]);
      
      ORB_SLAM3::MapPoint* oMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
      if(mpOrbMapPoints.find(mp->m_str_hex_id) == mpOrbMapPoints.end())
      {
        //std::cout << " - MP=" << mp->m_str_hex_id << " not found, processing..."<< std::endl;
        oMP = Converter::MapPointConverter::RosMapPointToOrb(mp,static_cast<ORB_SLAM3::MapPoint*>(NULL));
        mpOrbMapPoints[oMP->mstrHexId] = oMP;
        mpNotFound+=1;
      } else {
        //std::cout << " - MP=" << mp->m_str_hex_id << " found, processing..."<< std::endl;
        oMP = Converter::MapPointConverter::RosMapPointToOrb(mp, mpOrbMapPoints[mp->m_str_hex_id]);
        mpFound+=1;
      }

      //mpOrbMaps[rM->mn_id]->AddMapPoint(oMP);
      //oMP->UpdateMap(opM);
      mvpMPsDone.push_back(oMP->mstrHexId);
    } //else {
    //  std::cout << "MapPoint is already processed." << std::endl;
    //}

  }
  
  //mpTracker_->LocalMapIsUpdating(true);
  std::cout << "#KFs found=" << kfFound << ", # not found=" << kfNotFound << ", #MPs found=" << mpFound << ", # not found=" << mpNotFound << std::endl;
  //if(mpOrbMaps.find(opM->GetId()) != mpOrbMaps.end())
  //{
  //  ORB_SLAM3::Map* mpPrevMap = mpOrbMaps[opM->GetId()];
    
  //  std::cout << "Reseting Database..." << std::endl;;
  //  mpKfDb_->clearMap(mpPrevMap);
  //  mpAtlas_->clearMap();
  //}
  std::cout << "All data processed, next map postload -> fix the pointer connections." << std::endl; 
  bool bKFUnprocessed = false;
  
  {
    unique_lock<mutex> lock(mMutexUpdateMap);
    std::cout << "PostLoad for Map=" << opM->GetId() << std::endl;
    opM->PostLoad(mpKfDb_, m_SLAM->GetORBVocabulary(), mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed, mspUnprocKFids);

    for(const auto& kfId : rM->mvp_erased_keyframe_ids)
    {
      if(mpOrbKeyFrames.find(kfId) != mpOrbKeyFrames.end())
      {
        //std::cout << "  - KF=" << kfId << " is erased, SetBadFlag()" << std::endl;
        mpOrbKeyFrames[kfId]->SetBadFlag();
        mpOrbKeyFrames.erase(kfId);
      }
    }
    
    std::cout << "MPs before=" << opM->GetAllMapPoints().size() << std::endl;
    for(const auto& mpId : rM->mvp_erased_mappoint_ids)
    {
      if(mpOrbMapPoints.find(mpId) != mpOrbMapPoints.end())
      {
        if(mpOrbMapPoints[mpId])
        {
          //std::cout << "  - MP=" << mpId << " is erased, SetBadFlag()" << std::endl;
          mpOrbMapPoints[mpId]->SetBadFlag();
          mpOrbMapPoints.erase(mpId);
          
        }
      }
    }
    std::cout << "MPs after=" << opM->GetAllMapPoints().size() << std::endl;
  }
  mpTracker_->LocalMapIsUpdating(false); 
  
  //mpTracker_->LocalMapIsUpdating(false); 
  
  //{
  //  unique_lock<mutex> lock(mMutexUpdateMap);
  //  mpOrbMaps[rM->mn_id]->PostLoad(mpKfDb_, m_SLAM->GetORBVocabulary(), mpOrbKeyFrames, mpOrbMapPoints, mpOrbCameras, &bKFUnprocessed);
  //}
  if(bKFUnprocessed) std::cout << "Map with ID=" << opM->GetId() << " is unprocessed!!" << std::endl;
  
  //if(mpOrbMaps.find(opM->GetId()) != mpOrbMaps.end())
  //{
  //  std::cout << "Map found, replacing it .. " << std::endl;
  //  mpAtlas_->ReplaceMap(opM);
  //} else {
  //  std::cout << "New map, adding it to Atlas..." << std::endl;
  //  mpAtlas_->AddMap(opM);
  //}

  //mpOrbMaps[opM->GetId()] = opM;
  std::cout << "Done with GrabMap."<< std::endl;
  
  //mpTracker_->LocalMapIsUpdating(true); 
  mpTracker_->UpdateFromLocalMapping(mpOrbMaps[rM->mn_id], mpOrbKeyFrames, mpOrbMapPoints);

  
}

/*        MapPoint        */
void SlamWrapperNode::GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr rpMp) {
  if(rpMp->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  //RCLCPP_INFO(this->get_logger(), "Got a new mappoint, id: %d", rpMp->mn_id);
  bool bUnprocessed = false;
  
  ORB_SLAM3::MapPoint* oMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
  if(mpOrbMapPoints.find(rpMp->m_str_hex_id) == mpOrbMapPoints.end())
    oMP = Converter::MapPointConverter::RosMapPointToOrb(rpMp, static_cast<ORB_SLAM3::MapPoint*>(NULL));
  else
    oMP = Converter::MapPointConverter::RosMapPointToOrb(rpMp, mpOrbMapPoints[rpMp->m_str_hex_id]);

  if(bUnprocessed) {
    RCLCPP_INFO(this->get_logger(), "*##*#*#*#*#*#*#*###**#*#*#* MapPoint with ID=%s is unprocessed. *#*####*#*#*#**#*#*#*#*#", oMP->mstrHexId);
    mpUnprocOrbMapPoints[oMP->mstrHexId] = std::make_tuple(oMP, rpMp);
  } 

  mpOrbMapPoints[oMP->mstrHexId] = oMP; //.insert(std::make_pair(opMp->mnId, opMp));

  //RCLCPP_INFO(this->get_logger(), "MapPoint with ID '%d is converted.", opMp->mnId); 
}


/*        System Reset Active Map*/
void SlamWrapperNode::GrabResetActiveMap(const orbslam3_interfaces::msg::Int64::SharedPtr msg) {
  if(msg->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
  RCLCPP_INFO(this->get_logger(), "Reset requested for Active map id=%d", msg->data);

  mpTracker_->ResetActiveMap(false, msg->data);
  
  mpOrbMaps.clear();
  mpOrbKeyFrames.clear();
  mpOrbMapPoints.clear();
  
}


/*        LocalMapping reset        */
void SlamWrapperNode::GrabLMResetRequested(const std_msgs::msg::Bool::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Reset requested for Local mapping=%d", msg->data);

  mpLocalMapper_->RequestReset();
  

  stopTimer();
  mspKFsReadyForLM.clear();
  mpUnprocOrbKeyFrames.clear();
  mpOrbMaps.clear();
  mpOrbKeyFrames.clear();
  mpOrbMapPoints.clear();

  mpKfDb_->clear();
  mpAtlas_->clearAtlas();
  mpAtlas_->CreateNewMap();

}

/*        LocalMapping - Active        */
void SlamWrapperNode::GrabLMActive(const orbslam3_interfaces::msg::Bool::SharedPtr msg) {
  if(msg->system_id == std::getenv("SLAM_SYSTEM_ID")) 
    return;
  
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


void SlamWrapperNode::publishStep() 
{
  std_msgs::msg::Bool bMsg;
  bMsg.data = true;
  
  end_publisher_->publish(bMsg);
  RCLCPP_INFO(this->get_logger(), "Publishing to /step");
}


void SlamWrapperNode::stepCallback(std_msgs::msg::Bool::SharedPtr bMsg) {
  RCLCPP_INFO(this->get_logger(), "Received msg from /step");
  mpTracker_->mbStep = true;
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
  lm_active_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Bool>(
      "/LocalMapping/Active", 
      10);

  /* LocalMapping Active*/
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /LocalMapping/Reset");
  lm_reset_requested_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/LocalMapping/Reset", 
      10);


  /* Reset Active Map (System) */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map/Active/Reset");
  sys_reset_active_map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Int64>(
      "/Map/Reset/Active", 
      10);

  /* Destroy */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /destroy");
  end_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/destroy", 
      10);

  /* Step */
  RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /step");
  step_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/step", 
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
  m_lm_active_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Bool>(
      "LocalMapping/Active",
      10,
      std::bind(&SlamWrapperNode::GrabLMActive, this, std::placeholders::_1));

  /* LocalMapping reset requested */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Reset");
  m_lm_reset_requested_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "LocalMapping/Reset",
      10,
      std::bind(&SlamWrapperNode::GrabLMResetRequested, this, std::placeholders::_1));

  /* LocalMapping reset requested */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map/Reset/Active");
  m_sys_reset_active_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Int64>(
      "Map/Reset/Active",
      10,
      std::bind(&SlamWrapperNode::GrabResetActiveMap, this, std::placeholders::_1));
  

  /* Destroy node */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /destroy");
  end_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "/destroy", 
      10, 
      std::bind(&SlamWrapperNode::endCallback, this, std::placeholders::_1));
  
  /* Step */
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /step");
  step_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "/step", 
      10, 
      std::bind(&SlamWrapperNode::stepCallback, this, std::placeholders::_1));
}
