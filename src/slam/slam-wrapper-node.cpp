#include "slam-wrapper-node.hpp"
#include <cstdlib>
#include <stdlib.h>
#include <rmw/qos_profiles.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam, const std::string path, const std::string strResultFilename ) : Node("SlamWrapperNode") {
    
    char* systemId = std::getenv("SLAM_SYSTEM_ID");
    
    RCLCPP_INFO(this->get_logger(), "Initializing SLAM Wrapper Node. System ID=%s", systemId);
    
    if(string(systemId) == "TR") {
        RCLCPP_INFO(this->get_logger(), " °Main responsibility: Tracking");
        mnTaskModule = 1;
    } else if (string(systemId) == "LM") {
        RCLCPP_INFO(this->get_logger(), " °Main responsibility: Local Mapping");
        mnTaskModule = 2;
    } else if (string(systemId) == "LC") {
        RCLCPP_INFO(this->get_logger(), " °Main responsibility: Loop Closing");
        mnTaskModule = 3;
    } else {
        RCLCPP_INFO(this->get_logger(), " °Main responsibility: Idle");
        mnTaskModule = 0;
    }

    savePath=path;
    mstrResultFilename=strResultFilename;

    m_SLAM = pSLAM;
    mpTracker_ = m_SLAM->GetTrackerPtr();
    mpLocalMapper_ = m_SLAM->GetMapperPtr();
    mpLoopCloser_ = m_SLAM->GetLoopClosingPtr();
    mpAtlas_ = m_SLAM->GetAtlas();
    mpKeyFrameDB = m_SLAM->GetKeyFrameDatabase();
   
    UpdateReset(false);
    IncreaseKFCount(0);

    for(const auto& mpCam : mpAtlas_->GetAllCameras())
    {
        mORBCameras[mpCam->GetId()] = mpCam;
    }

    
    mbLocalMappingIsIdle = true;
    mpLocalMapper_->AllowLocalMapping(true);

    //mpTracker_->SetStepByStep(true);

    CreatePublishers();

    /* Init subscribers */
    if (subscribe_to_slam) { 
        CreateSubscribers();
        action_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SlamWrapperNode::checkForNewActions, this)
        );
        
        stopTimer();
    }

    //// Add the current map to the maps
    std::vector<ORB_SLAM3::Map*> mpAtlasMaps = mpAtlas_->GetAllMaps();
    for(ORB_SLAM3::Map* pM : mpAtlasMaps) {
        AddMap(pM);  
    }
    
    for(const auto& cam : mpAtlas_->GetAllCameras())
    {
        if(cam && cam != nullptr)
            mspCameras.insert(cam);
    }
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
  
    if(!mpUnprocOrbKeyFrames.empty()) {
      
        std::lock_guard<std::mutex> lock(mMutexUpdateMap);
        // Check if variables can be filled with new data
        std::cout << "STATS: # KF ptrs=" << mORBKeyFrames.size() << ", # MP ptrs=" << mORBMapPoints.size() << ", # CAM ptrs=" << mORBCameras.size()  << ", #Map ptrs=" << mORBMaps.size() << std::endl;
        for (auto it = mpUnprocOrbKeyFrames.begin(); it != mpUnprocOrbKeyFrames.end();)
        {
        
            if(MapActionActive())
                break;
            
            SetKeyFrameAction(true);
            //unique_lock<mutex> lock(mMutexNewKF);
            bool bKFUnprocessed = false;
            
            orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF = it->second;
            ORB_SLAM3::KeyFrame* pKF = mORBKeyFrames[mpRosKF->mn_id];

            //std::cout << "Before mp postloads" << std::endl; 
            for(const std::string mstrId : mpRosKF->mv_backup_map_points_id)
            {
                if(mstrId == "" || mstrId.length() < 6) continue; 
                mORBMapPoints[mstrId]->PostLoad(mORBKeyFrames, mORBMapPoints, &bKFUnprocessed);
                if(bKFUnprocessed) {
                    std::cout << "Map Point is unprocessed, continuing..." << std::endl;
                    //mpUnprocOrbKeyFrames[oKf->mnId] = std::make_tuple(oKf, rKf);
                    break;
                }
            }

            if(bKFUnprocessed) 
            {
                SetKeyFrameAction(false);
                ++it;
                continue;
            }
           
            pKF->SetORBVocabulary(m_SLAM->GetORBVocabulary());
            pKF->SetKeyFrameDatabase(mpKeyFrameDB);
            pKF->UpdateMap(mORBMaps[mpRosKF->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
            pKF->ComputeBoW(); 
            
            pKF->PostLoad(mORBKeyFrames, mORBMapPoints, mORBCameras, &bKFUnprocessed);
            if(!bKFUnprocessed){
                mspKFsReadyForLM.insert(pKF->mnId);
                it = mpUnprocOrbKeyFrames.erase(it);
                mspUnprocKFids.erase(pKF->mnId);   
            } else {
                std::cout << "KeyFrame is unprocessed (All MPs are good), continuing..." << std::endl;
                ++it;
            }
            SetKeyFrameAction(false);
        }
    }

    //TODO: Add somekind of logic so that there can be unprocessed keyframes but not any relevant ones.
    if(!mspKFsReadyForLM.empty() && mpLocalMapper_->AcceptKeyFrames() && mpUnprocOrbKeyFrames.empty())
    {
        std::lock_guard<std::mutex> lock(mMutexUpdateMap);
        //std::cout << "add keyframe to local mapping" << std::endl;
        while(!mspKFsReadyForLM.empty())
        {

            if(MapActionActive())
                break;

            long unsigned int id=*mspKFsReadyForLM.begin(); 
            ORB_SLAM3::KeyFrame* pKF = mORBKeyFrames[id];
            
            //if(!mpLocalMapper_->NeedNewKeyFrame(pKF))
            //{
            //    mspKFsReadyForLM.erase(mspKFsReadyForLM.begin());
            //    break;
            //}
            
            RCLCPP_INFO(this->get_logger(), "Adding a new KeyFrame with id: %d. Stats: #MPs=%d, Map ID=%d, #MP matches=%d, #mspChildrens=%d, #msploopedges=%d, #mspMergeEdges=%d", pKF->mnId, pKF->GetMapPoints().size(), pKF->GetMap()->GetId(), pKF->GetMapPointMatches().size(), pKF->GetChilds().size(), pKF->GetLoopEdges().size(), pKF->GetMergeEdges().size());
            ForwardKeyFrameToTarget(pKF, 2);
            //mpLocalMapper_->InsertKeyframeFromRos(pKF);
            mspKFsReadyForLM.erase(mspKFsReadyForLM.begin());

        }
    }

    if(mspKFsReadyForLM.empty() && mpUnprocOrbKeyFrames.empty()) 
        stopTimer();


}

void SlamWrapperNode::UpdateReset(bool mbReset)
{
    unique_lock<std::mutex> lock(mMutexReset);
    mbResetActive = mbReset;
}

bool SlamWrapperNode::GetResetStatus()
{
    unique_lock<std::mutex> lock(mMutexReset);
    return mbResetActive;

}

void SlamWrapperNode::AddKeyFrame(ORB_SLAM3::KeyFrame* pKF)
{
    unique_lock<std::mutex> lock(mMutexKF);
    if(!pKF || pKF->isBad())
        return;

    mORBKeyFrames[pKF->mnId] = pKF; 
}

void SlamWrapperNode::EraseKeyFrame(ORB_SLAM3::KeyFrame* pKF)
{
    unique_lock<std::mutex> lock(mMutexKF); 
    if(!pKF)
        return;
    
    mORBKeyFrames.erase(pKF->mnId); 
}

void SlamWrapperNode::AddMapPoint(ORB_SLAM3::MapPoint* pMP)
{
    unique_lock<std::mutex> lock(mMutexMapPoint); 
    if(!pMP)
        return;
    if(pMP->mnId > mnMaxMPId)
    {
        mnMaxMPId=pMP->mnId;
    }

    if(pMP->isBad())
        return;

    mORBMapPoints[pMP->mstrHexId] = pMP; 
}
void SlamWrapperNode::EraseMapPoint(ORB_SLAM3::MapPoint* pMP)
{
    unique_lock<std::mutex> lock(mMutexMapPoint); 
    if(!pMP)
        return;
    
    mORBMapPoints.erase(pMP->mstrHexId); 
    pMP->SetBadFlag();
    
    //pMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
}

void SlamWrapperNode::AddMap(ORB_SLAM3::Map* pM) 
{
    unique_lock<std::mutex> lock(mMutexMap); 
    mORBMaps[pM->GetId()] = pM; 
}

void SlamWrapperNode::EraseMap(ORB_SLAM3::Map* pM) 
{
    unique_lock<std::mutex> lock(mMutexMap); 
    mORBMaps.erase(pM->GetId()); 
}

unsigned long int SlamWrapperNode::GetMaxMapPointID()
{
    unique_lock<std::mutex> lock(mMutexMapPoint);
    return mnMaxMPId;
}
unsigned long int SlamWrapperNode::GetMaxKeyFrameID()
{
    unique_lock<std::mutex> lock(mMutexKF);
    return mnMaxKFId;
}

void SlamWrapperNode::CalcMaxMapPointID()
{
    unique_lock<std::mutex> lock(mMutexMapPoint);
    if (!mORBMapPoints.empty()) {
        unsigned long int largestId = 0; // Start with the first element as the largest
        for (auto it = mORBMapPoints.begin(); it != mORBMapPoints.end(); ++it) {
            if(!it->second)
                continue;
            
            if (it->second->mnId > largestId) {
                largestId = it->second->mnId; // Found a larger ID
            }
        }
        if(largestId > mnMaxMPId)
          mnMaxMPId = largestId;
        std::cout << "The object with the largest id has id: " << largestId << std::endl;
    } else {
        mnMaxMPId = 0;
        std::cout << "The map is empty." << std::endl;
    }
}
  
void SlamWrapperNode::CalcMaxKeyFrameID()
{
    unique_lock<std::mutex> lock(mMutexKF);
    if (!mORBKeyFrames.empty()) {
        unsigned long int largestId = 0; // Start with the first element as the largest
        for (auto it = mORBKeyFrames.begin(); it != mORBKeyFrames.end(); ++it) {
            if(it->second)
            {
                if (it->second->mnId > largestId) {
                    largestId = it->second->mnId; // Found a larger ID
                }
            }
        }
        mnMaxKFId = largestId;
        std::cout << "The object with the largest id has id: " << largestId << std::endl;
    } else {
        mnMaxKFId = 0;
        std::cout << "The map is empty." << std::endl;
    }
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

void SlamWrapperNode::SetKeyFrameAction(bool mbAction)
{
    unique_lock<std::mutex> lock(mMutexKF);
    mbKeyFrameAction = mbAction;
}


void SlamWrapperNode::SetMapAction(bool mbAction)
{
    unique_lock<std::mutex> lock(mMutexMap);
    mbMapAction = mbAction;
}

void SlamWrapperNode::SetAtlasAction(bool mbAction)
{
    unique_lock<std::mutex> lock(mMutexAtlas);
    mbAtlasAction = mbAction;
}


bool SlamWrapperNode::KeyFrameActionActive()
{
    unique_lock<std::mutex> lock(mMutexKF);
    return mbKeyFrameAction;
}

bool SlamWrapperNode::MapActionActive()
{
    unique_lock<std::mutex> lock(mMutexMap);
    return mbMapAction;
}


bool SlamWrapperNode::AtlasActionActive()
{
    unique_lock<std::mutex> lock(mMutexAtlas);
    return mbAtlasAction;
}


void SlamWrapperNode::IncreaseKFCount(int inc)
{
    unique_lock<std::mutex> lock(mMutexKF);
    if(inc==0)
        mnKeyFramesAfterLC=0;
    else
        mnKeyFramesAfterLC+=inc;
}
long int SlamWrapperNode::GetKFCount()
{
    unique_lock<std::mutex> lock(mMutexKF);
    return mnKeyFramesAfterLC;
}







void SlamWrapperNode::ForwardKeyFrameToTarget(ORB_SLAM3::KeyFrame* pKF, const unsigned int mnTargetModule)
{
    // Just update state and do not insert to any module
    if(mnTaskModule == 0) {
        for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
        {
            if(pMPi)
                mpAtlas_->AddMapPoint(pMPi);
        }
        
        if(pKF)
        {
            IncreaseKFCount(1);  
            mpKeyFrameDB->add(pKF);
            mpAtlas_->AddKeyFrame(pKF);
        }
    } else if (mnTaskModule == 1) {
        // in this case the system is mainly performing tracking
        // For now, tracking does not grab any KFs, only Maps 
    } else if (mnTaskModule == 2) {
        // in this case the system mainly performing local mapping
        if(mnTargetModule == 2) {
            // if the KF is meant to be inserted to LM
            // Insert to Local Mapping
            if(mpLocalMapper_->NeedNewKeyFrame(pKF))
            {
                //mpKeyFrameDB->add(pKF);
                IncreaseKFCount(1);  
                mpLocalMapper_->InsertKeyframeFromRos(pKF);
            } else {
                mspKFsReadyForLM.insert(pKF->mnId);
                startTimer();
                //pKF->SetBadFlag();
            }  
        }    
    } else if (mnTaskModule == 3) {
        // in this case the system mainly performing loop closing
        // not sure what to do here yet. 
        if(mnTargetModule == 3) {
            if(pKF)
            {
                for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
                {
                    if(pMPi)
                        mpAtlas_->AddMapPoint(pMPi);
                }
                IncreaseKFCount(1);  
                mpAtlas_->AddKeyFrame(pKF);
                pKF->UpdateConnections();
                mpLoopCloser_->InsertKeyFrame(pKF);
            }
        } else {
            for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
            {
                if(pMPi)
                    mpAtlas_->AddMapPoint(pMPi);
            }
            
            if(pKF)
            {
                IncreaseKFCount(1);  
                mpKeyFrameDB->add(pKF);
                mpAtlas_->AddKeyFrame(pKF);
            }
        }
    }
}

/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule) {
    
    //std::lock_guard<std::mutex> lock(mMutexUpdateMap);

    // Calculate the size of the object using sizeof
    RCLCPP_INFO(this->get_logger(), "Publishing a new KeyFrame with id: %d. Stats: #MPs=%d, Map ID=%d, #MP matches=%d, #mspChildrens=%d, #msploopedges=%d, #mspMergeEdges=%d", pKF->mnId, pKF->GetMapPoints().size(), pKF->GetMap()->GetId(), pKF->GetMapPointMatches().size(), pKF->GetChilds().size(), pKF->GetLoopEdges().size(), pKF->GetMergeEdges().size());

      
    while(AtlasActionActive() || MapActionActive() || KeyFrameActionActive())
    {
    //    std::cout << "Map action is active, waiting..." << std::endl;
        usleep(3000);
    }
    
    SetKeyFrameAction(true);

    

    
    std::vector<ORB_SLAM3::KeyFrame*> mvpKeyFrames = pKF->GetMap()->GetAllKeyFrames();
    std::vector<ORB_SLAM3::MapPoint*> mvpMapPoints = pKF->GetMap()->GetAllMapPoints();
    
    std::set<ORB_SLAM3::KeyFrame*> mspKeyFrames(mvpKeyFrames.begin(), mvpKeyFrames.end());
    std::set<ORB_SLAM3::MapPoint*> mspMapPoints(mvpMapPoints.begin(), mvpMapPoints.end());
    
    {
        unique_lock<std::mutex> lock(mMutexKF);
        pKF->PreSave(mspKeyFrames, mspMapPoints, mspCameras);
    } 
   
    AddKeyFrame(pKF);
    AddMap(pKF->GetMap());
    
    for(ORB_SLAM3::MapPoint* pMP : pKF->GetMapPoints())
    {
        pMP->PreSave(mspKeyFrames, mspMapPoints);
        AddMapPoint(pMP);
    }

    orbslam3_interfaces::msg::KeyFrame mRosKF = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKF);
    mRosKF.target = mnTargetModule;
    mRosKF.from = mnTaskModule;
    keyframe_publisher_->publish(mRosKF);
    
    // Tracking: Update kf count since it does not receive any KFs
    if(mnTaskModule == 1)
        IncreaseKFCount(1);  

    //ORB_SLAM3::KeyFrame* tempKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(std::make_shared<orbslam3_interfaces::msg::KeyFrame>(msgKf), static_cast<ORB_SLAM3::KeyFrame*>(NULL));
    //Utility::CompareTwoSerializedObjects(tempKF, pKf);
    
    
    SetKeyFrameAction(false);
}


/*        Atlas        */
void SlamWrapperNode::publishAtlas(bool mbMerged, bool mbLoopCloser, std::vector<unsigned long int> mvMergedIds) {

    RCLCPP_INFO(this->get_logger(), "Publishing Atlas after Merge=%d or LoopClosure=%d", mbMerged, mbLoopCloser);
    
    SetAtlasAction(true);
    IncreaseKFCount(0);
    //publishEndMsg(); 
    orbslam3_interfaces::msg::Atlas mRosAtlas;
    mRosAtlas.mp_current_map = Converter::MapConverter::OrbMapToRosMap(mpAtlas_->GetCurrentMap());

    mRosAtlas.system_id = std::getenv("SLAM_SYSTEM_ID");
    mRosAtlas.from_module_id = 3;
    mRosAtlas.mb_map_merge = mbMerged;
    mRosAtlas.mb_loop_closer = mbLoopCloser;
    mRosAtlas.mv_merged_map_ids = mvMergedIds;
    
    atlas_publisher_->publish(mRosAtlas);

    SetAtlasAction(false);
    //if(mbMerged && !mvMergedIds.empty() && !mbLoopCloser)
    //{
    //    // Construct Atlas message for merging.
    //     

    //} else if(mbLoopCloser && !mbMerged) {
    //    // Loop closure but no merge
    //} else if(mbLoopCloser && mbMerged) {
    //    // both
    //}



}


/*        Map        */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
    
    if(GetKFCount() <= 1)
        return;
    
    while(AtlasActionActive() || KeyFrameActionActive() || MapActionActive())
    {
        //std::cout << "KF action is active, waiting..." << std::endl;
        usleep(3000);
    }
    
    // drop update if reset active
    if(GetResetStatus())
        return;

    SetMapAction(true);
    
    //std::unique_lock<std::mutex> lock(mMutexUpdateMap);
    for(ORB_SLAM3::KeyFrame* pKF : pM->GetAllKeyFrames())
    {
        AddKeyFrame(pKF);
    }

    for(ORB_SLAM3::MapPoint* pMP : pM->GetAllMapPoints())
    {
        AddMapPoint(pMP);
    }

    

    RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
    RCLCPP_INFO(this->get_logger(), "Map Stats : #KFs=%d, #MPs=%d, #RefMPs=%d", pM->KeyFramesInMap(), pM->MapPointsInMap(), pM->GetReferenceMapPoints().size());
    RCLCPP_INFO(this->get_logger(), "Map stats : delete MPs=%d, update MPs=%d, delete KFs=%d, update KFs=%d", pM->GetErasedMPIds().size(), pM->GetUpdatedMPIds().size(), pM->GetErasedKFIds().size(), pM->GetUpdatedKFIds().size());
     
    
    for(const auto& mnMPid : pM->GetErasedMPIds())
    {
      if(mORBMapPoints.find(mnMPid) != mORBMapPoints.end())
          EraseMapPoint(mORBMapPoints[mnMPid]);
    }
    
    for(const auto& mnKFid : pM->GetErasedKFIds())
    {
      if(mORBKeyFrames.find(mnKFid) != mORBKeyFrames.end())
          EraseKeyFrame(mORBKeyFrames[mnKFid]);
    }
    

    pM->PreSave(mspCameras);
    
    
    auto mRosMap = Converter::MapConverter::OrbMapToRosMap(pM);
    
    mRosMap.from_module_id = mnTaskModule; 

    map_publisher_->publish(mRosMap);
    
    pM->ClearErasedData();
    pM->ClearUpdatedKFIds();
    pM->ClearUpdatedMPIds();
    
    
    SetMapAction(false);
}


/*        MapPoint        */
void SlamWrapperNode::publishMapPoint(ORB_SLAM3::MapPoint* pMP) {
    //RCLCPP_INFO(this->get_logger(), "Publishing a new mappoint with id: %d", pMp->mnId);
    AddMapPoint(pMP);
    orbslam3_interfaces::msg::MapPoint mRosMP = Converter::MapPointConverter::ORBSLAM3MapPointToROS(pMP);
    mRosMP.system_id = std::getenv("SLAM_SYSTEM_ID");
    
    map_point_publisher_->publish(mRosMP);
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
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF) {
  
    if(mpRosKF->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    if(mnTaskModule == 3 && mpRosKF->target != mnTaskModule)
        return;
    //if(mnTaskModule == 0 && mpRosKF->from == 2)
    //    return;
    
    //if(mnTaskModule != mpRosKF->target && mnTaskModule != 0)
    //    return;

    //if(mpRosKF->mn_id%3==0 && mpRosKF->mn_id >0)
    //  return;
    
    if(mpLoopCloser_->CheckIfRunning())
        return;
      
    // Add time check which checks from header compared to current ros time if the frame gets old.
    while(AtlasActionActive() || MapActionActive() || KeyFrameActionActive())
    {
        std::cout << "Map action is active, waiting..." << std::endl;
        usleep(5000);
    }
    
    SetKeyFrameAction(true);
    
    mpLocalMapper_->AllowLocalMapping(false); 
    usleep(3000);
    //mpLocalMapper_->RequestStop();
    // Wait until Local Mapping has effectively stopped
    // Do not let it hang.
    //int iter=0;
    //while(!mpLocalMapper_->isStopped())
    //{
    //    if(iter >= 100)
    //    {
    //        mpLocalMapper_->Release();
    //        return;
    //    }

    //    usleep(1000);
    //    ++iter;
    //}


    for(ORB_SLAM3::Map* pM : mpAtlas_->GetAllMaps())
    {
      //  mORBMaps[pM->GetId()] = pM;

        for(ORB_SLAM3::KeyFrame* pKF : pM->GetAllKeyFrames())
        {
            AddKeyFrame(pKF);

        }

        for(ORB_SLAM3::MapPoint* pMP : pM->GetAllMapPoints())
        {
            AddMapPoint(pMP);  
        }
    }
    
    CalcMaxMapPointID();
    CalcMaxKeyFrameID();

    
    if(mpLoopCloser_->CheckIfRunning())
    {
        SetKeyFrameAction(false);
        return;
    }  


    RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d, for a map: %d, is LC running:%d", mpRosKF->mn_id, mpRosKF->mp_map_id, mpLoopCloser_->CheckIfRunning());

    if(mORBMaps.find(mpRosKF->mp_map_id) == mORBMaps.end())
    {
        RCLCPP_INFO(this->get_logger(), "NEW map created w/ KF.");
        
        mpAtlas_->CreateNewMap();
        for(ORB_SLAM3::Map* pM : mpAtlas_->GetAllMaps())
        {
            AddMap(pM);
        }
    }
    
    ORB_SLAM3::KeyFrame* pKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
    if(mORBKeyFrames.find(mpRosKF->mn_id) == mORBKeyFrames.end())
    {  
        unsigned long int mnLargestKFID = GetMaxKeyFrameID();
        if(mpRosKF->mn_id < mnLargestKFID)
            mpRosKF->mn_id = mnLargestKFID + 1;
        //unique_lock<std::mutex> lock(mMutexUpdateMap);
        pKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, pKF);
        pKF->SetORBVocabulary(m_SLAM->GetORBVocabulary());
        pKF->SetKeyFrameDatabase(mpKeyFrameDB);
        pKF->UpdateMap(mORBMaps[mpRosKF->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
        pKF->ComputeBoW(); 
        AddKeyFrame(pKF);
    } else {
        // Check where the object was last updated
        // If in lower priority module, discard
        if(mORBKeyFrames[mpRosKF->mn_id])
        {
            if(mpRosKF->mn_last_module >= mORBKeyFrames[mpRosKF->mn_id]->GetLastModule())
            {
                pKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, mORBKeyFrames[mpRosKF->mn_id]);
            }
        }
    }

    if(!pKF)
    {
        RCLCPP_INFO(this->get_logger(), "Dropping the KF=%d since we have updated version of it.", mpRosKF->mn_id);
        SetKeyFrameAction(false);  
        mpLocalMapper_->AllowLocalMapping(true); 
        return;
    }
    
    for(size_t i = 0; i < mpRosKF->mvp_map_points.size(); i++)
    {
        if(mpRosKF->mvp_map_points[i].mn_id > mnMaxMPId)
        {
            mnMaxMPId = mpRosKF->mvp_map_points[i].mn_id;
            ++mnMaxMPId;
        }  
          
    } 
    
    std::cout << "The largest MP ID=" << mnMaxMPId<< std::endl;

    for(size_t i = 0; i < mpRosKF->mvp_map_points.size(); i++)
    {
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[i]);
        
        ORB_SLAM3::MapPoint* pMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
        if(mORBMapPoints.find(mpRosMP->m_str_hex_id) != mORBMapPoints.end())
        {
            if(!mORBMapPoints[mpRosMP->m_str_hex_id])
            {
                EraseMapPoint(mORBMapPoints[mpRosMP->m_str_hex_id]);
                continue;
            }
            // Check where the object was last updated
            // If in lower priority module, discard
            if(mpRosMP->mn_last_module >= mORBMapPoints[mpRosMP->m_str_hex_id]->GetLastModule())
            {
                pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, mORBMapPoints[mpRosMP->m_str_hex_id]);
            }
        } else {
            //Check if received ID have smaller integer id than the biggest ID in the system
            // If so, increment that by one and set as the id
            if(mpRosMP->mn_id < mnMaxMPId) 
            {
                unique_lock<std::mutex> lock(mMutexMapPoint);
                ++mnMaxMPId;
                mpRosMP->mn_id = mnMaxMPId;
            }
            pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, pMP);
            AddMapPoint(pMP);
        }
        
        if(!pMP)
          continue;
        else
            pMP->UpdateMap(mORBMaps[mpRosMP->mp_map_id]);
    }
    
    bool bKFUnprocessed = false;
    for(const auto& mstrId : mpRosKF->mv_backup_map_points_id)
    {
        if(mstrId == "" || mstrId.length() < 6) 
            continue;

        if(!mORBMapPoints[mstrId])
            continue;
        
        mORBMapPoints[mstrId]->PostLoad(mORBKeyFrames, mORBMapPoints, &bKFUnprocessed, mspUnprocKFids);

        if(bKFUnprocessed) {
            std::cout << "Map Point if unprocessed, continuing..." << std::endl;

            startTimer();
            mpUnprocOrbKeyFrames[pKF->mnId] = mpRosKF;
            mspUnprocKFids.insert(pKF->mnId);   
          
            SetKeyFrameAction(false);  
            mpLocalMapper_->AllowLocalMapping(true); 
            return;
        }
    }

    pKF->PostLoad(mORBKeyFrames, mORBMapPoints, mORBCameras, &bKFUnprocessed, mspUnprocKFids );
      
    if(!bKFUnprocessed){
        ForwardKeyFrameToTarget(pKF, mpRosKF->target);
    } else {
        std::cout << "KeyFrame is unprocessed (All MPs are good), continuing..." << std::endl;

        startTimer();
        mpUnprocOrbKeyFrames[pKF->mnId] = mpRosKF;
        mspUnprocKFids.insert(pKF->mnId);   
    }

    SetKeyFrameAction(false);
    
    //mpLocalMapper_->Release();
    mpLocalMapper_->AllowLocalMapping(true); 
}




/*        Atlas        */
void SlamWrapperNode::GrabAtlas(const orbslam3_interfaces::msg::Atlas::SharedPtr mpRosAtlas) {
    if(mpRosAtlas->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;

    while(KeyFrameActionActive() || MapActionActive() || mpLoopCloser_->CheckIfRunning())
    {
    //    std::cout << "KF action is active, waiting..." << std::endl;
        usleep(3000);
    }

    SetAtlasAction(true);
    
    
    orbslam3_interfaces::msg::Map::SharedPtr mpRosNewMap = std::make_shared<orbslam3_interfaces::msg::Map>(mpRosAtlas->mp_current_map);
    
    if(mpRosAtlas->mb_map_merge)
    {
        RCLCPP_INFO(this->get_logger(), "Got Atlas. Maps Merged=%d. Current map=%d. Merged maps=%d,%d", mpRosAtlas->mb_map_merge, mpRosNewMap->mn_id, mpRosAtlas->mv_merged_map_ids[0], mpRosAtlas->mv_merged_map_ids[1]);
    } else if (mpRosAtlas->mb_loop_closer) {
        RCLCPP_INFO(this->get_logger(), "Got Atlas. LoopClosure=%d, Current map=%d.", mpRosAtlas->mb_loop_closer, mpRosNewMap->mn_id);

    }

    GrabMap(mpRosNewMap);

    if(mpRosAtlas->mb_map_merge)
    {
        unsigned long int mnBadMapId;
        for(unsigned long int mapId : mpRosAtlas->mv_merged_map_ids)
        {
          std::cout << mapId << ",";
            if(mapId != mpRosAtlas->mp_current_map.mn_id)
                mnBadMapId = mapId;
        }
        std::cout << std::endl;

        ORB_SLAM3::Map* pBadMap = mORBMaps[mnBadMapId];
        if(pBadMap)
        {
            mpAtlas_->SetMapBad(pBadMap);
            mpAtlas_->RemoveBadMaps();
        }

        mpAtlas_->GetCurrentMap()->ClearErasedData();
        mpAtlas_->GetCurrentMap()->ClearUpdatedKFIds();
        mpAtlas_->GetCurrentMap()->ClearUpdatedMPIds();
    }
    
    IncreaseKFCount(0);
    std::cout << "num of maps=" << mpAtlas_->GetAllMaps().size() << std::endl; 
    SetAtlasAction(false);
    //usleep(10000);
    //publishEndMsg();
}




/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr mpRosMap) {
    if(mpRosMap->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    std::cout << GetKFCount() << std::endl; 
    if(GetKFCount() <= 0)
        return;

    if(mpLoopCloser_->CheckIfRunning())
        return;

    while(KeyFrameActionActive() || MapActionActive())
    {
        std::cout << "KF action is active, waiting..." << std::endl;
        usleep(10000);
    }
    
    SetMapAction(true);

    //std::chrono::system_clock::time_point mcMapReceiveTime = std::chrono::system_clock::now();
    //if(mpTracker_->mcLastResetTimeStamp < mcMapReceiveTime)
    //{
    //    if(mpRosMap->msp_keyframes.size() > mpAtlas_->GetCurrentMap()->KeyFramesInMap())
    //    {
    //        RCLCPP_INFO(this->get_logger(), "Map was resetted before receiving update, do not accept it. receive time=%d, last reset time=%d", mcMapReceiveTime, mpTracker_->mcLastResetTimeStamp);

    //        SetMapAction(false);
    //        return;
    //    }
    //}

    if(mORBMaps.find(mpRosMap->mn_id) != mORBMaps.end())
    {
        RCLCPP_INFO(this->get_logger(), "Got an update for a map, id=%d, from module=%d, is LC running=%d. Stats: #KFs=%d, #MPs=%d", mpRosMap->mn_id, mpRosMap->from_module_id, mpLoopCloser_->CheckIfRunning(), mpRosMap->msp_keyframes.size(), mpRosMap->msp_map_points.size());
    } else { 
        RCLCPP_INFO(this->get_logger(), "Got a new map, id=%d, from module=%d, is LC running=%d. Stats: #KFs=%d, #MPs=%d", mpRosMap->mn_id, mpRosMap->from_module_id, mpLoopCloser_->CheckIfRunning(), mpRosMap->msp_keyframes.size(), mpRosMap->msp_map_points.size());
    }

    //{
    //    //unique_lock<mutex> lock(mMutexUpdateMap);
    //    for(ORB_SLAM3::Map* pM : mpAtlas_->GetAllMaps())
    //    {
    //      //  mORBMaps[pM->GetId()] = pM;

    //        for(ORB_SLAM3::KeyFrame* pKF : pM->GetAllKeyFrames())
    //        {
    //            AddKeyFrame(pKF);

    //        }

    //        for(ORB_SLAM3::MapPoint* pMP : pM->GetAllMapPoints())
    //        {
    //            AddMapPoint(pMP);  
    //        }
    //    }
    //}

    CalcMaxMapPointID();
    CalcMaxKeyFrameID();

    //bool bUnprocessed = false;
    ORB_SLAM3::Map* pM = static_cast<ORB_SLAM3::Map*>(NULL);
    if(mORBMaps.find(mpRosMap->mn_id) != mORBMaps.end())
    {
        //unique_lock<mutex> lock(mMutexUpdateMap);
        pM = mORBMaps[mpRosMap->mn_id];
        pM = Converter::MapConverter::RosMapToOrbMap(mpRosMap, pM);
    } else {
        mpAtlas_->CreateNewMap();
        pM=mpAtlas_->GetCurrentMap();
        AddMap(pM);
    }
    std::vector<std::string> mvpMPsDone;
    std::cout << "#KFs=" << mpRosMap->mvp_backup_keyframes_ids.size() << ", #updates=" << mpRosMap->msp_keyframes.size() << ", erased=" << mpRosMap->mvp_erased_keyframe_ids.size() << std::endl;
    std::cout << "#MPs=" << mpRosMap->mvp_backup_map_points_ids.size() << ", #updates=" << mpRosMap->msp_map_points.size() << ", erased=" << mpRosMap->mvp_erased_mappoint_ids.size() << std::endl;
    

    size_t kfFound=0;
    size_t kfNotFound=0;
    size_t mpFound=0;
    size_t mpNotFound=0;
    

    if(mpLoopCloser_->CheckIfRunning())
    {
        SetMapAction(false);
        return;
    }  

    mpTracker_->LocalMapIsUpdating(true); 
    for(size_t j = 0; j < mpRosMap->msp_keyframes.size(); j++)
    {
        //unique_lock<mutex> lock(mMutexUpdateMap);
        
        orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrame>(mpRosMap->msp_keyframes[j]);


        ORB_SLAM3::KeyFrame* pKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
        if(mORBKeyFrames.find(mpRosKF->mn_id) == mORBKeyFrames.end())
        {
            pKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, static_cast<ORB_SLAM3::KeyFrame*>(NULL));    
            pKF->SetORBVocabulary(m_SLAM->GetORBVocabulary());
            pKF->SetKeyFrameDatabase(mpKeyFrameDB);
            pKF->ComputeBoW();
            AddKeyFrame(pKF);
            kfNotFound+=1;
        } else {
            pKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, mORBKeyFrames[mpRosKF->mn_id]);
            kfFound+=1;
        }

        //mORBMaps[rM->mn_id]->AddKeyFrame(oKf);
        
        for(size_t i = 0; i < mpRosKF->mvp_map_points.size(); i++)
        {
            orbslam3_interfaces::msg::MapPoint::SharedPtr mp = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[i]);
            
            //std::cout << "Processing MP=" << mp->mn_id << std::endl; 
            ORB_SLAM3::MapPoint* pMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
            if(mORBMapPoints.find(mp->m_str_hex_id) == mORBMapPoints.end())
            {
                if(mp->mn_id <= mnMaxMPId) 
                {
                    unique_lock<std::mutex> lock(mMutexMapPoint);
                    ++mnMaxMPId;
                    mp->mn_id = mnMaxMPId;
                }
                //std::cout << " - MP=" << mp->m_str_hex_id << " not found, processing..."<< std::endl;
                pMP = Converter::MapPointConverter::RosMapPointToOrb(mp, static_cast<ORB_SLAM3::MapPoint*>(NULL));
                AddMapPoint(pMP);
                mpNotFound+=1;
            } else {
                //std::cout << " - MP=" << mp->m_str_hex_id << " found, processing..."<< std::endl;
                pMP = Converter::MapPointConverter::RosMapPointToOrb(mp, mORBMapPoints[mp->m_str_hex_id]);
                mpFound+=1;
            }
            
            mvpMPsDone.push_back(pMP->mstrHexId);
        }
    }

    
    for(size_t j = 0; j < mpRosMap->msp_map_points.size(); j++)
    {
        //unique_lock<mutex> lock(mMutexUpdateMap);
        std::string targetId = mpRosMap->msp_map_points[j].m_str_hex_id;
        
        auto it = std::find(mvpMPsDone.begin(), mvpMPsDone.end(), targetId);

        if(it == mvpMPsDone.end())
        {
            //std::cout << "MapPoint is in map but not in KF." << std::endl;
            orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosMap->msp_map_points[j]);
            
            
            ORB_SLAM3::MapPoint* pMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
            if(mORBMapPoints.find(mpRosMP->m_str_hex_id) == mORBMapPoints.end())
            {
                if(mpRosMP->mn_id <= mnMaxMPId) 
                {
                    unique_lock<std::mutex> lock(mMutexMapPoint);
                    ++mnMaxMPId;
                    mpRosMP->mn_id = mnMaxMPId;
                }
                //std::cout << " - MP=" << mp->m_str_hex_id << " not found, processing..."<< std::endl;
                pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, pMP);
                AddMapPoint(pMP);
                mpNotFound+=1;
            } else {
                //std::cout << " - MP=" << mp->m_str_hex_id << " found, processing..."<< std::endl;
                pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, mORBMapPoints[mpRosMP->m_str_hex_id]);
                mpFound+=1;
            }

            mvpMPsDone.push_back(pMP->mstrHexId);
        } //else {
    }
    
    //mpTracker_->LocalMapIsUpdating(true);
    std::cout << "#KFs found=" << kfFound << ", # not found=" << kfNotFound << ", #MPs found=" << mpFound << ", # not found=" << mpNotFound << std::endl;
    bool bKFUnprocessed = false;
    
    //unique_lock<mutex> lock(mMutexUpdateMap);
    pM->PostLoad(mpKeyFrameDB, m_SLAM->GetORBVocabulary(), mORBKeyFrames, mORBMapPoints, mORBCameras, &bKFUnprocessed, mspUnprocKFids);
    
    if(mpLoopCloser_->CheckIfRunning())
    {
        SetMapAction(false);
        return;
    }  
    
    mpTracker_->LocalMapIsUpdating(false); 
    
    // Remove KFs which have been removed on the other machine
    for(const auto& kfId : mpRosMap->mvp_erased_keyframe_ids)
    {
        if(mORBKeyFrames.find(kfId) != mORBKeyFrames.end())
        {
            if(mORBKeyFrames[kfId])
            {
                mORBKeyFrames[kfId]->SetBadFlag();
                EraseKeyFrame(mORBKeyFrames[kfId]);
            }
        }
    }
   
    // Remove MapPoints which have been removed on the other machine
    for(const auto& mpId : mpRosMap->mvp_erased_mappoint_ids)
    {
        if(mORBMapPoints.find(mpId) != mORBMapPoints.end())
        {
            if(mORBMapPoints[mpId])
            {
                //mORBMapPoints[mpId]->SetBadFlag();
                EraseMapPoint(mORBMapPoints[mpId]);
            }
        }
    }
    
    
    if(bKFUnprocessed) 
        std::cout << "Map with ID=" << pM->GetId() << " is unprocessed!!" << std::endl;
    
    // only tracking needs to update all the stuff there
    if(mnTaskModule == 1) {
        mpTracker_->UpdateFromLocalMapping(mORBMaps[mpRosMap->mn_id], mORBKeyFrames, mORBMapPoints);
    }
 
    pM->ClearErasedData();
    pM->ClearUpdatedKFIds();
    
    SetMapAction(false);
}

/*        MapPoint        */
void SlamWrapperNode::GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP) {  
    if(mpRosMP->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    RCLCPP_INFO(this->get_logger(), "Got a new mappoint, id: %d", mpRosMP->mn_id);
    bool bUnprocessed = false;
    
    ORB_SLAM3::MapPoint* pMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
    if(mORBMapPoints.find(mpRosMP->m_str_hex_id) == mORBMapPoints.end())
        pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, pMP);
    else
        pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, mORBMapPoints[mpRosMP->m_str_hex_id]);

    if(bUnprocessed) {
        mpUnprocOrbMapPoints[pMP->mstrHexId] = std::make_tuple(pMP, mpRosMP);
    } 
    
    AddMapPoint(pMP);
}


/*        System Reset Active Map*/
void SlamWrapperNode::GrabResetActiveMap(const orbslam3_interfaces::msg::Int64::SharedPtr msg) {
    
    if(msg->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    RCLCPP_INFO(this->get_logger(), "Reset requested for Active map id=%d", msg->data);
    

    while(AtlasActionActive() || MapActionActive() || KeyFrameActionActive())
    {
        usleep(3000);
    }
    
    UpdateReset(true);
    
    mpLocalMapper_->Release();
    mpTracker_->ResetActiveMap(false, msg->data);
    
    //mORBMaps.clear();
    //mORBKeyFrames.clear();
    //mORBMapPoints.clear();
  
    UpdateReset(false);
}


/*        LocalMapping reset        */
void SlamWrapperNode::GrabLMResetRequested(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Reset requested for Local mapping=%d", msg->data);

    mpLocalMapper_->RequestReset();
    

    stopTimer();
    mspKFsReadyForLM.clear();
    mpUnprocOrbKeyFrames.clear();
    mORBMaps.clear();
    mORBKeyFrames.clear();
    mORBMapPoints.clear();

    mpKeyFrameDB->clear();
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
    RCLCPP_INFO(this->get_logger(), "Publishing to /Destroy");
    std_msgs::msg::Bool endMsg;
    endMsg.data = true;
    
    end_publisher_->publish(endMsg);
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
        rclcpp::QoS(rclcpp::KeepLast(20), rmw_qos_profile_sensor_data));
    
    /* MAP */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
    map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
        "/Map", 
        rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data));
    
    /* MAPPOINT */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /MapPoint");
    map_point_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapPoint>(
        "/MapPoint", 
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default));
    
    /* ATLAS */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Atlas");
    atlas_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Atlas>(
        "/Atlas", 
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default));
    
    /* LocalMapping Active*/
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /LocalMapping/Active");
    lm_active_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Bool>(
        "/LocalMapping/Active", 
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default));

    /* LocalMapping Active*/
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /LocalMapping/Reset");
    lm_reset_requested_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/LocalMapping/Reset", 
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default));


    /* Reset Active Map (System) */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map/Active/Reset");
    sys_reset_active_map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Int64>(
        "/Map/Reset/Active", 
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default));

    /* Destroy */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /destroy");
    end_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/destroy", 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));

    /* Step */
    //RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /step");
    //step_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    //    "/step", 
    //    10);


}







/*   INIT SUBSCRIBERS   */

void SlamWrapperNode::CreateSubscribers() {
      
    /* KF */  
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
    m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
        "KeyFrame",
        rclcpp::QoS(rclcpp::KeepLast(20), rmw_qos_profile_sensor_data),
        std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));
    
    /* Map */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
    m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
        "Map",
        rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data),
        std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));

    /* Atlas */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Atlas");
    m_atlas_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Atlas>(
        "Atlas",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabAtlas, this, std::placeholders::_1));
    
    /* MapPoint */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /MapPoint");
    m_map_point_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::MapPoint>(
        "MapPoint",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabMapPoint, this, std::placeholders::_1));
    

    /* LocalMapping active */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Active");
    m_lm_active_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Bool>(
        "LocalMapping/Active",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabLMActive, this, std::placeholders::_1));

    /* LocalMapping reset requested */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Reset");
    m_lm_reset_requested_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "LocalMapping/Reset",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabLMResetRequested, this, std::placeholders::_1));

    /* LocalMapping reset requested */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map/Reset/Active");
    m_sys_reset_active_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Int64>(
        "Map/Reset/Active",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabResetActiveMap, this, std::placeholders::_1));
    

    /* Destroy node */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /destroy");
    end_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/destroy", 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE), 
        std::bind(&SlamWrapperNode::endCallback, this, std::placeholders::_1));
    
    /* Step */
    //RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /step");
    //step_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    //    "/step", 
    //    10, 
    //    std::bind(&SlamWrapperNode::stepCallback, this, std::placeholders::_1));
}
