#include "slam-wrapper-node.hpp"
#include "./Distributor.hpp"

#include "../Distributor/System.hpp"
#include "../Distributor/Observer.hpp"
#include "../Distributor/MapHandler.hpp"
#include "../Distributor/KeyFramePublisher.hpp"
#include "../Distributor/KeyFrameSubscriber.hpp"

#include <cstdlib>
#include <stdlib.h>
#include <rmw/qos_profiles.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM, std::shared_ptr<System> pDistSystem, bool subscribe_to_slam, const std::string path, const std::string strResultFilename ) : Node("SlamWrapperNode") {
    
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

    // Attach Distribution pointers
    mpDistributionSystem=pDistSystem;
    mpObserver=pDistSystem->GetObserver();
    mpMapHandler=pDistSystem->GetMapHandler();
    mpKeyFramePublisher=pDistSystem->GetKeyFramePublisher();
    mpKeyFrameSubscriber=pDistSystem->GetKeyFrameSubscriber();

    // Attach ORB SLAM3 System pointers
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
    
    std_msgs::msg::Int32 wMsg;
    wMsg.data = mnTaskModule;
    
    worker_publisher_->publish(wMsg);

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
            ForwardKeyFrameToTarget(pKF, 2, 0);
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







void SlamWrapperNode::ForwardKeyFrameToTarget(ORB_SLAM3::KeyFrame* pKF, const unsigned int mnTargetModule, const unsigned int nFromModule)
{
    // Just update state and do not insert to any module
    if(mnTaskModule == 0) {
        //for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
        //{
        //    if(pMPi)
        //        mpAtlas_->AddMapPoint(pMPi);
        //}
        
        if(pKF)
        {
            //IncreaseKFCount(1);  
            if(nFromModule == 3)
            {
                mpKeyFrameDB->erase(pKF);
                mpKeyFrameDB->add(pKF);
            }

            mpAtlas_->AddKeyFrame(pKF);
        }
    } else if (mnTaskModule == 1) {
        // in this case the system is mainly performing tracking
        // For now, tracking does not grab any KFs, only Maps 
        if(pKF)
        {
            if(nFromModule == 3)
            {
                mpKeyFrameDB->erase(pKF);
                mpKeyFrameDB->add(pKF);
            }
            mpAtlas_->AddKeyFrame(pKF);
        }
    } else if (mnTaskModule == 2) {
        // in this case the system mainly performing local mapping
        if(mnTargetModule == 2) {
            // if the KF is meant to be inserted to LM
            // Insert to Local Mapping
            if(mpLocalMapper_->NeedNewKeyFrame(pKF))
            {
                //mpKeyFrameDB->add(pKF);
                //IncreaseKFCount(1);  
                mpLocalMapper_->InsertKeyframeFromRos(pKF);
            } 
            else {
                //for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
                //{
                //    if(pMPi)
                //        mpAtlas_->AddMapPoint(pMPi);
                //}
                
                if(pKF)
                {
                    //IncreaseKFCount(1);  
                    if(nFromModule == 3)
                    {
                        mpKeyFrameDB->erase(pKF);
                        mpKeyFrameDB->add(pKF);
                    }
                    mpAtlas_->AddKeyFrame(pKF);
                }
            //    mspKFsReadyForLM.insert(pKF->mnId);
            //    startTimer();
                //pKF->SetBadFlag();
            }  
        } else {

            if(pKF)
            {
                if(nFromModule == 3)
                {
                    mpKeyFrameDB->erase(pKF);
                    mpKeyFrameDB->add(pKF);
                }
                mpAtlas_->AddKeyFrame(pKF);
            }
        }    
    } else if (mnTaskModule == 3) {
        // in this case the system mainly performing loop closing
        // not sure what to do here yet. 
        if(mnTargetModule == 3) {
            if(pKF)
            {
                //for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
                //{
                //    if(pMPi)
                //        mpAtlas_->AddMapPoint(pMPi);
                //}
                //IncreaseKFCount(1);  
                mpAtlas_->AddKeyFrame(pKF);
                pKF->UpdateConnections();
                mpLoopCloser_->InsertKeyFrame(pKF);
            }
        } else {

            //for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
            //{
            //    if(pMPi)
            //        mpAtlas_->AddMapPoint(pMPi);
            //}
            
            if(pKF)
            {
                //mpKeyFrameDB->add(pKF);
                mpAtlas_->AddKeyFrame(pKF);
            }
        }
    }
}

/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(orbslam3_interfaces::msg::KeyFrame mRosKF) {
    
    keyframe_publisher_->publish(mRosKF);
    
    RCLCPP_INFO(this->get_logger(), "Publishing a new KeyFrame with id: %d. Stats: #MPs=%d, Map ID=%d", mRosKF.mn_id, mRosKF.mvp_map_points.size(), mRosKF.mp_map_id);
}


/*        Atlas        */
void SlamWrapperNode::publishAtlas(orbslam3_interfaces::msg::Atlas mRosAtlas) {

    
    //SetAtlasAction(true);
    //IncreaseKFCount(0);
    ////publishEndMsg(); 
    //orbslam3_interfaces::msg::Atlas mRosAtlas;
    //mRosAtlas.mp_current_map = Converter::MapConverter::OrbMapToRosMap(mpAtlas_->GetCurrentMap());

    //mRosAtlas.system_id = std::getenv("SLAM_SYSTEM_ID");
    //mRosAtlas.from_module_id = 3;
    //mRosAtlas.mb_map_merge = mbMerged;
    //mRosAtlas.mb_loop_closer = mbLoopCloser;
    //mRosAtlas.mv_merged_map_ids = mvMergedIds;
    
    atlas_publisher_->publish(mRosAtlas);

    RCLCPP_INFO(this->get_logger(), " ********************* Publishing Atlas after Merge=%d or LoopClosure=%d, #KFs=%d", mRosAtlas.mb_map_merge, mRosAtlas.mb_loop_closer, mpAtlas_->GetCurrentMap()->KeyFramesInMap());
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
void SlamWrapperNode::publishMap(orbslam3_interfaces::msg::Map mRosMap) {
    
    map_publisher_->publish(mRosMap);
    
    RCLCPP_INFO(this->get_logger(), "Publishing a new map with id: %d", mRosMap.mn_id);
    RCLCPP_INFO(this->get_logger(), "Map Stats : #updated KFs=%d, #updated MPs=%d, #del KFs=%d, #del MPs=%d", 
        mRosMap.msp_keyframes.size(), 
        mRosMap.msp_map_points.size(), 
        mRosMap.mvp_erased_keyframe_ids.size(), 
        mRosMap.mvp_erased_mappoint_ids.size());
}

///*        Map        */
//void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
//    
//    if(GetKFCount() <= 1)
//        return;
//    
//    while(AtlasActionActive() || KeyFrameActionActive() || MapActionActive())
//    {
//        //std::cout << "KF action is active, waiting..." << std::endl;
//        usleep(3000);
//    }
//    
//    // drop update if reset active
//    if(GetResetStatus())
//        return;
//
//    SetMapAction(true);
//    
//    //std::unique_lock<std::mutex> lock(mMutexUpdateMap);
//    for(ORB_SLAM3::KeyFrame* pKF : pM->GetAllKeyFrames())
//    {
//        AddKeyFrame(pKF);
//    }
//
//    for(ORB_SLAM3::MapPoint* pMP : pM->GetAllMapPoints())
//    {
//        AddMapPoint(pMP);
//    }
//
//    
//
//    RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
//    RCLCPP_INFO(this->get_logger(), "Map Stats : #KFs=%d, #MPs=%d, #RefMPs=%d", pM->KeyFramesInMap(), pM->MapPointsInMap(), pM->GetReferenceMapPoints().size());
//    RCLCPP_INFO(this->get_logger(), "Map stats : delete MPs=%d, update MPs=%d, delete KFs=%d, update KFs=%d", pM->GetErasedMPIds().size(), pM->GetUpdatedMPIds().size(), pM->GetErasedKFIds().size(), pM->GetUpdatedKFIds().size());
//     
//    
//    for(const auto& mnMPid : pM->GetErasedMPIds())
//    {
//      if(mORBMapPoints.find(mnMPid) != mORBMapPoints.end())
//          EraseMapPoint(mORBMapPoints[mnMPid]);
//    }
//    
//    for(const auto& mnKFid : pM->GetErasedKFIds())
//    {
//      if(mORBKeyFrames.find(mnKFid) != mORBKeyFrames.end())
//          EraseKeyFrame(mORBKeyFrames[mnKFid]);
//    }
//    
//
//    pM->PreSave(mspCameras);
//    
//    
//    auto mRosMap = Converter::MapConverter::OrbMapToRosMap(pM);
//    
//    mRosMap.from_module_id = mnTaskModule; 
//
//    map_publisher_->publish(mRosMap);
//    
//    pM->ClearErasedData();
//    pM->ClearUpdatedKFIds();
//    pM->ClearUpdatedMPIds();
//    
//    
//    SetMapAction(false);
//}


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

ORB_SLAM3::MapPoint* SlamWrapperNode::ProcessNewMapPoint(std::shared_ptr<orbslam3_interfaces::msg::MapPoint> mpRosMP, std::map<unsigned long int, ORB_SLAM3::Map*> mMaps)
{
    ORB_SLAM3::MapPoint* tempMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, static_cast<ORB_SLAM3::MapPoint*>(NULL));
    if(mMaps[mpRosMP->mp_map_id])
      tempMP->UpdateMap(mMaps[mpRosMP->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    else
      tempMP->UpdateMap(static_cast<ORB_SLAM3::Map*>(NULL));
    return tempMP;
}

ORB_SLAM3::KeyFrame* SlamWrapperNode::ProcessNewKeyFrame(std::shared_ptr<orbslam3_interfaces::msg::KeyFrame> mpRosKF, std::map<unsigned long int, ORB_SLAM3::Map*> mMaps)
{
    ORB_SLAM3::KeyFrame* tempKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, static_cast<ORB_SLAM3::KeyFrame*>(NULL));
    tempKF->SetORBVocabulary(m_SLAM->GetORBVocabulary());
    tempKF->SetKeyFrameDatabase(mpKeyFrameDB);
    if(mMaps[mpRosKF->mp_map_id])
      tempKF->UpdateMap(mMaps[mpRosKF->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    else
      tempKF->UpdateMap(static_cast<ORB_SLAM3::Map*>(NULL));
    tempKF->ComputeBoW(); 
    return tempKF;
}

ORB_SLAM3::KeyFrame* SlamWrapperNode::InjectKeyFrame(ORB_SLAM3::KeyFrame* tempKF, std::map<unsigned long int, ORB_SLAM3::KeyFrame*> mMapKFs)
{ 
    if(mMapKFs.find(tempKF->mnId) == mMapKFs.end())
    {
        return tempKF;
    } else {
        // copy tempKF -> KF
        // Create copy constructor
        // Delete tempKF
        ORB_SLAM3::KeyFrame* pKF = mMapKFs[tempKF->mnId];
        pKF->UpdateKeyFrame(*tempKF);
        delete tempKF;
        return pKF; 
    }
}

void SlamWrapperNode::InjectMapPoint(ORB_SLAM3::MapPoint* tempMP, std::map<std::string, ORB_SLAM3::MapPoint*> mMapMPs)
{
    if(mMapMPs.find(tempMP->mstrHexId) == mMapMPs.end())
    {
        ORB_SLAM3::Map* pMap = tempMP->GetMap();
        if(pMap)
          pMap->AddMapPoint(tempMP);
    } else {
        // copy tempMP -> MP
        // Create copy constructor
        // Delete tempMP
        ORB_SLAM3::MapPoint* pMP = mMapMPs[tempMP->mstrHexId];
        pMP->UpdateMapPoint(*tempMP);
        delete tempMP;
    }
}

void SlamWrapperNode::InjectMap(ORB_SLAM3::Map* tempMap, ORB_SLAM3::Map* mpCurrentMap)
{
    // copy tempMP -> MP
    // Create copy constructor
    // Delete tempMP
    mpCurrentMap->UpdateMap(*tempMap);
    delete tempMap;
}
/* OBJECT SUBSCRIPTION CALLBACKS */

/*        KeyFrame        */
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF) {
  
    if(mpRosKF->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    if(mnTaskModule == 3 && mpRosKF->target != mnTaskModule)
        return;

    RCLCPP_INFO(this->get_logger(), " **************************** Got a new keyframe, id: %d, for a map: %d", mpRosKF->mn_id, mpRosKF->mp_map_id);
    
    mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKF);


    //// Get maps and the map where KF is
    //ORB_SLAM3::Map* pCurrentMap = static_cast<ORB_SLAM3::Map*>(NULL);
    //std::map<unsigned long int, ORB_SLAM3::Map*> mMaps;
    //for(ORB_SLAM3::Map* pM : mpAtlas_->GetAllMaps())
    //{
    //    if(pM->GetId() == mpRosKF->mp_map_id)
    //    {
    //        pCurrentMap = pM;
    //    }
    //    mMaps[pM->GetId()] = pM;
    //}

    //if(!pCurrentMap)
    //{
    //    mpAtlas_->CreateNewMap();
    //    pCurrentMap = mpAtlas_->GetCurrentMap(); 
    //    mMaps[pCurrentMap->GetId()] = pCurrentMap;
    //}

    //// Create map data structures for postloads
    //std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    //std::map<std::string, ORB_SLAM3::MapPoint*> mMapMPs; 
    //std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mFusedKFs; 
    //std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    //
    //for(ORB_SLAM3::KeyFrame* pKF : pCurrentMap->GetAllKeyFrames())
    //{
    //  mFusedKFs[pKF->mnId] = pKF;
    //}

    //for(ORB_SLAM3::MapPoint* pMP : pCurrentMap->GetAllMapPoints())
    //{
    //  mFusedMPs[pMP->mstrHexId] = pMP;
    //  mMapMPs[pMP->mstrHexId] = pMP;
    //}

    //for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas_->GetAllCameras())
    //{
    //  mCameras[pCam->GetId()] = pCam;
    //}
    //
    //RCLCPP_INFO(this->get_logger(), " Got a new keyframe. Existing data is collected.");
    //
    //// Convert ros msg to ORB_SLAM3 Objects
    //ORB_SLAM3::KeyFrame* tempKF = ProcessNewKeyFrame(mpRosKF, mMaps);
    //std::vector<ORB_SLAM3::MapPoint*> mvpTempMPs;
    //mvpTempMPs.resize(mpRosKF->mvp_map_points.size());
    //
    //for(int i=0; i<mvpTempMPs.size(); ++i)
    //{
    //    mvpTempMPs[i] = ProcessNewMapPoint(std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[i]), mMaps);
    //    if(mFusedMPs.find(mvpTempMPs[i]->mstrHexId) == mFusedMPs.end())
    //        mFusedMPs[mvpTempMPs[i]->mstrHexId] = mvpTempMPs[i];
    //}
    //
    //RCLCPP_INFO(this->get_logger(), " Got a new keyframe. New data is fused with old data.");

    //// Postloads -> Reassign pointers etc to temporary data
    //bool bKFUnprocessed = false;
    //tempKF->PostLoad(mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);

    //// -------------------- From here on we have unprocessed data -----------------------
    //// Here the data is injected into existing KF or the same one is returned. Function handles pointers etc.
    //ORB_SLAM3::KeyFrame* pKF = InjectKeyFrame(tempKF, mFusedKFs);
    //mFusedKFs[pKF->mnId] = pKF;
    //
    //RCLCPP_INFO(this->get_logger(), " Got a new keyframe. KF PostLoad, and injection is completed.");
    //
    //for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    //{
    //    tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    //}
    //
    //RCLCPP_INFO(this->get_logger(), " Got a new keyframe. All MP PostLoads Completed.");
    //
    //for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    //{
    //    InjectMapPoint(tempMP, mMapMPs);
    //}
    //// -------------------- To here -----------------------
    //
    //RCLCPP_INFO(this->get_logger(), " Got a new keyframe. All MPs are injected to ORB_SLAM3.");
    //
    //
    //// If KF exists, forward it to target
    //if(pKF)
    //  ForwardKeyFrameToTarget(pKF, mpRosKF->target, mpRosKF->from);
    //
    //RCLCPP_INFO(this->get_logger(), " Got a new keyframe. KF is forwarded to the target. KF is COMPLETE.");

}




/*        Atlas        */
void SlamWrapperNode::GrabAtlas(const orbslam3_interfaces::msg::Atlas::SharedPtr mpRosAtlas) {
    if(mpRosAtlas->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    RCLCPP_INFO(this->get_logger(), "Got Atlas Update, merge=%d, closure=%d", mpRosAtlas->mb_map_merge, mpRosAtlas->mb_loop_closer);
    
    mpMapHandler->InsertNewSubGlobalMap(mpRosAtlas);

    //orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = std::make_shared<orbslam3_interfaces::msg::Map>(mpRosAtlas->mp_current_map);
    //
    //if(mpRosAtlas->mb_map_merge)
    //{
    //    RCLCPP_INFO(this->get_logger(), "Got Atlas. Maps Merged=%d. Current map=%d. Merged maps=%d,%d", mpRosAtlas->mb_map_merge, mpRosMap->mn_id, mpRosAtlas->mv_merged_map_ids[0], mpRosAtlas->mv_merged_map_ids[1]);
    //} else if (mpRosAtlas->mb_loop_closer) {
    //    RCLCPP_INFO(this->get_logger(), "Got Atlas. LoopClosure=%d, Current map=%d.", mpRosAtlas->mb_loop_closer, mpRosMap->mn_id);

    //} else if(!mpRosAtlas->mb_loop_closer && !mpRosAtlas->mb_map_merge) {

    //    RCLCPP_INFO(this->get_logger(), "Got Atlas. General update with #KFs=%d", mpRosMap->msp_keyframes.size());
    //}


    //// Create map data structures for postloads
    //std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    //std::map<std::string, ORB_SLAM3::MapPoint*> mMapMPs;

    //std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mFusedKFs; 
    //std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mMapKFs; 
    //
    //std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    //// Get maps and the map where KF is
    //ORB_SLAM3::Map* pMergeMap = static_cast<ORB_SLAM3::Map*>(NULL);
    //ORB_SLAM3::Map* pCurrentMap = static_cast<ORB_SLAM3::Map*>(NULL);
    //std::map<unsigned long int, ORB_SLAM3::Map*> mMaps;
    //for(ORB_SLAM3::Map* pM : mpAtlas_->GetAllMaps())
    //{
    //    if(mpRosAtlas->mb_map_merge && mpRosAtlas->mv_merged_map_ids.size() >= 2)
    //    {

    //        if(pM->GetId() == mpRosAtlas->mv_merged_map_ids[1])
    //        {
    //            pCurrentMap = pM;
    //        }
    //        
    //        if(pM->GetId() == mpRosAtlas->mv_merged_map_ids[0])
    //        {
    //            pMergeMap = pM;
    //        }
    //    } else {
    //        if(pM->GetId() == mpRosMap->mn_id)
    //        {
    //            pCurrentMap = pM;
    //        }
    //    }

    //    mMaps[pM->GetId()] = pM;
    //}
    //
    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 1. Maps collected.");

    ////if(!pCurrentMap)
    ////{
    ////    mpAtlas_->CreateNewMap();
    ////    pCurrentMap = mpAtlas_->GetCurrentMap(); 
    ////    mMaps[pCurrentMap->GetId()] = pCurrentMap;
    ////}

    //
    //if(mpRosAtlas->mb_map_merge)
    //{

    //    
    //    // Collect all KFs from current and merged maps
    //    for(ORB_SLAM3::KeyFrame* pKF : pCurrentMap->GetAllKeyFrames())
    //    {
    //      mFusedKFs[pKF->mnId] = pKF;
    //      mMapKFs[pKF->mnId] = pKF;
    //    }
    //    
    //    for(ORB_SLAM3::KeyFrame* pKF : pMergeMap->GetAllKeyFrames())
    //    {
    //      mFusedKFs[pKF->mnId] = pKF;
    //      mMapKFs[pKF->mnId] = pKF;
    //    }

    //    // Collect all MPs from current and merged maps
    //    for(ORB_SLAM3::MapPoint* pMP : pCurrentMap->GetAllMapPoints())
    //    {
    //      mFusedMPs[pMP->mstrHexId] = pMP;
    //      mMapMPs[pMP->mstrHexId] = pMP;
    //    }

    //    for(ORB_SLAM3::MapPoint* pMP : pMergeMap->GetAllMapPoints())
    //    {
    //      mFusedMPs[pMP->mstrHexId] = pMP;
    //      mMapMPs[pMP->mstrHexId] = pMP;
    //    }

    //    // Collect all cameras
    //    for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas_->GetAllCameras())
    //    {
    //      mCameras[pCam->GetId()] = pCam;
    //    }

    //} else {

    //    for(ORB_SLAM3::KeyFrame* pKF : pCurrentMap->GetAllKeyFrames())
    //    {
    //      mFusedKFs[pKF->mnId] = pKF;
    //      mMapKFs[pKF->mnId] = pKF;
    //    }

    //    for(ORB_SLAM3::MapPoint* pMP : pCurrentMap->GetAllMapPoints())
    //    {
    //      mFusedMPs[pMP->mstrHexId] = pMP;
    //      mMapMPs[pMP->mstrHexId] = pMP;
    //    }

    //    for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas_->GetAllCameras())
    //    {
    //      mCameras[pCam->GetId()] = pCam;
    //    }
    //}

    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 2. ORB_SLAM3 data collected.");

    //
    //std::vector<ORB_SLAM3::KeyFrame*> mvpTempKFs;
    //mvpTempKFs.resize(mpRosMap->msp_keyframes.size());
    //std::map<std::string, ORB_SLAM3::MapPoint*> mTempMPs;
    //
    //// Sort the keyframes from smallest id to largest
    //// So that postloads should be done in order
    //std::vector<orbslam3_interfaces::msg::KeyFrame> mvRosKF = mpRosMap->msp_keyframes;
    //std::sort(mvRosKF.begin(), mvRosKF.end(), [](const orbslam3_interfaces::msg::KeyFrame& a, const orbslam3_interfaces::msg::KeyFrame& b) {
    //    return a.mn_id < b.mn_id;
    //});

    //// Loop through each KF
    //// Convert to ORB
    //// Add to mFusedKFs if not found -> if its not there then its a new object and can be used as it is for other objects
    //// if its there, then use original pointer in postload and just update that object afterwards
    //for(size_t i = 0; i<mvRosKF.size(); ++i)
    //{
    //    orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrame>(mvRosKF[i]);
    //    ORB_SLAM3::KeyFrame* tempKF = ProcessNewKeyFrame(mpRosKF, mMaps);
    //    mvpTempKFs[i] = tempKF;

    //    if(mFusedKFs.find(tempKF->mnId) == mFusedKFs.end())
    //        mFusedKFs[tempKF->mnId] = tempKF;
    //    
    //    // Do same stuff for each MP
    //    // Skip MPs which were already found in this map update (same objects in original map)
    //    for(size_t j = 0; j<mpRosKF->mvp_map_points.size(); ++j)
    //    {
    //        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[j]);
    //        // This MP was already seen in the map, skip since both points to the same pointer in the original
    //        if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
    //            continue;
    //        
    //        ORB_SLAM3::MapPoint* tempMP = ProcessNewMapPoint(mpRosMP, mMaps);
    //        mTempMPs[tempMP->mstrHexId] = tempMP;

    //        if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
    //            mFusedMPs[tempMP->mstrHexId] = tempMP;
    //    }
    //}


    //// There might be MPs which were not associated with KFs
    //// Process those as well
    //for(size_t i = 0; i<mpRosMap->msp_map_points.size(); ++i)
    //{
    //    orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosMap->msp_map_points[i]);

    //    if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
    //        continue;

    //    ORB_SLAM3::MapPoint* tempMP = ProcessNewMapPoint(mpRosMP, mMaps);
    //    mTempMPs[tempMP->mstrHexId] = tempMP;

    //    if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
    //        mFusedMPs[tempMP->mstrHexId] = tempMP;
    //}

    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 3. New data fused with the existing.");
    //
    //// PostLoad for all the KFs
    //bool bKFUnprocessed = false;
    //for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    //{
    //  ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
    //  tempKF->PostLoad(mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    //}

    //// PostLoad for all the MPs
    //for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    //{
    //  ORB_SLAM3::MapPoint* tempMP = it->second;
    //  tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    //}

    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 4. PostLoads (KF + MPs) completed.");
    //
    ////mpLocalMapper_->RequestStop();
    ////// Wait till stopped
    ////while(!mpLocalMapper_->isStopped())
    ////{
    ////    usleep(1000);
    ////}

    //// Inject KFs
    //for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    //{
    //  ORB_SLAM3::MapPoint* tempMP = it->second;
    //  InjectMapPoint(tempMP, mMapMPs);
    //}
    //
    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 5. MPs injected into ORB_SLAM3");
    //
    //for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    //{
    //  ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
    //  ORB_SLAM3::KeyFrame* pKF = InjectKeyFrame(tempKF, mMapKFs);
    //  if(pKF)
    //  {
    //      ForwardKeyFrameToTarget(pKF, 0, mpRosAtlas->from_module_id);
    //  }
    //}
    //
    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 6. KFs injected into ORB_SLAM3");
    //
    ////mpLocalMapper_->Release();
    //// Remove KFs and MPs
    //for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
    //{
    //    unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
    //    ORB_SLAM3::KeyFrame* pEraseKF=mFusedKFs[mnId];
    //    if(pEraseKF)
    //        pEraseKF->SetBadFlag();
    //}

    //for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
    //{
    //  std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
    //    ORB_SLAM3::MapPoint* pEraseMP=mFusedMPs[mnId];
    //    if(pEraseMP)
    //        pEraseMP->SetBadFlag();
    //}
    //
    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 7. KFs and MPs removed");

    //// Finally update Map object 
    //// First create temporary Map from ROS data
    //// Then Inject the data into real map (do not update KFs and MPs since those are already added)
    //ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    //tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);
    //tempMap->PostLoad(mpKeyFrameDB, m_SLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    //InjectMap(tempMap, pCurrentMap);
    //
    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 8. Map Injection completed");

    //if(mpRosAtlas->mb_map_merge && pMergeMap)
    //{
    //    mpAtlas_->SetMapBad(pMergeMap);
    //    mpAtlas_->RemoveBadMaps();
    //}

    //RCLCPP_INFO(this->get_logger(), "Got Atlas Update. 9. Bad maps are removed and GrabAtlas COMPLETE.");

}




/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr mpRosMap) {
    if(mpRosMap->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;

    RCLCPP_INFO(this->get_logger(), "Got an update for a map, id=%d, from module=%d. Stats: #KFs=%d, #MPs=%d", mpRosMap->mn_id, mpRosMap->from_module_id, mpRosMap->msp_keyframes.size(), mpRosMap->msp_map_points.size());
    
    mpMapHandler->InsertNewSubLocalMap(mpRosMap);
    
    //// Get maps and the map where KF is
    //ORB_SLAM3::Map* pCurrentMap = static_cast<ORB_SLAM3::Map*>(NULL);
    //std::map<unsigned long int, ORB_SLAM3::Map*> mMaps;
    //for(ORB_SLAM3::Map* pM : mpAtlas_->GetAllMaps())
    //{
    //    if(pM->GetId() == mpRosMap->mn_id)
    //    {
    //        pCurrentMap = pM;
    //    }
    //    mMaps[pM->GetId()] = pM;
    //}

    //if(!pCurrentMap)
    //{
    //    mpAtlas_->CreateNewMap();
    //    pCurrentMap = mpAtlas_->GetCurrentMap(); 
    //    mMaps[pCurrentMap->GetId()] = pCurrentMap;
    //}

    //// Create map data structures for postloads
    //std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    //std::map<std::string, ORB_SLAM3::MapPoint*> mMapMPs;

    //std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mFusedKFs; 
    //std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mMapKFs; 
    //
    //std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    //
    //for(ORB_SLAM3::KeyFrame* pKF : pCurrentMap->GetAllKeyFrames())
    //{
    //  mFusedKFs[pKF->mnId] = pKF;
    //  mMapKFs[pKF->mnId] = pKF;
    //}

    //for(ORB_SLAM3::MapPoint* pMP : pCurrentMap->GetAllMapPoints())
    //{
    //  mFusedMPs[pMP->mstrHexId] = pMP;
    //  mMapMPs[pMP->mstrHexId] = pMP;
    //}

    //for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas_->GetAllCameras())
    //{
    //  mCameras[pCam->GetId()] = pCam;
    //}
    //
    //RCLCPP_INFO(this->get_logger(), "Got an update for a map. 1. Existing data collected.");
    //
    //std::vector<ORB_SLAM3::KeyFrame*> mvpTempKFs;
    //mvpTempKFs.resize(mpRosMap->msp_keyframes.size());
    //std::map<std::string, ORB_SLAM3::MapPoint*> mTempMPs;
    //
    //// Sort the keyframes from smallest id to largest
    //// So that postloads should be done in order
    //std::vector<orbslam3_interfaces::msg::KeyFrame> mvRosKF = mpRosMap->msp_keyframes;
    //std::sort(mvRosKF.begin(), mvRosKF.end(), [](const orbslam3_interfaces::msg::KeyFrame& a, const orbslam3_interfaces::msg::KeyFrame& b) {
    //    return a.mn_id < b.mn_id;
    //});

    //// Loop through each KF
    //// Convert to ORB
    //// Add to mFusedKFs if not found -> if its not there then its a new object and can be used as it is for other objects
    //// if its there, then use original pointer in postload and just update that object afterwards
    //for(size_t i = 0; i<mvRosKF.size(); ++i)
    //{
    //    orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrame>(mvRosKF[i]);
    //    ORB_SLAM3::KeyFrame* tempKF = ProcessNewKeyFrame(mpRosKF, mMaps);
    //    mvpTempKFs[i] = tempKF;

    //    if(mFusedKFs.find(tempKF->mnId) == mFusedKFs.end())
    //        mFusedKFs[tempKF->mnId] = tempKF;
    //    
    //    // Do same stuff for each MP
    //    // Skip MPs which were already found in this map update (same objects in original map)
    //    for(size_t j = 0; j<mpRosKF->mvp_map_points.size(); ++j)
    //    {
    //        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[j]);
    //        // This MP was already seen in the map, skip since both points to the same pointer in the original
    //        if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
    //            continue;
    //        
    //        ORB_SLAM3::MapPoint* tempMP = ProcessNewMapPoint(mpRosMP, mMaps);
    //        mTempMPs[tempMP->mstrHexId] = tempMP;

    //        if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
    //            mFusedMPs[tempMP->mstrHexId] = tempMP;
    //    }
    //}
    //
    //RCLCPP_INFO(this->get_logger(), "Got an update for a map. 2. Data fusion complete.");

    //// There might be MPs which were not associated with KFs
    //// Process those as well
    //for(size_t i = 0; i<mpRosMap->msp_map_points.size(); ++i)
    //{
    //    orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosMap->msp_map_points[i]);

    //    if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
    //        continue;

    //    ORB_SLAM3::MapPoint* tempMP = ProcessNewMapPoint(mpRosMP, mMaps);
    //    mTempMPs[tempMP->mstrHexId] = tempMP;

    //    if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
    //        mFusedMPs[tempMP->mstrHexId] = tempMP;
    //}

    //RCLCPP_INFO(this->get_logger(), "Got an update for a map. 3. MPs processed.");
    //
    //// PostLoad for all the KFs
    //bool bKFUnprocessed = false;
    //for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    //{
    //  ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
    //  tempKF->PostLoad(mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    //}

    //// PostLoad for all the MPs
    //for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    //{
    //  ORB_SLAM3::MapPoint* tempMP = it->second;
    //  tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    //}
    //
    //RCLCPP_INFO(this->get_logger(), "Got an update for a map. 4. PostLoads completed.");

    //// Inject KFs
    //for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    //{
    //  ORB_SLAM3::MapPoint* tempMP = it->second;
    //  InjectMapPoint(tempMP, mMapMPs);
    //}

    //for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    //{
    //  ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
    //  ORB_SLAM3::KeyFrame* pKF = InjectKeyFrame(tempKF, mMapKFs);

    //  if(pKF)
    //      ForwardKeyFrameToTarget(pKF, 0, mpRosMap->from_module_id);
    //}
    //
    //RCLCPP_INFO(this->get_logger(), "Got an update for a map. 5. All the data is injected into ORB_SLAM3.");

    //// Remove KFs and MPs
    ////for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
    ////{
    ////    unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
    ////    ORB_SLAM3::KeyFrame* pEraseKF=mFusedKFs[mnId];
    ////    if(pEraseKF)
    ////        pEraseKF->SetBadFlag();
    ////}

    ////for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
    ////{
    ////  std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
    ////    ORB_SLAM3::MapPoint* pEraseMP=mFusedMPs[mnId];
    ////    if(pEraseMP)
    ////        pEraseMP->SetBadFlag();
    ////}

    //RCLCPP_INFO(this->get_logger(), "Got an update for a map. 6. Data which was deleted elsewhere is removed from the system.");

    //// Finally update Map object 
    //// First create temporary Map from ROS data
    //// Then Inject the data into real map (do not update KFs and MPs since those are already added)
    //ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    //tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);
    //tempMap->PostLoad(mpKeyFrameDB, m_SLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    //InjectMap(tempMap, pCurrentMap);
    //RCLCPP_INFO(this->get_logger(), "Got an update for a map. 7. Map is injected into ORB_SLAM3 and Map update is COMPLETE.");
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

void SlamWrapperNode::workerCallback(std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received msg from /worker, new worker added to the network=%d.", msg->data);
    mpObserver->AddNewWorker(msg->data); 
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
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data));
    
    /* MAP */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
    map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
        "/Map", 
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data));
    
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

    /* Worker */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /worker");
    worker_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "/worker", 
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()) ;

    /* Step */
    //RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /step");
    //step_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    //    "/step", 
    //    10);


}







/*   INIT SUBSCRIBERS   */

void SlamWrapperNode::CreateSubscribers() {
    //rclcpp::SubscriptionOptions options1;
    //rclcpp::SubscriptionOptions options2;
    //rclcpp::SubscriptionOptions options3;
    //options1.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    //options2.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    //options3.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    

    /* KF */  
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
    m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
        "KeyFrame",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));//, options1);
    
    /* Map */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
    m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
        "Map",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));//, options2);

    /* Atlas */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Atlas");
    m_atlas_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Atlas>(
        "Atlas",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabAtlas, this, std::placeholders::_1));//, options3);
    
    /* MapPoint */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /MapPoint");
    m_map_point_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::MapPoint>(
        "MapPoint",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabMapPoint, this, std::placeholders::_1));//, options3);
    

    /* LocalMapping active */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Active");
    m_lm_active_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Bool>(
        "LocalMapping/Active",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabLMActive, this, std::placeholders::_1));//, options3);

    /* LocalMapping reset requested */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Reset");
    m_lm_reset_requested_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "LocalMapping/Reset",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabLMResetRequested, this, std::placeholders::_1));//, options3);

    /* LocalMapping reset requested */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map/Reset/Active");
    m_sys_reset_active_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Int64>(
        "Map/Reset/Active",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabResetActiveMap, this, std::placeholders::_1));//, options3);
    

    /* Destroy node */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /destroy");
    end_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/destroy", 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE), 
        std::bind(&SlamWrapperNode::endCallback, this, std::placeholders::_1));//, options3);

    /* Worker node */
    rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
    qosLatching.transient_local();
    qosLatching.reliable();
    
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /worker");
    worker_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/worker", 
        qosLatching, 
        std::bind(&SlamWrapperNode::workerCallback, this, std::placeholders::_1));//, options3);
    
    /* Step */
    //RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /step");
    //step_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    //    "/step", 
    //    10, 
    //    std::bind(&SlamWrapperNode::stepCallback, this, std::placeholders::_1));
}
