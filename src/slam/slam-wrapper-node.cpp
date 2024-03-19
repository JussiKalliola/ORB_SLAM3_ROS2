#include "slam-wrapper-node.hpp"
#include <cstdlib>
#include <stdlib.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


SlamWrapperNode::SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam, const std::string path, const std::string strResultFilename ) : Node("SlamWrapperNode") {
    
    char* systemId = std::getenv("SLAM_SYSTEM_ID");
    
    RCLCPP_INFO(this->get_logger(), "Initializing SLAM Wrapper Node. System ID=%s", systemId);
    
    if(string(systemId) == "main") {
        RCLCPP_INFO(this->get_logger(), " °Main responsibility: Tracking");
        mnTaskModule = 1;
    } else if (string(systemId) == "sub") {
        RCLCPP_INFO(this->get_logger(), " °Main responsibility: Local Mapping");
        mnTaskModule = 2;
    } else {
        RCLCPP_INFO(this->get_logger(), " °Main responsibility: Idle");
        mnTaskModule = 0;
    }

    savePath=path;
    mstrResultFilename=strResultFilename;

    m_SLAM = pSLAM;
    mpTracker_ = m_SLAM->GetTrackerPtr();
    mpLocalMapper_ = m_SLAM->GetMapperPtr();
    mpAtlas_ = m_SLAM->GetAtlas();
    mpKeyFrameDB = m_SLAM->GetKeyFrameDatabase();
    
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
    if(!pKF || pKF->isBad())
        return;
    
    mORBKeyFrames.erase(pKF->mnId); 
}

void SlamWrapperNode::AddMapPoint(ORB_SLAM3::MapPoint* pMP)
{
    unique_lock<std::mutex> lock(mMutexMapPoint); 
    if(!pMP || pMP->isBad())
        return;

    mORBMapPoints[pMP->mstrHexId] = pMP; 
}
void SlamWrapperNode::EraseMapPoint(ORB_SLAM3::MapPoint* pMP)
{
    unique_lock<std::mutex> lock(mMutexMapPoint); 
    if(!pMP || pMP->isBad())
        return;
    
    mORBMapPoints.erase(pMP->mstrHexId); 
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
        unsigned long int largestId = mORBMapPoints.begin()->second->mnId; // Start with the first element as the largest
        for (auto it = mORBMapPoints.begin(); it != mORBMapPoints.end(); ++it) {
            if (it->second->mnId > largestId) {
                largestId = it->second->mnId; // Found a larger ID
            }
        }
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
        unsigned long int largestId = mORBKeyFrames.begin()->second->mnId; // Start with the first element as the largest
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
                mpKeyFrameDB->add(pKF);
                mpLocalMapper_->InsertKeyframeFromRos(pKF);
            } else {
                pKF->SetBadFlag();
            }  
        }    
    } else if (mnTaskModule == 3) {
        // in this case the system mainly performing loop closing
        // not sure what to do here yet. 
    }
}

/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTragetModule) {
    
    //std::lock_guard<std::mutex> lock(mMutexUpdateMap);

    // Calculate the size of the object using sizeof
    RCLCPP_INFO(this->get_logger(), "Publishing a new KeyFrame with id: %d. Stats: #MPs=%d, Map ID=%d, #MP matches=%d, #mspChildrens=%d, #msploopedges=%d, #mspMergeEdges=%d", pKF->mnId, pKF->GetMapPoints().size(), pKF->GetMap()->GetId(), pKF->GetMapPointMatches().size(), pKF->GetChilds().size(), pKF->GetLoopEdges().size(), pKF->GetMergeEdges().size());

      
    //while(MapActionActive())
    //{
    //    std::cout << "Map action is active, waiting..." << std::endl;
    //    usleep(3000);
    //}
    
    SetKeyFrameAction(true);

    for(const auto& cam : mpAtlas_->GetAllCameras())
    {
        if(cam && cam != nullptr)
            mspCameras.insert(cam);
    }
    

    
    std::vector<ORB_SLAM3::KeyFrame*> mvpKeyFrames = pKF->GetMap()->GetAllKeyFrames();
    std::vector<ORB_SLAM3::MapPoint*> mvpMapPoints = pKF->GetMap()->GetAllMapPoints();
    
    std::set<ORB_SLAM3::KeyFrame*> mspKeyFrames(mvpKeyFrames.begin(), mvpKeyFrames.end());
    std::set<ORB_SLAM3::MapPoint*> mspMapPoints(mvpMapPoints.begin(), mvpMapPoints.end());
    

    pKF->PreSave(mspKeyFrames, mspMapPoints, mspCameras);
   
    AddKeyFrame(pKF);
    AddMap(pKF->GetMap());
    
    for(ORB_SLAM3::MapPoint* pMP : pKF->GetMapPoints())
    {
        pMP->PreSave(mspKeyFrames, mspMapPoints);
        AddMapPoint(pMP);
    }

    orbslam3_interfaces::msg::KeyFrame mRosKF = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKF);
    mRosKF.target = mnTragetModule;
    keyframe_publisher_->publish(mRosKF);

    //ORB_SLAM3::KeyFrame* tempKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(std::make_shared<orbslam3_interfaces::msg::KeyFrame>(msgKf), static_cast<ORB_SLAM3::KeyFrame*>(NULL));
    //Utility::CompareTwoSerializedObjects(tempKF, pKf);
    
    
    SetKeyFrameAction(false);
}

/*        Map        */
void SlamWrapperNode::publishMap(ORB_SLAM3::Map* pM) {
     
    //while(KeyFrameActionActive())
    //{
    //    std::cout << "KF action is active, waiting..." << std::endl;
    //    usleep(3000);
    //}
    
    mpLocalMapper_->SetAcceptKeyFrames(false);
    mpLocalMapper_->AllowLocalMapping(false);
    
    SetMapAction(true);
    
    RCLCPP_INFO(this->get_logger(), "Publishing a new map (full object) with id: %d", pM->GetId());
    RCLCPP_INFO(this->get_logger(), "Map Stats : #KFs=%d, #MPs=%d, #RefMPs=%d", pM->KeyFramesInMap(), pM->MapPointsInMap(), pM->GetReferenceMapPoints().size());

    
    pM->PreSave(mspCameras);
    
    auto mRosMap = Converter::MapConverter::OrbMapToRosMap(pM);

    map_publisher_->publish(mRosMap);
    RCLCPP_INFO(this->get_logger(), "Map published with id: %d, delete MPs=%d, update MPs=%d, delete KFs=%d, update KFs=%d", mRosMap.mn_id, pM->GetErasedMPIds().size(), pM->GetUpdatedMPIds().size(), pM->GetErasedKFIds().size(), pM->GetUpdatedKFIds().size());
    
    pM->ClearErasedData();
    pM->ClearUpdatedKFIds();
    pM->ClearUpdatedMPIds();
    
    {
        //std::unique_lock<std::mutex> lock(mMutexUpdateMap);
        for(ORB_SLAM3::KeyFrame* pKF : pM->GetAllKeyFrames())
        {
            AddKeyFrame(pKF);
        }

        for(ORB_SLAM3::MapPoint* pMP : pM->GetAllMapPoints())
        {
            AddMapPoint(pMP);
        }

    } 
    
    mpLocalMapper_->SetAcceptKeyFrames(true);
    mpLocalMapper_->AllowLocalMapping(true);
    
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
    
    CalcMaxMapPointID();
    CalcMaxKeyFrameID();

    //while(MapActionActive())
    //{
    //    std::cout << "Map action is active, waiting..." << std::endl;
    //    usleep(3000);
    //}
    
    SetKeyFrameAction(true);


    RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d, for a map: %d", mpRosKF->mn_id, mpRosKF->mp_map_id);

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
        if(mpRosKF->mn_last_module >= mORBKeyFrames[mpRosKF->mn_id]->GetLastModule())
        {
            pKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, mORBKeyFrames[mpRosKF->mn_id]);
        }
    }

    if(!pKF)
    {
        RCLCPP_INFO(this->get_logger(), "Dropping the KF=%d since we have updated version of it.", mpRosKF->mn_id);
        SetKeyFrameAction(false);  
        return;
    }
    
    unsigned long int mnLargestMPID = GetMaxMapPointID();
    
    for(size_t i = 0; i < mpRosKF->mvp_map_points.size(); i++)
    {
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[i]);
        
        ORB_SLAM3::MapPoint* pMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
        if(mORBMapPoints.find(mpRosMP->m_str_hex_id) != mORBMapPoints.end())
        {
            // Check where the object was last updated
            // If in lower priority module, discard
            if(mpRosMP->mn_last_module >= mORBMapPoints[mpRosMP->m_str_hex_id]->GetLastModule())
            {
                pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, mORBMapPoints[mpRosMP->m_str_hex_id]);
            }
        } else {
            //Check if received ID have smaller integer id than the biggest ID in the system
            // If so, increment that by one and set as the id
            if(mpRosMP->mn_id < mnLargestMPID) 
            {
                ++mnLargestMPID;
                mpRosMP->mn_id = mnLargestMPID;
            }
            pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, pMP);
            AddMapPoint(pMP);
        }
        if(pMP)
            pMP->UpdateMap(mORBMaps[mpRosMP->mp_map_id]);
    }
    
    bool bKFUnprocessed = false;
    for(const auto& mstrId : mpRosKF->mv_backup_map_points_id)
    {
        if(mstrId == "" || mstrId.length() < 6) continue;
        mORBMapPoints[mstrId]->PostLoad(mORBKeyFrames, mORBMapPoints, &bKFUnprocessed, mspUnprocKFids);

        if(bKFUnprocessed) {
            std::cout << "Map Point if unprocessed, continuing..." << std::endl;

            startTimer();
            mpUnprocOrbKeyFrames[pKF->mnId] = mpRosKF;
            mspUnprocKFids.insert(pKF->mnId);   
          
            SetKeyFrameAction(false);  
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
}


/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr mpRosMap) {
    if(mpRosMap->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    

    //while(KeyFrameActionActive())
    //{
    //    std::cout << "KF action is active, waiting..." << std::endl;
    //    usleep(3000);
    //}
    
    SetMapAction(true);

    std::chrono::system_clock::time_point mcMapReceiveTime = std::chrono::system_clock::now();
    if(mpTracker_->mcLastResetTimeStamp < mcMapReceiveTime)
    {
        if(mpRosMap->msp_keyframes.size() > mpAtlas_->GetCurrentMap()->KeyFramesInMap())
        {
            RCLCPP_INFO(this->get_logger(), "Map was resetted before receiving update, do not accept it. receive time=%d, last reset time=%d", mcMapReceiveTime, mpTracker_->mcLastResetTimeStamp);

            SetMapAction(false);
            return;
        }
    }

    if(mORBMaps.find(mpRosMap->mn_id) != mORBMaps.end())
    {
        RCLCPP_INFO(this->get_logger(), "Got an update for a map, id=%d. Stats: #KFs=%d, #MPs=%d", mpRosMap->mn_id, mpRosMap->msp_keyframes.size(), mpRosMap->msp_map_points.size());
    } else { 
        RCLCPP_INFO(this->get_logger(), "Got a new map, id=%d. Stats: #KFs=%d, #MPs=%d", mpRosMap->mn_id, mpRosMap->msp_keyframes.size(), mpRosMap->msp_map_points.size());
    }

    {
        //unique_lock<mutex> lock(mMutexUpdateMap);
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
    }

    CalcMaxMapPointID();
    CalcMaxKeyFrameID();

    //bool bUnprocessed = false;
    ORB_SLAM3::Map* pM = static_cast<ORB_SLAM3::Map*>(NULL);
    if(mORBMaps.find(mpRosMap->mn_id) != mORBMaps.end())
    {
        //unique_lock<mutex> lock(mMutexUpdateMap);
        pM = mORBMaps[mpRosMap->mn_id];
        pM = Converter::MapConverter::RosMapToOrbMap(mpRosMap, pM);
    }
    std::vector<std::string> mvpMPsDone;
    std::cout << "#KFs=" << mpRosMap->mvp_backup_keyframes_ids.size() << ", #updates=" << mpRosMap->msp_keyframes.size() << ", erased=" << mpRosMap->mvp_erased_keyframe_ids.size() << std::endl;
    std::cout << "#MPs=" << mpRosMap->mvp_backup_map_points_ids.size() << ", #updates=" << mpRosMap->msp_map_points.size() << ", erased=" << mpRosMap->mvp_erased_mappoint_ids.size() << std::endl;
    

    size_t kfFound=0;
    size_t kfNotFound=0;
    size_t mpFound=0;
    size_t mpNotFound=0;
    


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
                //std::cout << " - MP=" << mp->m_str_hex_id << " not found, processing..."<< std::endl;
                pMP = Converter::MapPointConverter::RosMapPointToOrb(mp, static_cast<ORB_SLAM3::MapPoint*>(NULL));
                AddMapPoint(pMP);
                mpNotFound+=1;
            } else {
                //std::cout << " - MP=" << mp->m_str_hex_id << " found, processing..."<< std::endl;
                pMP = Converter::MapPointConverter::RosMapPointToOrb(mp, mORBMapPoints[mp->m_str_hex_id]);
                mpNotFound+=1;
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
    
    // Remove KFs which have been removed on the other machine
    for(const auto& kfId : mpRosMap->mvp_erased_keyframe_ids)
    {
        if(mORBKeyFrames.find(kfId) != mORBKeyFrames.end())
        {
            mORBKeyFrames[kfId]->SetBadFlag();
            //EraseKeyFrame(mORBKeyFrames[kfId]);
        }
    }
   
    // Remove MapPoints which have been removed on the other machine
    for(const auto& mpId : mpRosMap->mvp_erased_mappoint_ids)
    {
        if(mORBMapPoints.find(mpId) != mORBMapPoints.end())
        {
            if(mORBMapPoints[mpId])
            {
                mORBMapPoints[mpId]->SetBadFlag();
                //EraseMapPoint(mORBMapPoints[mpId]);
            }
        }
    }
    
    mpTracker_->LocalMapIsUpdating(false); 
    
    if(bKFUnprocessed) 
        std::cout << "Map with ID=" << pM->GetId() << " is unprocessed!!" << std::endl;
    
    if(mnTaskModule == 2) {
        mpTracker_->UpdateFromLocalMapping(mORBMaps[mpRosMap->mn_id], mORBKeyFrames, mORBMapPoints);
    }

    
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

    mpTracker_->ResetActiveMap(false, msg->data);
    
    mORBMaps.clear();
    mORBKeyFrames.clear();
    mORBMapPoints.clear();
  
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
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_DURABILITY_VOLATILE));
    
    /* MAP */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
    map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
        "/Map", 
        rclcpp::QoS(rclcpp::KeepLast(5)).reliability(RMW_QOS_POLICY_DURABILITY_VOLATILE));
    
    /* MAPPOINT */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /MapPoint");
    map_point_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapPoint>(
        "/MapPoint", 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    
    /* LocalMapping Active*/
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /LocalMapping/Active");
    lm_active_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Bool>(
        "/LocalMapping/Active", 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));

    /* LocalMapping Active*/
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /LocalMapping/Reset");
    lm_reset_requested_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/LocalMapping/Reset", 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));


    /* Reset Active Map (System) */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map/Active/Reset");
    sys_reset_active_map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Int64>(
        "/Map/Reset/Active", 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));

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
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));
    
    /* Map */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
    m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
        "Map",
        rclcpp::QoS(rclcpp::KeepLast(5)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));

    /* MapPoint */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /MapPoint");
    m_map_point_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::MapPoint>(
        "MapPoint",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        std::bind(&SlamWrapperNode::GrabMapPoint, this, std::placeholders::_1));
    

    /* LocalMapping active */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Active");
    m_lm_active_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Bool>(
        "LocalMapping/Active",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
        std::bind(&SlamWrapperNode::GrabLMActive, this, std::placeholders::_1));

    /* LocalMapping reset requested */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Reset");
    m_lm_reset_requested_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "LocalMapping/Reset",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
        std::bind(&SlamWrapperNode::GrabLMResetRequested, this, std::placeholders::_1));

    /* LocalMapping reset requested */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map/Reset/Active");
    m_sys_reset_active_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Int64>(
        "Map/Reset/Active",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
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
