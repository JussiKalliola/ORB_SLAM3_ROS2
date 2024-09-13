#include "slam-wrapper-node.hpp"
//#include "./Distributor.hpp"

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
    

    savePath=path;
    mstrResultFilename=strResultFilename;

    // Attach Distribution pointers
    mpDistributionSystem=pDistSystem;
    mpObserver=pDistSystem->GetObserver();
    mpMapHandler=pDistSystem->GetMapHandler();
    mpKeyFramePublisher=pDistSystem->GetKeyFramePublisher();
    mpKeyFrameSubscriber=pDistSystem->GetKeyFrameSubscriber();

    // Use ROS time 
    std::chrono::system_clock::time_point time_Start(std::chrono::nanoseconds(this->now().nanoseconds()));
    mpDistributionSystem->UpdateGlobalStartTime(time_Start);

    // Attach ORB SLAM3 System pointers
    m_SLAM = pSLAM;
    mpTracker_ = m_SLAM->GetTrackerPtr();
    mpLocalMapper_ = m_SLAM->GetMapperPtr();
    mpLoopCloser_ = m_SLAM->GetLoopClosingPtr();
    mpAtlas_ = m_SLAM->GetAtlas();
    mpKeyFrameDB = m_SLAM->GetKeyFrameDatabase();
   

    mpLocalMapper_->AllowLocalMapping(true);

    //mpTracker_->SetStepByStep(true);

    CreatePublishers();

    /* Init subscribers */
    if (subscribe_to_slam) { 
        CreateSubscribers();
    }
    
    int mnTaskModule = 0;
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

    std_msgs::msg::Int32 wMsg;
    wMsg.data = mnTaskModule;
    
    worker_publisher_->publish(wMsg);
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

/* Publish functions */

/* OBJECT PUBLISHERS */

/*        KeyFrame        */
void SlamWrapperNode::publishKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr mRosKF, unsigned int mnTarget) 
{
    // Add header
    std_msgs::msg::Header header;
    header.stamp = this->now();
    mRosKF->header = header;

    if(mnTarget == 3)
        keyframe_lc_publisher_->publish(*mRosKF);
    else if (mnTarget == 2)
        keyframe_lm_publisher_->publish(*mRosKF);
    
    RCLCPP_INFO(this->get_logger(), "Publishing a new KeyFrame with id: %d. Stats: #MPs=%d, Map ID=%d", mRosKF->mn_id, mRosKF->mvp_map_points.size(), mRosKF->mp_map_id);
}

void SlamWrapperNode::publishKeyFrame(orbslam3_interfaces::msg::KeyFrameUpdate mRosKFUpdate)
{
    // Add header
    std_msgs::msg::Header header;
    header.stamp = this->now();
    mRosKFUpdate.header = header;

    keyframe_update_publisher_->publish(mRosKFUpdate);
    
    RCLCPP_INFO(this->get_logger(), "Publishing KeyFrame Uådate with id: %d. Stats: #MPs=%d, Map ID=%d", mRosKFUpdate.mn_id, mRosKFUpdate.mvp_map_points.size(), mRosKFUpdate.mp_map_id);
}


/*        Atlas        */
void SlamWrapperNode::publishAtlas(const orbslam3_interfaces::msg::Atlas::SharedPtr& mRosAtlas) 
{

    // Add header
    std_msgs::msg::Header header;
    header.stamp = this->now();
    mRosAtlas->header = header;
    
    if(mRosAtlas->mb_map_merge)
        atlas_merge_publisher_->publish(*mRosAtlas);
    else if(mRosAtlas->mb_loop_closer)
        atlas_lc_publisher_->publish(*mRosAtlas);

    RCLCPP_INFO(this->get_logger(), " ********************* Publishing Atlas after Merge=%d or LoopClosure=%d, #KFs=%d", mRosAtlas->mb_map_merge, mRosAtlas->mb_loop_closer, mRosAtlas->mp_current_map.msp_keyframes.size());
    RCLCPP_INFO(this->get_logger(), "Map Stats : #updated KFs=%d, #updated MPs=%d, #del KFs=%d, #del MPs=%d", 
        mRosAtlas->mp_current_map.mvp_updated_keyframes_ids.size(), 
        mRosAtlas->mp_current_map.mvp_updated_map_points_ids.size(), 
        mRosAtlas->mp_current_map.mvp_erased_keyframe_ids.size(), 
        mRosAtlas->mp_current_map.mvp_erased_mappoint_ids.size());
}

/*        Map        */
void SlamWrapperNode::publishMap(const orbslam3_interfaces::msg::Map::SharedPtr mRosMap) 
{

    // Add header
    std_msgs::msg::Header header;
    header.stamp = this->now();
    mRosMap->header = header;
    
    map_publisher_->publish(*mRosMap);
    
    RCLCPP_INFO(this->get_logger(), "Publishing a new map with id: %d", mRosMap->mn_id);
    RCLCPP_INFO(this->get_logger(), "Map Stats : #updated KFs=%d, #updated MPs=%d, #del KFs=%d, #del MPs=%d", 
        mRosMap->mvp_updated_keyframes_ids.size(), 
        mRosMap->mvp_updated_map_points_ids.size(), 
        mRosMap->mvp_erased_keyframe_ids.size(), 
        mRosMap->mvp_erased_mappoint_ids.size());
}



/*        MapPoint        */
void SlamWrapperNode::publishMapPoint(ORB_SLAM3::MapPoint* pMP) {
    //RCLCPP_INFO(this->get_logger(), "Publishing a new mappoint with id: %d", pMp->mnId);
    //orbslam3_interfaces::msg::MapPoint::SharedPtr mRosMP = Converter::MapPointConverter::ORBSLAM3MapPointToROS(pMP);
    //mRosMP->system_id = std::getenv("SLAM_SYSTEM_ID");
    //
    //map_point_publisher_->publish(*mRosMP);
}

/*        Local Mapping - Active        */
//void SlamWrapperNode::publishLMActivityChange(bool bActive) {
//    RCLCPP_INFO(this->get_logger(), "Publishing Local Mapping Activity change=%d", bActive);
//    orbslam3_interfaces::msg::Bool msg = orbslam3_interfaces::msg::Bool();
//    
//    msg.system_id = std::getenv("SLAM_SYSTEM_ID");
//    msg.data = bActive;
//    
//
//    lm_active_publisher_->publish(msg);
//}

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

    std_msgs::msg::Header header;
    header.stamp = this->now();
    msg.header = header;

    mpObserver->UpdateLastResetTime();
    mpMapHandler->ResetQueue();
    mpKeyFrameSubscriber->ResetQueue(true);
    mpKeyFramePublisher->ResetQueue();
    
    sys_reset_active_map_publisher_->publish(msg);
}



/* OBJECT SUBSCRIPTION CALLBACKS */

/*        KeyFrame        */
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF) {
  
    if(mpRosKF->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;

    rclcpp::Time msgTime = mpRosKF->header.stamp;
    std::chrono::system_clock::time_point time_Start(std::chrono::nanoseconds(msgTime.nanoseconds()));
    std::chrono::system_clock::time_point time_Now(std::chrono::nanoseconds(this->now().nanoseconds()));
    double timeLatency = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_Now - time_Start).count();
    vdLatencyKF_ms.push_back(timeLatency);

    RCLCPP_INFO(this->get_logger(), "Got a new keyframe, id: %d, for a map: %d, latency=%f", mpRosKF->mn_id, mpRosKF->mp_map_id, timeLatency);

    //auto timeDiff = rclcpp::Clock(RCL_SYSTEM_TIME).now() - mpRosKF->header.stamp;
    
    //RCLCPP_FATAL(this->get_logger(), " *** Latency=%d", mpRosKF->header.stamp.to_chrono<std::chrono::milliseconds>());
    mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKF);

    if(mpObserver->GetTaskModule()==2)
       publishKeyFrame(mpRosKF, 3); 

}


void SlamWrapperNode::GrabKeyFrameUpdate(orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKFUpdate) {
  
    if(mpRosKFUpdate->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;

    rclcpp::Time msgTime = mpRosKFUpdate->header.stamp;
    std::chrono::system_clock::time_point time_Start(std::chrono::nanoseconds(msgTime.nanoseconds()));
    std::chrono::system_clock::time_point time_Now(std::chrono::nanoseconds(this->now().nanoseconds()));
    double timeLatency = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_Now - time_Start).count();
    vdLatencyKF_ms.push_back(timeLatency);

    //RCLCPP_INFO(this->get_logger(), " **************************** Got a new keyframe, id: %d, for a map: %d", mpRosKF->mn_id, mpRosKF->mp_map_id);

    //auto timeDiff = rclcpp::Clock(RCL_SYSTEM_TIME).now() - mpRosKF->header.stamp;
    
    //RCLCPP_FATAL(this->get_logger(), " *** Latency=%d", mpRosKF->header.stamp.to_chrono<std::chrono::milliseconds>());
    mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKFUpdate);

}


/*        Atlas        */
void SlamWrapperNode::GrabAtlas(const orbslam3_interfaces::msg::Atlas::SharedPtr mpRosAtlas) {
    if(mpRosAtlas->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;

    rclcpp::Time msgTime = mpRosAtlas->header.stamp;
    std::chrono::system_clock::time_point time_Start(std::chrono::nanoseconds(msgTime.nanoseconds()));
    std::chrono::system_clock::time_point time_Now(std::chrono::nanoseconds(this->now().nanoseconds()));
    double timeLatency = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_Now - time_Start).count();
    vdLatencyAtlas_ms.push_back(timeLatency);
    
    RCLCPP_INFO(this->get_logger(), "Got Atlas Update, merge=%d, closure=%d, latency=%f", mpRosAtlas->mb_map_merge, mpRosAtlas->mb_loop_closer, timeLatency);
    
    mpMapHandler->InsertNewSubGlobalMap(mpRosAtlas);

}




/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr mpRosMap) {
    if(mpRosMap->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    rclcpp::Time msgTime = mpRosMap->header.stamp;
    std::chrono::system_clock::time_point time_Start(std::chrono::nanoseconds(msgTime.nanoseconds()));
    std::chrono::system_clock::time_point time_Now(std::chrono::nanoseconds(this->now().nanoseconds()));
    double timeLatency = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_Now - time_Start).count();
    
    setprecision(5);
    RCLCPP_INFO(this->get_logger(), "Got an update for a map, id=%d, from module=%d. Stats: #KFs=%d, #MPs=%d, latency=%f", mpRosMap->mn_id, mpRosMap->from_module_id, mpRosMap->msp_keyframes.size(), mpRosMap->msp_map_points.size(), timeLatency);

    vdLatencyMap_ms.push_back(timeLatency);

    
    mpMapHandler->InsertNewSubLocalMap(mpRosMap);
    
}

/*        MapPoint        */
void SlamWrapperNode::GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP) {  
    if(mpRosMP->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    RCLCPP_INFO(this->get_logger(), "Got a new mappoint, id: %d", mpRosMP->mn_id);
    //bool bUnprocessed = false;
    //
    //ORB_SLAM3::MapPoint* pMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
    //if(mORBMapPoints.find(mpRosMP->m_str_hex_id) == mORBMapPoints.end())
    //    pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, pMP);
    //else
    //    pMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, mORBMapPoints[mpRosMP->m_str_hex_id]);

    //if(bUnprocessed) {
    //    mpUnprocOrbMapPoints[pMP->mstrHexId] = std::make_tuple(pMP, mpRosMP);
    //} 
    //
    //AddMapPoint(pMP);
}


/*        System Reset Active Map*/
void SlamWrapperNode::GrabResetActiveMap(const orbslam3_interfaces::msg::Int64::SharedPtr msg) {
    
    if(msg->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    mpObserver->UpdateLastResetTime();

    mpKeyFramePublisher->ResetQueue();
    mpKeyFrameSubscriber->ResetQueue(true);
    mpMapHandler->ResetQueue();
    mpLoopCloser_->StopGBA();

    
    RCLCPP_INFO(this->get_logger(), "Reset requested for Active map id=%d", msg->data);

    mpLocalMapper_->Release();

    //mpObserver->ClearMapPointsFromMap(mpAtlas_->GetCurrentMap());
    //mpObserver->ClearKeyFramesFromMap(mpAtlas_->GetCurrentMap());

    if(mpObserver->GetTaskModule() == 2)
        mpTracker_->ResetActiveMap(false, msg->data);
    else
        mpTracker_->ResetActiveMap(true, msg->data);
}


/*        LocalMapping reset        */
void SlamWrapperNode::GrabLMResetRequested(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Reset requested for Local mapping=%d", msg->data);

    mpLocalMapper_->RequestReset();
    

    //mspKFsReadyForLM.clear();
    //mpUnprocOrbKeyFrames.clear();
    //mORBMaps.clear();
    //mORBKeyFrames.clear();
    //mORBMapPoints.clear();

    mpKeyFrameDB->clear();
    mpAtlas_->clearAtlas();
    mpAtlas_->CreateNewMap();

}

/*        LocalMapping - Active        */
//void SlamWrapperNode::GrabLMActive(const orbslam3_interfaces::msg::Bool::SharedPtr msg) {
//    if(msg->system_id == std::getenv("SLAM_SYSTEM_ID")) 
//        return;
//    
//    RCLCPP_INFO(this->get_logger(), "Local mapping is active=%d", msg->data);
//}

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

void SlamWrapperNode::publishStopLM(const bool bStopLM) 
{
    if(mpObserver->GetTaskModule() < 3)
        return;

    orbslam3_interfaces::msg::Bool bMsg;
    bMsg.system_id = std::getenv("SLAM_SYSTEM_ID");
    bMsg.data = bStopLM;

    
    stop_lm_publisher_->publish(bMsg);
    RCLCPP_INFO(this->get_logger(), "Publishing to /LocalMapping/Stop");
}


void SlamWrapperNode::stopLMCallback(orbslam3_interfaces::msg::Bool::SharedPtr bMsg) {
    
    if(bMsg->system_id == std::getenv("SLAM_SYSTEM_ID"))
       return; 

    RCLCPP_INFO(this->get_logger(), "Received msg from /LocalMapping/Stop");
    if(bMsg->data)
    {
        //mpTracker_->mbStep = true;
        if(!mpLocalMapper_->mbGBARunning)
        {
            //for(const auto& mnId : mpMapHandler->msToBeErasedKFs)
            //{
            //    ORB_SLAM3::KeyFrame* pKFi = mpObserver->GetKeyFrame(mnId);
            //    if(pKFi)
            //    {
            //      pKFi->SetBadFlag();
            //      mpObserver->EraseKeyFrame(pKFi);
            //    }
            //}

            //for(const auto& mnId : mpMapHandler->msToBeErasedMPs)
            //{
            //    ORB_SLAM3::MapPoint* pMPi = mpObserver->GetMapPoint(mnId);
            //    if(pMPi)
            //    {
            //      pMPi->SetBadFlag();
            //      mpObserver->EraseMapPoint(pMPi);
            //    }
            //}

            //mpMapHandler->msToBeErasedKFs.clear();
            //mpMapHandler->msToBeErasedMPs.clear();
        }
        //mpLocalMapper_->mbGBARunning = true;
        mpLocalMapper_->RequestStop();
        //mpLocalMapper_->EmptyQueue(); // Proccess keyframes in the queue
        //mpLocalMapper_->RequestStop();
        //mpMapHandler->ResetQueue();
        //mpKeyFrameSubscriber->ResetQueue(false);
        //mpKeyFramePublisher->ResetQueue();
    } else {
      //mpLocalMapper_->mbGBARunning = false;
      //if(mpObserver->GetTaskModule() == 1)
      //    mpLocalMapper_->Release();
      //mpLocalMapper_->EmptyQueue();
    }
}


/*    INIT PUBLISHERS    */

void SlamWrapperNode::CreatePublishers() {

    int nTaskId = mpObserver->GetTaskModule();

    rclcpp::QoS qosMap = rclcpp::QoS(rclcpp::KeepLast(100));
    qosMap.reliable();
    //qosMap.durability(rclcpp::DurabilityPolicy(0)); // Volatile
    //qosMap.deadline(rclcpp::Duration(0, 400000000)); // 200ms
    qosMap.lifespan(rclcpp::Duration(0, 300000000)); // 100ms

    rclcpp::QoS qosKF = rclcpp::QoS(rclcpp::KeepLast(100));
    //qosKF.best_effort();
    if(nTaskId==1)
        qosKF.reliable();
    else if(nTaskId==2)
        qosKF.best_effort();
    //qosKF.durability(rclcpp::DurabilityPolicy(0)); // Volatile
    //qosKF.deadline(rclcpp::Duration(0, 200000000)); // 200ms
    qosKF.lifespan(rclcpp::Duration(0, 100000000)); // 50ms
                                                   

    rclcpp::QoS qosAtlas = rclcpp::QoS(rclcpp::KeepLast(25));
    qosAtlas.reliable();
    //qosAtlas.durability(rclcpp::DurabilityPolicy(0)); // Volatile
    //qosKF.deadline(rclcpp::Duration(0, 200000000)); // 200ms
    //qosAtlas.lifespan(rclcpp::Duration(0, 100000000)); // 80ms

    if(nTaskId==1)
    {
        /* KEYFRAME */
        RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame/LocalMapping");
        keyframe_lm_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
            "/KeyFrame/LocalMapping", 
            qosKF);
            //rclcpp::QoS(rclcpp::KeepLast(10),  rmw_qos_profile_default));//rmw_qos_profile_sensor_data));
    }

    if(nTaskId==2)
    {
        /* KEYFRAME */
        RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame/LoopClosing");
        keyframe_lc_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
            "/KeyFrame/LoopClosing", 
            qosKF);
            //rclcpp::QoS(rclcpp::KeepLast(10),  rmw_qos_profile_default));//rmw_qos_profile_sensor_data));
    }
    /* KEYFRAME */
    //RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame");
    //keyframe_update_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrameUpdate>(
    //    "/KeyFrame/Update", 
    //    rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default));//rmw_qos_profile_sensor_data));
    
    if(nTaskId==2)
    {
        /* MAP */
        RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
        map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
            "/Map", 
            qosMap);
            //rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default));
    }
    
    /* MAPPOINT */
    //RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /MapPoint");
    //map_point_publisher_ = this->create_publisher<orbslam3_interfaces::msg::MapPoint>(
    //    "/MapPoint", 
    //    rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default));
    
    if(nTaskId==3)
    {
        /* ATLAS */
        RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Atlas/LoopClosing");
        atlas_lc_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Atlas>(
            "/Atlas/LoopClosing", 
            qosAtlas);
            //rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default));
    }


    if(nTaskId==3)
    {
        /* ATLAS */
        RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Atlas/MapMerge");
        atlas_merge_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Atlas>(
            "/Atlas/MapMerge", 
            qosAtlas);
            //rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default));
    }
    
    /* LocalMapping Active*/
    //RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /LocalMapping/Active");
    //lm_active_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Bool>(
    //    "/LocalMapping/Active", 
    //    rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default));

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
        rclcpp::QoS(rclcpp::KeepLast(5)).transient_local().reliable()) ;

    /* Stop LM */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /LocalMapping/Stop");
    stop_lm_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Bool>(
        "/LocalMapping/Stop", 
        10);


}







/*   INIT SUBSCRIBERS   */

void SlamWrapperNode::CreateSubscribers() {

    int nTaskId = mpObserver->GetTaskModule();
    
    rclcpp::QoS qosMap = rclcpp::QoS(rclcpp::KeepLast(100));
    //qosMap.best_effort();
    if(nTaskId==3)
        qosMap.reliable();
    else
        qosMap.reliable();
    //qosMap.durability(rclcpp::DurabilityPolicy(0)); // Volatile
    //qosMap.deadline(rclcpp::Duration(0, 400000000)); // 200ms
    qosMap.lifespan(rclcpp::Duration(0, 300000000)); // 100ms


    rclcpp::QoS qosKF = rclcpp::QoS(rclcpp::KeepLast(100));
    if(nTaskId==2)
        qosKF.reliable();
    else if(nTaskId==3)
        qosKF.best_effort();
    //qosKF.durability(rclcpp::DurabilityPolicy(0)); // Volatile
   // qosKF.deadline(rclcpp::Duration(0, 200000000)); // 200ms
    qosKF.lifespan(rclcpp::Duration(0, 100000000)); // 50ms
                                                   

    rclcpp::QoS qosAtlas = rclcpp::QoS(rclcpp::KeepLast(25));
    qosAtlas.reliable();
    //qosAtlas.durability(rclcpp::DurabilityPolicy(0)); // Volatile
    //qosKF.deadline(rclcpp::Duration(0, 200000000)); // 200ms
    //qosAtlas.lifespan(rclcpp::Duration(0, 100000000)); // 150ms


    if(nTaskId == 2)
    {
        /* KF */  
        RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame/LocalMapping");
        m_keyframe_lm_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
            "KeyFrame/LocalMapping",
            qosKF,
            //rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),//rmw_qos_profile_sensor_data),
            std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));//, options1);
    }
    
    if(nTaskId == 3)
    {
        /* KF */  
        RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame/LoopClosing");
        m_keyframe_lc_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
            "KeyFrame/LoopClosing",
            qosKF,
            //rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),//rmw_qos_profile_sensor_data),
            std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));//, options1);
    }
    // KF Update
    //RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
    //m_keyframe_update_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrameUpdate>(
    //    "KeyFrame/Update",
    //    rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default), //rmw_qos_profile_sensor_data),
    //    std::bind(&SlamWrapperNode::GrabKeyFrameUpdate, this, std::placeholders::_1));//, options1);
    
    if(nTaskId!=2)
    {
        /* Map */
        RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
        m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
            "Map",
            qosMap, 
            //rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),//rmw_qos_profile_sensor_data),
            std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));//, options2);
    }

    if(nTaskId!=3)
    {
        /* Atlas */
        RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Atlas/LoopClosing");
        m_atlas_lc_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Atlas>(
            "Atlas/LoopClosing",
            qosAtlas,
            //rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
            std::bind(&SlamWrapperNode::GrabAtlas, this, std::placeholders::_1));//, options3);
    }


    if(nTaskId!=3)
    {
        /* Atlas */
        RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Atlas/MapMerge");
        m_atlas_merge_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Atlas>(
            "Atlas/MapMerge",
            qosAtlas,
            //rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
            std::bind(&SlamWrapperNode::GrabAtlas, this, std::placeholders::_1));//, options3);
    }
    
    /* MapPoint */
    //RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /MapPoint");
    //m_map_point_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::MapPoint>(
    //    "MapPoint",
    //    rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
    //    std::bind(&SlamWrapperNode::GrabMapPoint, this, std::placeholders::_1));//, options3);
    

    /* LocalMapping active */
    //RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Active");
    //m_lm_active_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Bool>(
    //    "LocalMapping/Active",
    //    rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
    //    std::bind(&SlamWrapperNode::GrabLMActive, this, std::placeholders::_1));//, options3);

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
    rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(5));
    qosLatching.transient_local();
    qosLatching.reliable();
    
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /worker");
    worker_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/worker", 
        qosLatching, 
        std::bind(&SlamWrapperNode::workerCallback, this, std::placeholders::_1));//, options3);
    
    /* Stop LM */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /LocalMapping/Stop");
    stop_lm_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Bool>(
        "/LocalMapping/Stop", 
        10, 
        std::bind(&SlamWrapperNode::stopLMCallback, this, std::placeholders::_1));
}
