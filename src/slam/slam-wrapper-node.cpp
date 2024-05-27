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
void SlamWrapperNode::publishKeyFrame(orbslam3_interfaces::msg::KeyFrame::SharedPtr mRosKF) 
{
    // Add header
    std_msgs::msg::Header header;
    header.stamp = this->now();
    mRosKF->header = header;

    keyframe_publisher_->publish(*mRosKF);
    
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
void SlamWrapperNode::publishAtlas(orbslam3_interfaces::msg::Atlas mRosAtlas) 
{

    // Add header
    std_msgs::msg::Header header;
    header.stamp = this->now();
    mRosAtlas.header = header;
    
    atlas_publisher_->publish(mRosAtlas);

    RCLCPP_INFO(this->get_logger(), " ********************* Publishing Atlas after Merge=%d or LoopClosure=%d, #KFs=%d", mRosAtlas.mb_map_merge, mRosAtlas.mb_loop_closer, mpAtlas_->GetCurrentMap()->KeyFramesInMap());
}

/*        Map        */
void SlamWrapperNode::publishMap(orbslam3_interfaces::msg::Map mRosMap) 
{

    // Add header
    std_msgs::msg::Header header;
    header.stamp = this->now();
    mRosMap.header = header;
    
    map_publisher_->publish(mRosMap);
    
    RCLCPP_INFO(this->get_logger(), "Publishing a new map with id: %d", mRosMap.mn_id);
    RCLCPP_INFO(this->get_logger(), "Map Stats : #updated KFs=%d, #updated MPs=%d, #del KFs=%d, #del MPs=%d", 
        mRosMap.msp_keyframes.size(), 
        mRosMap.msp_map_points.size(), 
        mRosMap.mvp_erased_keyframe_ids.size(), 
        mRosMap.mvp_erased_mappoint_ids.size());
}



/*        MapPoint        */
void SlamWrapperNode::publishMapPoint(ORB_SLAM3::MapPoint* pMP) {
    //RCLCPP_INFO(this->get_logger(), "Publishing a new mappoint with id: %d", pMp->mnId);
    orbslam3_interfaces::msg::MapPoint mRosMP = Converter::MapPointConverter::ORBSLAM3MapPointToROS(pMP);
    mRosMP.system_id = std::getenv("SLAM_SYSTEM_ID");
    
    map_point_publisher_->publish(mRosMP);
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
    
    sys_reset_active_map_publisher_->publish(msg);
}



/* OBJECT SUBSCRIPTION CALLBACKS */

/*        KeyFrame        */
void SlamWrapperNode::GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF) {
  
    if(mpRosKF->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;

    rclcpp::Time msgTime = mpRosKF->header.stamp;
    std::chrono::system_clock::time_point time_Start(std::chrono::nanoseconds(msgTime.nanoseconds()));
    double timeLatency = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - time_Start).count();
    vdLatencyKF_ms.push_back(timeLatency);

    //RCLCPP_INFO(this->get_logger(), " **************************** Got a new keyframe, id: %d, for a map: %d", mpRosKF->mn_id, mpRosKF->mp_map_id);

    //auto timeDiff = rclcpp::Clock(RCL_SYSTEM_TIME).now() - mpRosKF->header.stamp;
    
    //RCLCPP_FATAL(this->get_logger(), " *** Latency=%d", mpRosKF->header.stamp.to_chrono<std::chrono::milliseconds>());
    mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKF);

}


void SlamWrapperNode::GrabKeyFrameUpdate(const orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKFUpdate) {
  
    if(mpRosKFUpdate->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;

    rclcpp::Time msgTime = mpRosKFUpdate->header.stamp;
    std::chrono::system_clock::time_point time_Start(std::chrono::nanoseconds(msgTime.nanoseconds()));
    double timeLatency = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - time_Start).count();
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
    double timeLatency = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - time_Start).count();
    vdLatencyAtlas_ms.push_back(timeLatency);
    
    RCLCPP_INFO(this->get_logger(), "Got Atlas Update, merge=%d, closure=%d", mpRosAtlas->mb_map_merge, mpRosAtlas->mb_loop_closer);
    
    mpMapHandler->InsertNewSubGlobalMap(mpRosAtlas);

}




/*        Map        */
void SlamWrapperNode::GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr mpRosMap) {
    if(mpRosMap->system_id == std::getenv("SLAM_SYSTEM_ID")) 
        return;
    
    rclcpp::Time msgTime = mpRosMap->header.stamp;
    std::chrono::system_clock::time_point time_Start(std::chrono::nanoseconds(msgTime.nanoseconds()));
    double timeLatency = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - time_Start).count();
    vdLatencyMap_ms.push_back(timeLatency);

    RCLCPP_INFO(this->get_logger(), "Got an update for a map, id=%d, from module=%d. Stats: #KFs=%d, #MPs=%d", mpRosMap->mn_id, mpRosMap->from_module_id, mpRosMap->msp_keyframes.size(), mpRosMap->msp_map_points.size());
    
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
    
    rclcpp::Time msgTime = msg->header.stamp;
    //rclcpp::Time now = this->now();
    std::chrono::nanoseconds ns(msgTime.nanoseconds());
    std::chrono::system_clock::time_point chrono_time_point(ns);
    double mdTime = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - chrono_time_point).count();
    //std::cout << "Time since msg was sent=" << (now-msgTime).nanoseconds()/1000000 << std::endl;
    std::cout << "Time since msg was sent=" << mdTime<< std::endl;
    mpObserver->UpdateLastResetTime();
    
    RCLCPP_INFO(this->get_logger(), "Reset requested for Active map id=%d", msg->data);

    mpLocalMapper_->Release();

    mpObserver->ClearMapPointsFromMap(mpAtlas_->GetCurrentMap());
    mpObserver->ClearKeyFramesFromMap(mpAtlas_->GetCurrentMap());

    if(mpObserver->GetTaskModule() == 2)
        mpTracker_->ResetActiveMap(true, msg->data);
    else
        mpTracker_->ResetActiveMap(false, msg->data);
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

    rclcpp::QoS qosMap = rclcpp::QoS(rclcpp::KeepLast(1));
    qosMap.best_effort();
    //qosMap.deadline(rclcpp::Duration(1000));
    //qosMap.lifespan(rclcpp::Duration(1));

    /* KEYFRAME */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame");
    keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
        "/KeyFrame/New", 
        rclcpp::QoS(rclcpp::KeepLast(1),  rmw_qos_profile_sensor_data));//rmw_qos_profile_sensor_data));

    /* KEYFRAME */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /KeyFrame");
    keyframe_update_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrameUpdate>(
        "/KeyFrame/Update", 
        rclcpp::QoS(rclcpp::KeepLast(1),  rmw_qos_profile_sensor_data));//rmw_qos_profile_sensor_data));
    
    /* MAP */
    RCLCPP_INFO(this->get_logger(), "Creating a publisher for a topic /Map");
    map_publisher_ = this->create_publisher<orbslam3_interfaces::msg::Map>(
        "/Map", 
        //qosMap);
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
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data));
    
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
    
    
    rclcpp::QoS qosMap = rclcpp::QoS(rclcpp::KeepLast(1));
    qosMap.best_effort();
    //qosMap.deadline(rclcpp::Duration(1000));
    //qosMap.lifespan(rclcpp::Duration(1));
    

    /* KF */  
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
    m_keyframe_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrame>(
        "KeyFrame/New",
        rclcpp::QoS(rclcpp::KeepLast(10),  rmw_qos_profile_sensor_data),//rmw_qos_profile_sensor_data),
        std::bind(&SlamWrapperNode::GrabKeyFrame, this, std::placeholders::_1));//, options1);
    
    // KF Update
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /KeyFrame");
    m_keyframe_update_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::KeyFrameUpdate>(
        "KeyFrame/Update",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data), //rmw_qos_profile_sensor_data),
        std::bind(&SlamWrapperNode::GrabKeyFrameUpdate, this, std::placeholders::_1));//, options1);
    
    /* Map */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Map");
    m_map_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Map>(
        "Map",
        //qosMap, 
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&SlamWrapperNode::GrabMap, this, std::placeholders::_1));//, options2);

    /* Atlas */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /Atlas");
    m_atlas_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::Atlas>(
        "Atlas",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&SlamWrapperNode::GrabAtlas, this, std::placeholders::_1));//, options3);
    
    /* MapPoint */
    RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /MapPoint");
    m_map_point_subscriber_ = this->create_subscription<orbslam3_interfaces::msg::MapPoint>(
        "MapPoint",
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
        std::bind(&SlamWrapperNode::GrabMapPoint, this, std::placeholders::_1));//, options3);
    

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
