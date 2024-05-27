
#ifndef __SLAM_WRAPPER_NODE_HPP_
#define __SLAM_WRAPPER_NODE_HPP_

// ROS2 standard
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"

// orbslam3 interfaces
#include "orbslam3_interfaces/msg/int64.hpp"
#include "orbslam3_interfaces/msg/bool.hpp"
#include "orbslam3_interfaces/msg/key_frame.hpp"
#include "orbslam3_interfaces/msg/map.hpp"
#include "orbslam3_interfaces/msg/atlas.hpp"
#include "orbslam3_interfaces/msg/map_point.hpp"

// Utils
#include "utility.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>

// ORB SLAM3
#include "System.h"

class Observer;
class System;
class MapHandler;
class KeyFramePublisher;
class KeyFrameSubscriber;

class SlamWrapperNode : public rclcpp::Node 
{
  public:
    SlamWrapperNode(ORB_SLAM3::System* pSLAM, std::shared_ptr<System> pDistSystem, bool subscribe_to_slam, const std::string path="./", const std::string strResultFilename="KeyFrameTrajectory.txt");

    ~SlamWrapperNode();
    
    // Publishers
    void publishKeyFrame(orbslam3_interfaces::msg::KeyFrame::SharedPtr mRosKF);
    void publishKeyFrame(orbslam3_interfaces::msg::KeyFrameUpdate mRosKFUpdate);
    void publishMap(orbslam3_interfaces::msg::Map mRosMap);
    void publishMap(ORB_SLAM3::Map* pM);
    void publishMapPoint(ORB_SLAM3::MapPoint* pMp);
    void publishAtlas(orbslam3_interfaces::msg::Atlas mRosAtlas);
    
    void publishResetActiveMap(unsigned long int mnMapId);
    void publishLMResetRequested();
    //void publishLMActivityChange(bool bActive);
    void publishEndMsg(); 
    void publishStep(); 
    
    void CreatePublishers();
    void CreateSubscribers();
   
    // ROS Latencies
    vector<double> vdLatencyKF_ms;
    vector<double> vdLatencyMap_ms;
    vector<double> vdLatencyAtlas_ms;

      
  protected:
    //std::shared_ptr<Distributor> mpDistributor;
    std::shared_ptr<System> mpDistributionSystem;
    std::shared_ptr<Observer> mpObserver;
    MapHandler* mpMapHandler;
    KeyFramePublisher* mpKeyFramePublisher;
    KeyFrameSubscriber* mpKeyFrameSubscriber;


  private:
    std::string savePath;
    std::string mstrResultFilename;
    
    // ORB SLAM3 Data
    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::Tracking* mpTracker_;
    ORB_SLAM3::LocalMapping* mpLocalMapper_;
    ORB_SLAM3::LoopClosing* mpLoopCloser_;
    ORB_SLAM3::Atlas* mpAtlas_;    
    ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB; 
    
    
    // Action callbacks
    void GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf);
    void GrabKeyFrameUpdate(const orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKFUpdate);
    void GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM);
    void GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr rpMp);
    void GrabAtlas(const orbslam3_interfaces::msg::Atlas::SharedPtr rAtlas);
    
    //void GrabLMActive(const orbslam3_interfaces::msg::Bool::SharedPtr msg);
    void GrabLMResetRequested(const std_msgs::msg::Bool::SharedPtr msg);
    void GrabResetActiveMap(const orbslam3_interfaces::msg::Int64::SharedPtr msg);
    
    void workerCallback(std_msgs::msg::Int32::SharedPtr msg);
    void endCallback(std_msgs::msg::Bool::SharedPtr bMsg);
    void stepCallback(std_msgs::msg::Bool::SharedPtr bMsg);
    

    // Publishers
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrame>::SharedPtr keyframe_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrameUpdate>::SharedPtr keyframe_update_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Map>::SharedPtr map_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::MapPoint>::SharedPtr map_point_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Atlas>::SharedPtr atlas_publisher_;  
    
    //rclcpp::Publisher<orbslam3_interfaces::msg::Bool>::SharedPtr lm_active_publisher_;  
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lm_reset_requested_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Int64>::SharedPtr sys_reset_active_map_publisher_;  
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr step_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr worker_publisher_;  
    

    // Subscribers
    rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrame>::SharedPtr m_keyframe_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrameUpdate>::SharedPtr m_keyframe_update_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::Map>::SharedPtr m_map_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::MapPoint>::SharedPtr m_map_point_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::Atlas>::SharedPtr m_atlas_subscriber_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_lm_reset_requested_subscriber_;     
    //rclcpp::Subscription<orbslam3_interfaces::msg::Bool>::SharedPtr m_lm_active_subscriber_;     
    rclcpp::Subscription<orbslam3_interfaces::msg::Int64>::SharedPtr m_sys_reset_active_map_subscriber_;     
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr worker_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr end_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr step_subscriber_;
};

#endif
