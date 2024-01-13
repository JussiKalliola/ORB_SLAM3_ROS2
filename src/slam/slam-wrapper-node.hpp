
#ifndef __SLAM_WRAPPER_NODE_HPP_
#define __SLAM_WRAPPER_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ActionChecker.hpp"

#include "utility.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"
#include "Converter.h"
#include "KeyFrameDatabase.h"
#include "GeometricCamera.h"

class SlamWrapperNode : public rclcpp::Node 
{
  public:
    SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam);

    ~SlamWrapperNode();
    //void publishMessage(const std::string& message_text);
    void publishKeyFrame(ORB_SLAM3::KeyFrame* pKf);
    void publishMap(ORB_SLAM3::Map* pM);
    void publishMapPoint(ORB_SLAM3::MapPoint* pMp);
    
    void publishLMActivityChange(bool bActive);
    //void publishMapAction();
    //void publishMapPointAction(orbslam3_interfaces::msg::MapPointActions::SharedPtr mpMsg);
    //void publishAtlasAction(orbslam3_interfaces::msg::AtlasActions::SharedPtr aMsg);
    //void publishKFAction(orbslam3_interfaces::msg::KeyFrameActions::SharedPtr kfMsg);   
    //void publishKFDBAction(orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr kfdbMsg);

    void publishEndMsg(); 


  private:
    ActionChecker* mpActionChecker;
    std::thread* mptActionChecker;
    
    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::Tracking* mpTracker_;
    ORB_SLAM3::LocalMapping* mpLocalMapper_;
    ORB_SLAM3::Atlas* mpAtlas_;    
    ORB_SLAM3::KeyFrameDatabase* mpKfDb_; 
    void checkForNewActions();
    
    void CreatePublishers();
    void CreateSubscribers();
    void endCallback(std_msgs::msg::Bool::SharedPtr bMsg);
    
    rclcpp::TimerBase::SharedPtr action_check_timer_;
    
    // Action callbacks
    void GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf);
    void GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM);
    void GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr rpMp);
    void GrabLMActive(const std_msgs::msg::Bool::SharedPtr msg);
    //void GrabAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rM);
    //void GrabKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rM);
    //void GrabKFDBAction(const orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr rKfdbA);
    //void GrabMapPointAction(const orbslam3_interfaces::msg::MapPointActions::SharedPtr rMpA);
    
    // Action Executors
    //bool PerformKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rKF);
    //bool PerformAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rAA);
    //bool PerformKFDBAction(const orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr rKfdb);
    //bool PerformMapPointAction(const orbslam3_interfaces::msg::MapPointActions::SharedPtr rMpA);

    // Publishers
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrame>::SharedPtr keyframe_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Map>::SharedPtr map_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::MapPoint>::SharedPtr map_point_publisher_;  
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lm_active_publisher_;  
    
    //rclcpp::Publisher<orbslam3_interfaces::msg::MapActions>::SharedPtr map_action_publisher_; 
    //rclcpp::Publisher<orbslam3_interfaces::msg::AtlasActions>::SharedPtr atlas_action_publisher_; 
    //rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrameActions>::SharedPtr kf_action_publisher_; 
    //rclcpp::Publisher<orbslam3_interfaces::msg::MapPointActions>::SharedPtr mp_action_publisher_; 
    //rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrameDatabaseActions>::SharedPtr kf_db_action_publisher_; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_publisher_;
    

    // Subscribers
    rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrame>::SharedPtr m_keyframe_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::Map>::SharedPtr m_map_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::MapPoint>::SharedPtr m_map_point_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_lm_active_subscriber_;     
    //rclcpp::Subscription<orbslam3_interfaces::msg::AtlasActions>::SharedPtr m_atlas_action_subscriber_;
    //rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrameActions>::SharedPtr m_kf_action_subscriber_;
    //rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrameDatabaseActions>::SharedPtr m_kfdb_action_subscriber_;
    //rclcpp::Subscription<orbslam3_interfaces::msg::MapPointActions>::SharedPtr m_mp_action_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr end_subscriber_;
    

    // Store SLAM data
    std::map<long unsigned int, std::tuple<ORB_SLAM3::Map*, orbslam3_interfaces::msg::Map::SharedPtr>> mpUnprocOrbMaps;
    std::map<long unsigned int, std::tuple<ORB_SLAM3::KeyFrame*, orbslam3_interfaces::msg::KeyFrame::SharedPtr>> mpUnprocOrbKeyFrames;
    std::map<long unsigned int, std::tuple<ORB_SLAM3::MapPoint*, orbslam3_interfaces::msg::MapPoint::SharedPtr>> mpUnprocOrbMapPoints;
    
    std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps;
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames;
    std::map<long unsigned int, ORB_SLAM3::MapPoint*> mpOrbMapPoints;
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mpOrbCameras;
    
      
    std::set<ORB_SLAM3::KeyFrame*> mspKeyFrames;
    std::set<ORB_SLAM3::MapPoint*> mspMapPoints;
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras;

    std::vector<unsigned long int> mvNewKFIds;
    
    // Store ROS Action messages for later prosessing
    std::vector<std::tuple<int, int>> mRosActionMap; // First is the action idx 0=Atlas, 1=KeyFrame, ..., and second is the idx in the corresponding vector
    std::vector<orbslam3_interfaces::msg::KeyFrameActions::SharedPtr> mvpKfRosActions; 
    std::vector<orbslam3_interfaces::msg::AtlasActions::SharedPtr> mvpAtlasRosActions; 
    std::vector<orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr> mvpKFDBRosActions; 
    std::vector<orbslam3_interfaces::msg::MapPointActions::SharedPtr> mvpMpRosActions; 

    //std::map<long unsigned int, orbslam3_interfaces::msg::KeyFrame*> mpRosKeyFrames;
    
    bool mbLocalMappingIsIdle;

    std::mutex mMutexNewKF;
    std::mutex mMutexUpdateMap;
};

#endif
