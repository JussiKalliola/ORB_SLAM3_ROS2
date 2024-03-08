
#ifndef __SLAM_WRAPPER_NODE_HPP_
#define __SLAM_WRAPPER_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "orbslam3_interfaces/msg/int64.hpp"
#include "orbslam3_interfaces/msg/bool.hpp"
//#include "ActionChecker.hpp"


#include "orbslam3_interfaces/Converter.hpp"
#include "orbslam3_interfaces/KeyFrameConverter.hpp"
#include "orbslam3_interfaces/MapConverter.hpp"



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
    SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam, const std::string path="./", const std::string strResultFilename="KeyFrameTrajectory.txt");

    ~SlamWrapperNode();
    //void publishMessage(const std::string& message_text);
    void publishKeyFrame(ORB_SLAM3::KeyFrame* pKf);
    void publishMap(ORB_SLAM3::Map* pM);
    void publishMapPoint(ORB_SLAM3::MapPoint* pMp);
    
    void publishResetActiveMap(unsigned long int mnMapId);
    void publishLMResetRequested();
    void publishLMActivityChange(bool bActive);
    void publishEndMsg(); 
    void publishStep(); 


  private:
    //ActionChecker* mpActionChecker;
    //std::thread* mptActionChecker;
    std::string savePath;
    std::string mstrResultFilename;
    
    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::Tracking* mpTracker_;
    ORB_SLAM3::LocalMapping* mpLocalMapper_;
    ORB_SLAM3::Atlas* mpAtlas_;    
    ORB_SLAM3::KeyFrameDatabase* mpKfDb_; 
    void checkForNewActions();
    void startTimer();
    void stopTimer();
    
    void CreatePublishers();
    void CreateSubscribers();
    
    void endCallback(std_msgs::msg::Bool::SharedPtr bMsg);
    void stepCallback(std_msgs::msg::Bool::SharedPtr bMsg);
    
    rclcpp::TimerBase::SharedPtr action_check_timer_;
    
    // Action callbacks
    void GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf);
    void GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM);
    void GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr rpMp);
    void GrabLMActive(const orbslam3_interfaces::msg::Bool::SharedPtr msg);
    void GrabLMResetRequested(const std_msgs::msg::Bool::SharedPtr msg);
    void GrabResetActiveMap(const orbslam3_interfaces::msg::Int64::SharedPtr msg);

    // Publishers
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrame>::SharedPtr keyframe_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Map>::SharedPtr map_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::MapPoint>::SharedPtr map_point_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Bool>::SharedPtr lm_active_publisher_;  
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lm_reset_requested_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Int64>::SharedPtr sys_reset_active_map_publisher_;  
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr step_publisher_;
    

    // Subscribers
    rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrame>::SharedPtr m_keyframe_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::Map>::SharedPtr m_map_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::MapPoint>::SharedPtr m_map_point_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_lm_reset_requested_subscriber_;     
    rclcpp::Subscription<orbslam3_interfaces::msg::Bool>::SharedPtr m_lm_active_subscriber_;     
    rclcpp::Subscription<orbslam3_interfaces::msg::Int64>::SharedPtr m_sys_reset_active_map_subscriber_;     
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr end_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr step_subscriber_;

    // Store SLAM data
    std::map<long unsigned int, std::tuple<ORB_SLAM3::Map*, orbslam3_interfaces::msg::Map::SharedPtr>> mpUnprocOrbMaps;
    std::map<long unsigned int, orbslam3_interfaces::msg::KeyFrame::SharedPtr> mpUnprocOrbKeyFrames;
    std::map<std::string, std::tuple<ORB_SLAM3::MapPoint*, orbslam3_interfaces::msg::MapPoint::SharedPtr>> mpUnprocOrbMapPoints;
    
    std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps;
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames;
    std::map<std::string, ORB_SLAM3::MapPoint*> mpOrbMapPoints;
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mpOrbCameras;

    std::set<unsigned long int> mspKFsReadyForLM;
    std::set<unsigned long int> mspUnprocKFids;
      
    std::set<ORB_SLAM3::KeyFrame*> mspKeyFrames;
    std::set<ORB_SLAM3::MapPoint*> mspMapPoints;
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras;

    std::vector<unsigned long int> mvNewKFIds;
    
    //std::map<long unsigned int, orbslam3_interfaces::msg::KeyFrame*> mpRosKeyFrames;
    
    bool mbLocalMappingIsIdle;

    std::mutex mMutexNewKF;
    std::mutex mMutexPublishKF;
    std::mutex mMutexUpdateMap;
    std::mutex mMutexPublishMap;
    std::mutex mMutexChecker;
};

#endif
