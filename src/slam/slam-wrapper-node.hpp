
#ifndef __SLAM_WRAPPER_NODE_HPP_
#define __SLAM_WRAPPER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "orbslam3_interfaces/Converter.hpp"
#include "orbslam3_interfaces/KeyFrameConverter.hpp"
#include "orbslam3_interfaces/MapConverter.hpp"
#include "orbslam3_interfaces/AtlasConverter.hpp"
#include "orbslam3_interfaces/ActionParser.hpp"
#include "std_msgs/msg/int64.hpp"

#include "utility.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"
#include "Converter.h"

class SlamWrapperNode : public rclcpp::Node 
{
  public:
    SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam);

    //void publishMessage(const std::string& message_text);
    void publishKeyFrame(ORB_SLAM3::KeyFrame* pKf);
    void publishMap(ORB_SLAM3::Map* pM);
    void publishMapAction();
    void publishAtlasAction(orbslam3_interfaces::msg::AtlasActions aMsg);
    void publishKFAction(orbslam3_interfaces::msg::KeyFrameActions kfMsg);   
    
     ~SlamWrapperNode();

  private:
    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::LocalMapping* mpLocalMapper_;
    ORB_SLAM3::Atlas* mpAtlas_;    
   
    void checkForNewActions();
    
    void CreatePublishers();
    void CreateSubscribers();
    
    rclcpp::TimerBase::SharedPtr action_check_timer_;

    void GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf);
    void GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM);
    void GrabAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rM);
    void GrabKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rM);
    bool PerformKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rKF);
    bool PerformAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rAA);

    // Publishers
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrame>::SharedPtr keyframe_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Map>::SharedPtr map_publisher_;  
    
    rclcpp::Publisher<orbslam3_interfaces::msg::MapActions>::SharedPtr map_action_publisher_; 
    rclcpp::Publisher<orbslam3_interfaces::msg::AtlasActions>::SharedPtr atlas_action_publisher_; 
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrameActions>::SharedPtr kf_action_publisher_; 
    

    // Subscribers
    rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrame>::SharedPtr m_keyframe_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::Map>::SharedPtr m_map_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::AtlasActions>::SharedPtr m_atlas_action_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrameActions>::SharedPtr m_kf_action_subscriber_;
    
    // Store SLAM data
    std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps;
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames;
    std::map<long unsigned int, ORB_SLAM3::MapPoint*> mpOrbMapPoints;


    std::vector<unsigned long int> mvNewKFIds;
    
    // Store ROS Action messages for later prosessing
    std::vector<std::tuple<int, int>> mRosActionMap; // First is the action idx 0=Atlas, 1=KeyFrame, ..., and second is the idx in the corresponding vector
    std::vector<orbslam3_interfaces::msg::KeyFrameActions::SharedPtr> mvpKfRosActions; 
    std::vector<orbslam3_interfaces::msg::AtlasActions::SharedPtr> mvpAtlasRosActions; 

    //std::map<long unsigned int, orbslam3_interfaces::msg::KeyFrame*> mpRosKeyFrames;
};

#endif
