
#ifndef __SLAM_WRAPPER_NODE_HPP_
#define __SLAM_WRAPPER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "orbslam3_interfaces/Converter.hpp"
#include "orbslam3_interfaces/KeyFrameConverter.hpp"

#include "utility.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "Converter.h"

class SlamWrapperNode : public rclcpp::Node 
{
  public:
    SlamWrapperNode(ORB_SLAM3::System* pSLAM, bool subscribe_to_slam);

    //void publishMessage(const std::string& message_text);
    void publishKeyFrame(ORB_SLAM3::KeyFrame* pKf);
   
    ~SlamWrapperNode();

  private:
    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::LocalMapping* mpLocalMapper_;
    ORB_SLAM3::Atlas* mpAtlas_;    
    void GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf);

    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrame>::SharedPtr keyframe_publisher_;  
    rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrame>::SharedPtr m_keyframe_subscriber_;
    
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames;
    //std::map<long unsigned int, orbslam3_interfaces::msg::KeyFrame*> mpRosKeyFrames;
};

#endif
