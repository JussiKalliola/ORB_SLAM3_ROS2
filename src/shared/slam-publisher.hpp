#ifndef __SLAM_PUBLISHER_NODE_HPP_
#define __SLAM_PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "orbslam3_interfaces/msg/key_frame.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "Converter.h"

class SLAMPublisher : public rclcpp::Node 
{
  public:
    SLAMPublisher();
    
    void publishMessage(const std::string& message_text);
    void publishKeyFrame(ORB_SLAM3::KeyFrame* pKf);
    
    ~SLAMPublisher();

  private:  
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrame>::SharedPtr keyframe_publisher_;

    orbslam3_interfaces::msg::KeyFrame KeyFrameConstructor(ORB_SLAM3::KeyFrame* pKf);
};

#endif
