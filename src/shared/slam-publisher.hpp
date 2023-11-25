#ifndef __SLAM_PUBLISHER_NODE_HPP_
#define __SLAM_PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "orbslam3_interfaces/msg/atlas.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

class SLAMPublisher : public rclcpp::Node 
{
  public:
    SLAMPublisher();
    
    void publishMessage(const std::string& message_text);
    
    ~SLAMPublisher();

  private:  
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif
