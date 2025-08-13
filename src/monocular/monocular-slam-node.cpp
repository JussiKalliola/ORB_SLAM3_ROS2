#include "monocular-slam-node.hpp"
#include <rmw/qos_profiles.h>

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, std::shared_ptr<SlamWrapperNode> slam_node, const std::string path, const std::string strResultFilename, const std::string strDatasetName, rclcpp::NodeOptions nOptions) : Node("MonocularSlamNode", nOptions) 
{
  RCLCPP_INFO(this->get_logger(), "Initializing Monocular SLAM node.");
  m_SLAM = pSLAM;
  slam_node_ = slam_node;

  savePath = path;
  mstrResultFilename = strResultFilename;
  mstrDatasetName = strDatasetName;


  // sim clock related
  start_time = this->now();
  publisher_clock_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1).best_effort().durability_volatile());
  RCLCPP_INFO_STREAM(this->get_logger(), "Start the /clock timer.");
  this->clock_callback();
  clock_timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      10ms, //16.667ms,
      std::bind(&MonocularSlamNode::clock_callback, this)
  );

    // std::cout << "slam changed" << std::endl;
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /camera");
  m_image_subscriber = this->create_subscription<ImageMsg>(
      "camera",
      10,
      //rclcpp::QoS(rclcpp::KeepLast(10),  rmw_qos_profile_sensor_data),//rmw_qos_profile_sensor_data),
      std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
}

MonocularSlamNode::~MonocularSlamNode()
{
  RCLCPP_FATAL(this->get_logger(),  "~MonocularSlamNode");
  
  //rclcpp::sleep_for(std::chrono::seconds(1));
  // Stop all threads
  //m_SLAM->Shutdown();
  // Save camera trajectory
}



void MonocularSlamNode::clock_callback()
{
    auto clock_msg = rosgraph_msgs::msg::Clock();

    auto cur_time = this->now();
    rclcpp::Duration elapsed = cur_time - start_time;

    clock_msg.clock.sec = (int)(elapsed.seconds());
    clock_msg.clock.nanosec = (long int)((elapsed.seconds() - clock_msg.clock.sec)*1e9);
    publisher_clock_->publish(clock_msg);

}


void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat im = m_cvImPtr->image;
    if(mstrDatasetName=="TUM")
    {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8,8));
      clahe->apply(im,im);
    }

    // publisher_node_->publishMessage("Was able to grab image.");
    //std::cout<<"one frame has been sent"<<std::endl;
    m_SLAM->TrackMonocular(im, Utility::StampToSec(msg->header.stamp));
}
