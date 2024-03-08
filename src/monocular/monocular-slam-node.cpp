#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, std::shared_ptr<SlamWrapperNode> slam_node, const std::string path, const std::string strResultFilename) : Node("MonocularSlamNode") 
{
  RCLCPP_INFO(this->get_logger(), "Initializing Monocular SLAM node.");
  m_SLAM = pSLAM;
  slam_node_ = slam_node;

  savePath = path;
  mstrResultFilename = strResultFilename;

    // std::cout << "slam changed" << std::endl;
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /camera");
  m_image_subscriber = this->create_subscription<ImageMsg>(
      "camera",
      10,
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

    // publisher_node_->publishMessage("Was able to grab image.");
    //std::cout<<"one frame has been sent"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}
