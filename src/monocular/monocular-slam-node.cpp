#include "monocular-slam-node.hpp"
//#include "../shared/slam-publisher.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, const std::string path, const std::string strResultFilename)
: Node("ORB_SLAM3_ROS2") 
{
    m_SLAM = pSLAM;
    savePath = path;
    mstrResultFilename = strResultFilename;

    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;


}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();
    // Save camera trajectory
    std::cout << "Saving data to the path=" + savePath + mstrResultFilename << std::endl; 
    m_SLAM->SaveKeyFrameTrajectoryTUM(savePath + mstrResultFilename);
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
