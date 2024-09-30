#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"

#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

#include "../slam/slam-wrapper-node.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"



class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM, std::shared_ptr<SlamWrapperNode> slam_node, const std::string path, const std::string strResultFilename, const std::string strDatasetName, const std::string strCameraTopic);
    void TimeStats2File();

    ~MonocularSlamNode();
    
    vector<double> vdTrackingTimes_ms;

protected:
    void StopTimer();
    void StartTimer();
    void timer_callback();
    void workerCallback(std_msgs::msg::Int32::SharedPtr msg);

    bool mbTimerRunning;

    std::set<int> msWorkers;

    
    std::mutex mMutexIm;
    cv::Mat m_latestIm;
    builtin_interfaces::msg::Time m_latestStamp;

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;
    std::shared_ptr<SlamWrapperNode> slam_node_;
    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr worker_subscriber_;

    std::string savePath;
    std::string mstrResultFilename;
    std::string mstrDatasetName;
    std::string mstrCameraTopic;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Serialization<ImageMsg> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

#endif
