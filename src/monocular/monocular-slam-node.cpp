#include "monocular-slam-node.hpp"


#include<thread>
#include <rmw/qos_profiles.h>
#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, std::shared_ptr<SlamWrapperNode> slam_node, const std::string path, const std::string strResultFilename, const std::string strDatasetName, const std::string strCameratopic) : Node("MonocularSlamNode") 
{
  RCLCPP_INFO(this->get_logger(), "Initializing Monocular SLAM node.");
  m_SLAM = pSLAM;
  slam_node_ = slam_node;

  savePath = path;
  mstrResultFilename = strResultFilename;
  mstrDatasetName= strDatasetName;
  mstrCameraTopic = strCameratopic;

    // std::cout << "slam changed" << std::endl;
  RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /'%s'", mstrCameraTopic);
  m_image_subscriber = this->create_subscription<ImageMsg>(
      mstrCameraTopic,
      rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default),
      std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));



  /* Worker node */
  //rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(10));
  //qosLatching.transient_local();
  //qosLatching.reliable();
  //
  //RCLCPP_INFO(this->get_logger(), "Creating a subscriber for a topic /worker");
  //worker_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
  //    "/worker", 
  //    qosLatching, 
  //    std::bind(&MonocularSlamNode::workerCallback, this, std::placeholders::_1));//, options3);

  //mbTimerRunning = false;
  //msWorkers = std::set<int>();


  //// Timer 20fps
  timer_ = this->create_wall_timer(
      50ms, std::bind(&MonocularSlamNode::timer_callback, this));
  //StopTimer();

  //RCLCPP_INFO(this->get_logger(), "Reading ROS2 bag...");
  //rosbag2_storage::StorageOptions storage_options;
  //storage_options.uri = "/root/datasets/TUM/corridor1/corridor1.db3";
  //reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  //reader_->open(storage_options);
  //RCLCPP_INFO(this->get_logger(), "Rosbag and node initialization finished.");
}

MonocularSlamNode::~MonocularSlamNode()
{
  TimeStats2File();
  RCLCPP_FATAL(this->get_logger(),  "~MonocularSlamNode");
  //StopTimer();
  
  //rclcpp::sleep_for(std::chrono::seconds(1));
  // Stop all threads
  //m_SLAM->Shutdown();
  // Save camera trajectory
}


void MonocularSlamNode::StopTimer()
{
    mbTimerRunning = false;
    timer_->cancel(); 
}

void MonocularSlamNode::StartTimer()
{
    std::cout << "Start timer" << std::endl;
    mbTimerRunning = true;
    timer_->reset(); 
}

void MonocularSlamNode::timer_callback()
{
   
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_Start = std::chrono::steady_clock::now();

    cv::Mat curIm;
    builtin_interfaces::msg::Time curStamp;

    {
        unique_lock<mutex> lock(mMutexIm);
        curIm = m_latestIm.clone();
        curStamp = m_latestStamp;

        m_latestIm.release();
    }

    
    if(curIm.empty())
        return;

    m_SLAM->TrackMonocular(curIm, Utility::StampToSec(curStamp));


    std::chrono::steady_clock::time_point time_End = std::chrono::steady_clock::now();
    double timeTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_End - time_Start).count();
    vdTrackingTimes_ms.push_back(timeTrack);
  //if(!mbTimerRunning)
  //{
  //  std::cout << "Timer has not started" << std::endl;
  //  return;
  //}

  //while (reader_->has_next()) {
  //  rosbag2_storage::SerializedBagMessageSharedPtr bagMsg = reader_->read_next();

  //  if (bagMsg->topic_name != "/cam0/image_raw") {
  //    continue;
  //  }

  //  rclcpp::SerializedMessage serialized_msg(*bagMsg->serialized_data);
  //  ImageMsg::SharedPtr msg = std::make_shared<ImageMsg>();

  //  serialization_.deserialize_message(&serialized_msg, msg.get());

  //  // Start of a timer -------------
  //  std::chrono::steady_clock::time_point time_Start = std::chrono::steady_clock::now();
  //  // Copy the ros image message to cv::Mat.
  //  try
  //  {
  //      m_cvImPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  //  }
  //  catch (cv_bridge::Exception& e)
  //  {
  //      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  //      return;
  //  }


  //  cv::Mat im = m_cvImPtr->image;
  //  if(mstrDatasetName=="TUM")
  //  {
  //      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  //      // clahe
  //      clahe->apply(im,im);

  //  }
  //  // publisher_node_->publishMessage("Was able to grab image.");
  //  //std::cout<<"one frame has been sent"<<std::endl;
  //  //thread t1(&ORB_SLAM3::System::TrackMonocular, 
  //  //    m_SLAM, 
  //  //    m_cvImPtr->image, 
  //  //    Utility::StampToSec(msg->header.stamp),
  //  //    vector<ORB_SLAM3::IMU::Point>(),
  //  //    std::string("asd"));
  //  //t1.join();
  //  m_SLAM->TrackMonocular(im, Utility::StampToSec(msg->header.stamp));

  //  // End of timer
  //  std::chrono::steady_clock::time_point time_End = std::chrono::steady_clock::now();
  //  double timeTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_End - time_Start).count();
  //  vdTrackingTimes_ms.push_back(timeTrack);
  //  break;
  //}
}

void MonocularSlamNode::workerCallback(std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received msg from /worker, new worker added to the network=%d.", msg->data);
    msWorkers.insert(msg->data);
    if(msWorkers.size() == 3)  
      StartTimer();
}

void MonocularSlamNode::TimeStats2File()
{
    ofstream f;
    string sysId(std::getenv("SLAM_SYSTEM_ID"));
    std::string fileName=savePath + sysId + "/TrackingOverallStats-"+sysId+".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Time[ms]" << endl;

    for(int i=0; i<vdTrackingTimes_ms.size(); ++i)
    {
        f << vdTrackingTimes_ms[i]<< endl;
    }

    f.close();
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    
    // Start of a timer -------------
    //std::chrono::steady_clock::time_point time_Start = std::chrono::steady_clock::now();
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
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        // clahe
        clahe->apply(im,im);

    }

    {
        unique_lock<mutex> lock(mMutexIm);
        m_latestIm = im;
        m_latestStamp = msg->header.stamp;
    }

    // publisher_node_->publishMessage("Was able to grab image.");
    //std::cout<<"one frame has been sent"<<std::endl;
    //thread t1(&ORB_SLAM3::System::TrackMonocular, 
    //    m_SLAM, 
    //    m_cvImPtr->image, 
    //    Utility::StampToSec(msg->header.stamp),
    //    vector<ORB_SLAM3::IMU::Point>(),
    //    std::string("asd"));
    //t1.join();

    //m_SLAM->TrackMonocular(im, Utility::StampToSec(msg->header.stamp));

    // End of timer
    //std::chrono::steady_clock::time_point time_End = std::chrono::steady_clock::now();
    //double timeTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_End - time_Start).count();
    //vdTrackingTimes_ms.push_back(timeTrack);
}
