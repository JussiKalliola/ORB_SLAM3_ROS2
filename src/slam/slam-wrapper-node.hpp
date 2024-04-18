
#ifndef __SLAM_WRAPPER_NODE_HPP_
#define __SLAM_WRAPPER_NODE_HPP_


#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "orbslam3_interfaces/msg/int64.hpp"
#include "orbslam3_interfaces/msg/bool.hpp"
//#include "ActionChecker.hpp"


#include "orbslam3_interfaces/Converter.hpp"
#include "orbslam3_interfaces/KeyFrameConverter.hpp"
#include "orbslam3_interfaces/MapConverter.hpp"
#include "orbslam3_interfaces/AtlasConverter.hpp"


#include "utility.hpp"

#include <cv_bridge/cv_bridge.h>

//#include "../Distributor/System.hpp"
//#include "../Distributor/Observer.hpp"

#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"
#include "Converter.h"
#include "KeyFrameDatabase.h"
#include "GeometricCamera.h"

class Distributor;
class Observer;
class System;
class MapHandler;
class KeyFramePublisher;
class KeyFrameSubscriber;

class SlamWrapperNode : public rclcpp::Node 
{
  public:
    SlamWrapperNode(ORB_SLAM3::System* pSLAM, std::shared_ptr<System> pDistSystem, bool subscribe_to_slam, const std::string path="./", const std::string strResultFilename="KeyFrameTrajectory.txt");

    ~SlamWrapperNode();
    
    // Publishers
    void publishKeyFrame(orbslam3_interfaces::msg::KeyFrame mRosKF);
    void publishMap(orbslam3_interfaces::msg::Map mRosMap);
    void publishMap(ORB_SLAM3::Map* pM);
    void publishMapPoint(ORB_SLAM3::MapPoint* pMp);
    void publishAtlas(orbslam3_interfaces::msg::Atlas mRosAtlas);
    
    void publishResetActiveMap(unsigned long int mnMapId);
    void publishLMResetRequested();
    void publishLMActivityChange(bool bActive);
    void publishEndMsg(); 
    void publishStep(); 

    // Reset
    void UpdateReset(bool mbReset);
    bool GetResetStatus();
    
    // Activity
    void SetKeyFrameAction(bool mbAction);
    void SetMapAction(bool mbAction);
    void SetAtlasAction(bool mbAction);

    bool KeyFrameActionActive();
    bool MapActionActive();
    bool AtlasActionActive();

    // ORB SLAM3 Functions
    void AddKeyFrame(ORB_SLAM3::KeyFrame* pKF);
    void EraseKeyFrame(ORB_SLAM3::KeyFrame* pKF);
    
    void AddMapPoint(ORB_SLAM3::MapPoint* pMP);
    void EraseMapPoint(ORB_SLAM3::MapPoint* pMP);

    void AddMap(ORB_SLAM3::Map* pM);
    void EraseMap(ORB_SLAM3::Map* pM);
    
    // Keep track of the biggest ids (used in optimization)
    unsigned long int GetMaxMapPointID();
    unsigned long int GetMaxKeyFrameID();

    void CalcMaxMapPointID();
    void CalcMaxKeyFrameID();
      
    void IncreaseKFCount(int inc);
    long int GetKFCount();


    // Generic functions
    void checkForNewActions();
    void startTimer();
    void stopTimer();
    
    void CreatePublishers();
    void CreateSubscribers();
      
  protected:
    
    ORB_SLAM3::KeyFrame* ProcessNewKeyFrame(std::shared_ptr<orbslam3_interfaces::msg::KeyFrame> mpRosKF, std::map<unsigned long int, ORB_SLAM3::Map*> mMaps);
    ORB_SLAM3::MapPoint* ProcessNewMapPoint(std::shared_ptr<orbslam3_interfaces::msg::MapPoint> mpRosMP, std::map<unsigned long int, ORB_SLAM3::Map*> mMaps);
    
    ORB_SLAM3::KeyFrame* InjectKeyFrame(ORB_SLAM3::KeyFrame* tempKF, std::map<unsigned long int, ORB_SLAM3::KeyFrame*> mFusedKFs);
    void InjectMapPoint(ORB_SLAM3::MapPoint* tempMP, std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs);
    void InjectMap(ORB_SLAM3::Map* tempMap, ORB_SLAM3::Map* mpCurrentMap);

    std::shared_ptr<Distributor> mpDistributor;
    std::shared_ptr<System> mpDistributionSystem;
    std::shared_ptr<Observer> mpObserver;
    MapHandler* mpMapHandler;
    KeyFramePublisher* mpKeyFramePublisher;
    KeyFrameSubscriber* mpKeyFrameSubscriber;

    void ForwardKeyFrameToTarget(ORB_SLAM3::KeyFrame* pKF, const unsigned int mnTargetModule, const unsigned int nFromModule);
    bool mbKeyFrameAction;
    bool mbMapAction;
    bool mbAtlasAction;

    long int mnKeyFramesAfterLC;

    bool mbResetActive;

    unsigned int mnTaskModule;
    
    unsigned long int mnMaxMPId;
    unsigned long int mnMaxKFId;

    std::mutex mMutexReset;
    std::mutex mMutexKF;
    std::mutex mMutexMap;
    std::mutex mMutexAtlas;
    std::mutex mMutexMapPoint;

    std::mutex mMutexNewKF;
    std::mutex mMutexPublishKF;
    std::mutex mMutexUpdateMap;
    std::mutex mMutexPublishMap;
    std::mutex mMutexChecker;

  private:
    std::string savePath;
    std::string mstrResultFilename;
    
    // ORB SLAM3 Data
    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::Tracking* mpTracker_;
    ORB_SLAM3::LocalMapping* mpLocalMapper_;
    ORB_SLAM3::LoopClosing* mpLoopCloser_;
    ORB_SLAM3::Atlas* mpAtlas_;    
    ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB; 
    
    
    rclcpp::TimerBase::SharedPtr action_check_timer_;
    
    // Action callbacks
    void GrabKeyFrame(const orbslam3_interfaces::msg::KeyFrame::SharedPtr rKf);
    void GrabMap(const orbslam3_interfaces::msg::Map::SharedPtr rM);
    void GrabMapPoint(const orbslam3_interfaces::msg::MapPoint::SharedPtr rpMp);
    void GrabAtlas(const orbslam3_interfaces::msg::Atlas::SharedPtr rAtlas);
    
    void GrabLMActive(const orbslam3_interfaces::msg::Bool::SharedPtr msg);
    void GrabLMResetRequested(const std_msgs::msg::Bool::SharedPtr msg);
    void GrabResetActiveMap(const orbslam3_interfaces::msg::Int64::SharedPtr msg);
    
    void workerCallback(std_msgs::msg::Int32::SharedPtr msg);
    void endCallback(std_msgs::msg::Bool::SharedPtr bMsg);
    void stepCallback(std_msgs::msg::Bool::SharedPtr bMsg);
    

    // Publishers
    rclcpp::Publisher<orbslam3_interfaces::msg::KeyFrame>::SharedPtr keyframe_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Map>::SharedPtr map_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::MapPoint>::SharedPtr map_point_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Atlas>::SharedPtr atlas_publisher_;  
    
    rclcpp::Publisher<orbslam3_interfaces::msg::Bool>::SharedPtr lm_active_publisher_;  
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lm_reset_requested_publisher_;  
    rclcpp::Publisher<orbslam3_interfaces::msg::Int64>::SharedPtr sys_reset_active_map_publisher_;  
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr step_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr worker_publisher_;  
    

    // Subscribers
    rclcpp::Subscription<orbslam3_interfaces::msg::KeyFrame>::SharedPtr m_keyframe_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::Map>::SharedPtr m_map_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::MapPoint>::SharedPtr m_map_point_subscriber_;
    rclcpp::Subscription<orbslam3_interfaces::msg::Atlas>::SharedPtr m_atlas_subscriber_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_lm_reset_requested_subscriber_;     
    rclcpp::Subscription<orbslam3_interfaces::msg::Bool>::SharedPtr m_lm_active_subscriber_;     
    rclcpp::Subscription<orbslam3_interfaces::msg::Int64>::SharedPtr m_sys_reset_active_map_subscriber_;     
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr worker_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr end_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr step_subscriber_;

    // Store SLAM data
    std::map<long unsigned int, std::tuple<ORB_SLAM3::Map*, orbslam3_interfaces::msg::Map::SharedPtr>> mpUnprocOrbMaps;
    std::map<long unsigned int, orbslam3_interfaces::msg::KeyFrame::SharedPtr> mpUnprocOrbKeyFrames;
    std::map<std::string, std::tuple<ORB_SLAM3::MapPoint*, orbslam3_interfaces::msg::MapPoint::SharedPtr>> mpUnprocOrbMapPoints;
    
    std::map<long unsigned int, ORB_SLAM3::Map*> mORBMaps;
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mORBKeyFrames;
    std::map<std::string, ORB_SLAM3::MapPoint*> mORBMapPoints;
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mORBCameras;

    std::set<unsigned long int> mspKFsReadyForLM;
    std::set<unsigned long int> mspUnprocKFids;
      
    std::set<ORB_SLAM3::KeyFrame*> mspKeyFrames;
    std::set<ORB_SLAM3::MapPoint*> mspMapPoints;
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras;

    std::vector<unsigned long int> mvNewKFIds;
    
    bool mbLocalMappingIsIdle;

};

#endif
