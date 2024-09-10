#ifndef DISTRIBUTOR_KEYFRAME_SUBSCRIBER_H
#define DISTRIBUTOR_KEYFRAME_SUBSCRIBER_H

#include "System.h"
#include "../Converter/Converter.hpp"
#include "../Converter/KeyFrameConverter.hpp"
//#include "Frame.h"
//#include "KeyFrame.h"
//#include "Map.h"
//#include "MapPoint.h"
//#include "Tracking.h"
//#include "Converter.h"
//#include "KeyFrameDatabase.h"
//#include "GeometricCamera.h"
//#include "../slam/slam-wrapper-node.hpp"
//#include "./Observer.hpp"
#include <stdlib.h>
#include <mutex>

//namespace TempDistributor
//{

class Observer;

class KeyFrameSubscriber 
{
  public:  
    KeyFrameSubscriber();
    ~KeyFrameSubscriber();

    // main function
    void Run();

    void InsertNewKeyFrame(orbslam3_interfaces::msg::KeyFrame::SharedPtr pRosKF);
    void InsertNewKeyFrame(orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr& pRosKFUpdate);
    
    int KeyFramesInQueue();
    int KeyFrameUpdatesInQueue();

    void RequestFinish();
    bool isFinished();

    void ResetQueue(const bool bAll);

    void AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM);
    void AttachObserver(std::shared_ptr<Observer> pObserver);

    // Processing times ROS->ORB, include postload etc.
    vector<double> vdRos2OrbProcKF_ms;
    vector<double> vdRos2OrbConvKF_ms;
    vector<double> vdRos2OrbConvMP_ms;
    vector<double> vdPostLoadKF_ms;
    vector<double> vdPostLoadMP_ms;
    vector<double> vdInjectMP_ms;
    vector<double> vdInjectKF_ms;
    vector<double> vdPrepDataKF_ms;;

    vector<int> vnKFAmount;
    vector<int> vnMPAmount;

    vector<int> vnNewMPAmount;
    vector<int> vnUpdateMPAmount;

    vector<std::chrono::system_clock::time_point> vtTimes;

  protected:  

    bool CheckNewKeyFrames();
    bool CheckNewKeyFrameUpdates();
    
    void ProcessNewKeyFrame();
    void ProcessNewKeyFrameUpdate();
    
    std::vector<ORB_SLAM3::KeyFrame*> mlpReadyKeyFrames;
    std::vector<ORB_SLAM3::MapPoint*> mlpReadyMapPoints;
    std::list<orbslam3_interfaces::msg::KeyFrame::SharedPtr> mlpRosKeyFrameQueue;
    std::list<orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr> mlpRosKeyFrameUpdateQueue;
    std::map<unsigned long int, orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr> mpRosKeyFrameUpdateQueue;
    std::mutex mMutexNewRosKFs;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;


  private:  
    ORB_SLAM3::System* pSLAM;
    ORB_SLAM3::Tracking* mpTracker;
    ORB_SLAM3::LocalMapping* mpLocalMapper;
    ORB_SLAM3::LoopClosing* mpLoopCloser;
    ORB_SLAM3::Atlas* mpAtlas;    
    ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB; 
    
    std::shared_ptr<Observer> mpObserver;

};
//§}

#endif // DISTRIBUTOR_DISTRIBUTOR_KEYFRAME_SUBSCRIBER_H
