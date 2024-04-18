#ifndef DISTRIBUTOR_KEYFRAME_SUBSCRIBER_H
#define DISTRIBUTOR_KEYFRAME_SUBSCRIBER_H

#include "System.h"
#include "orbslam3_interfaces/Converter.hpp"
#include "orbslam3_interfaces/KeyFrameConverter.hpp"
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
    
    int KeyFramesInQueue();

    void RequestFinish();
    bool isFinished();

    void AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM);
    void AttachObserver(std::shared_ptr<Observer> pObserver);

  protected:  

    bool CheckNewKeyFrames();
    
    void ProcessNewKeyFrame();
    
    std::list<orbslam3_interfaces::msg::KeyFrame::SharedPtr> mlpRosKeyFrameQueue;
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
