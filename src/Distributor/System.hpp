#ifndef DISTRIBUTOR_SYSTEM_H
#define DISTRIBUTOR_SYSTEM_H



#include "System.h"
//#include "Frame.h"
//#include "KeyFrame.h"
//#include "Map.h"
//#include "MapPoint.h"
//#include "Tracking.h"
//#include "Converter.h"
//#include "KeyFrameDatabase.h"
//#include "GeometricCamera.h"
//
//#include "../slam/slam-wrapper-node.hpp"
//#include "./Observer.hpp"
//#include "./MapHandler.hpp"
//#include "./KeyFramePublisher.hpp"
//#include "./KeyFrameSubscriber.hpp"

#include <stdlib.h>
//namespace TempDistributor
//{

class MapHandler;
class KeyFramePublisher;
class KeyFrameSubscriber;
class Tracker;
class SlamWrapperNode;
class Observer;

class System
{
  public:  
    System(std::string mStatSavePath);
    ~System();

    void AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM);
    void AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node);
    void LaunchThreads();
      
    std::shared_ptr<Observer> GetObserver();

    MapHandler* GetMapHandler();
    KeyFrameSubscriber* GetKeyFrameSubscriber();
    KeyFramePublisher* GetKeyFramePublisher();
    
    void ShutDown();

    void TrackStats2File();
    void LoopClosingStats2File();
    void LocalMapStats2File();
    void TimeStats2File();
    void SystemStats2File();
    void PrintTimeStats();


  protected:  
    ORB_SLAM3::System* pSLAM;
    ORB_SLAM3::Tracking* mpTracker;
    ORB_SLAM3::LocalMapping* mpLocalMapper;
    ORB_SLAM3::LoopClosing* mpLoopCloser;

    std::shared_ptr<SlamWrapperNode> pSLAMNode;

    std::string mStrStatSavePath;

    std::mutex mMutexTime;
    std::chrono::steady_clock::time_point time_GlobalSystemStart;


  private:  
    // Map handler. Handles subscriber callbacks and publishing local and global maps 
    // Not time critical, so everything is handled in the same thread.
    // Can be separated into multiple threads if needed.
    MapHandler* mpMapHandler;

    // KeyFrame publisher. KeyFrames are time critical so publish/subscription needs
    // to work immidiatley without delays. Handles conversions and other processing
    // related to publishing keyframes.
    KeyFramePublisher* mpKeyFramePublisher;

    // KeyFrame subscriber. Handles KF subscription callbacks in its own thread.
    // Time critical.
    KeyFrameSubscriber* mpKeyFrameSubscriber;

    //Observer. ORB_SLAM3 uses observer to advertise new data. Observer handles
    // routing of the data: if there is no worker for task, keep it on the machine
    // otherwise send to ros network
    std::shared_ptr<Observer> mpObserver;

    // system Tracker. Keeps track of CPU, memory etc. utilization. working in its own thread. 
    Tracker* mpSystemTracker;

    // System threads: KF Publisher, KF Subscriber, Map Handler (sub+pub)
    std::thread* mptKeyFramePublisher;
    std::thread* mptKeyFrameSubscriber;
    std::thread* mptMapHandler;
    std::thread* mptSystemTracker;
};
//}

#endif // DISTRIBUTOR_SYSTEM_H
