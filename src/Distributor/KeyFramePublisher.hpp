#ifndef DISTRIBUTOR_KEYFRAME_PUBLISHER_H
#define DISTRIBUTOR_KEYFRAME_PUBLISHER_H

#include "../Converter/Converter.hpp"
#include "../Converter/KeyFrameConverter.hpp"
//#include "../slam/slam-wrapper-node.hpp"
#include "System.h"
//#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
//#include "Tracking.h"
//#include "Converter.h"
//#include "KeyFrameDatabase.h"
#include "GeometricCamera.h"

//#include "./Observer.hpp"
#include <stdlib.h>
#include <mutex>

//namespace TempDistributor
//{
class Observer;
class SlamWrapperNode;

class KeyFramePublisher 
{
  public:  
    KeyFramePublisher();
    ~KeyFramePublisher();

    // main function
    void Run();

    void InsertNewKeyFrame(ORB_SLAM3::KeyFrame* pKF);
    void InsertNewKeyFrame(ORB_SLAM3::KeyFrame* pKF, std::set<std::string> msNewMapPointIds);
    
    int KeyFramesInQueue();

    void ResetQueue();
    void RequestFinish();
    bool isFinished();

    void AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM);
    void AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node);
    void AttachObserver(std::shared_ptr<Observer> pObserver);
    int mnTaskModule;

    // Processing times ROS->ORB, include postload etc.
    vector<double> vdPreSaveMP_ms;
    vector<double> vdPreSaveKF_ms;
    vector<double> vdOrb2RosProcKF_ms;
    vector<double> vdOrb2RosConvKF_ms;
    vector<std::chrono::steady_clock::time_point> vtTimes;

  protected:  
    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    
    std::set<std::string> msUpdatedMPs;

    std::list<ORB_SLAM3::KeyFrame*> mlpKeyFrameQueue;
    std::mutex mMutexNewKFs;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
    

  private:  
    // ORB_SLAM3
    ORB_SLAM3::System* pSLAM;
    ORB_SLAM3::Tracking* mpTracker;
    ORB_SLAM3::LocalMapping* mpLocalMapper;
    ORB_SLAM3::LoopClosing* mpLoopCloser;
    ORB_SLAM3::Atlas* mpAtlas;    
    ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB; 

    std::shared_ptr<SlamWrapperNode> pSLAMNode;

    std::shared_ptr<Observer> mpObserver;
};
//}

#endif // DISTRIBUTOR_KEYFRAME_PUBLISHER_H
