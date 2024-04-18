#ifndef DISTRIBUTOR_KEYFRAME_PUBLISHER_H
#define DISTRIBUTOR_KEYFRAME_PUBLISHER_H

#include "orbslam3_interfaces/Converter.hpp"
#include "orbslam3_interfaces/KeyFrameConverter.hpp"
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

    void InsertNewKeyFrame(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule);
    
    int KeyFramesInQueue();

    void RequestFinish();
    bool isFinished();

    void AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM);
    void AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node);
    void AttachObserver(std::shared_ptr<Observer> pObserver);
    int mnTaskModule;

  protected:  
    bool CheckNewKeyFrames();
    orbslam3_interfaces::msg::KeyFrame ProcessNewKeyFrame();

    std::list<ORB_SLAM3::KeyFrame*> mlpKeyFrameQueue;
    std::list<unsigned int> mlKeyFrameTargetModule;
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
