#include "./KeyFramePublisher.hpp"
//#include "./Observer.hpp"
#include "../slam/slam-wrapper-node.hpp"

//namespace TempDistributor {

KeyFramePublisher::KeyFramePublisher():
  mbFinishRequested(false), mbFinished(true)
{

}


KeyFramePublisher::~KeyFramePublisher()
{

}

// main function
void KeyFramePublisher::Run()
{
  mbFinished = false;
  
  while(1)
  {
    // Then new keyframes
    // Here, write some logic which checks if the updated KF was in map
    // if so, do not publish again.
    if(CheckNewKeyFrames())
    {
      orbslam3_interfaces::msg::KeyFrame mRosKF = ProcessNewKeyFrame();
      pSLAMNode->publishKeyFrame(mRosKF);
    }

    usleep(1000);
    if(CheckFinish())
        break;
  }

  SetFinish();
}

orbslam3_interfaces::msg::KeyFrame KeyFramePublisher::ProcessNewKeyFrame()
{
    std::cout << "KeyFramePublisher :: Processing new KF, KFs in Queue=" << KeyFramesInQueue() << std::endl;
    ORB_SLAM3::KeyFrame* pKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
    int mnTargetModule = 0;
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        pKF = mlpKeyFrameQueue.front();
        mnTargetModule = mlKeyFrameTargetModule.front();

        mlpKeyFrameQueue.pop_front();
        mlKeyFrameTargetModule.pop_front();
    }
    
    // Here KF Might not hve map for some reason, handle that
    std::vector<ORB_SLAM3::KeyFrame*> mvpKeyFrames = pKF->GetMap()->GetAllKeyFrames();
    std::vector<ORB_SLAM3::MapPoint*> mvpMapPoints = pKF->GetMap()->GetAllMapPoints();
    std::vector<ORB_SLAM3::GeometricCamera*> mvpCameras = mpAtlas->GetAllCameras();
    
    std::set<ORB_SLAM3::KeyFrame*> mspKeyFrames(mvpKeyFrames.begin(), mvpKeyFrames.end());
    std::set<ORB_SLAM3::MapPoint*> mspMapPoints(mvpMapPoints.begin(), mvpMapPoints.end());
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras(mvpCameras.begin(), mvpCameras.end());
    
    pKF->PreSave(mspKeyFrames, mspMapPoints, mspCameras);

    for(ORB_SLAM3::MapPoint* pMP : pKF->GetMapPoints())
    {
        pMP->PreSave(mspKeyFrames, mspMapPoints);
    }
    
    orbslam3_interfaces::msg::KeyFrame mRosKF = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKF);
    mRosKF.target = mnTargetModule;
    mRosKF.from = mnTaskModule;
    
    return mRosKF;
}


int KeyFramePublisher::KeyFramesInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlpKeyFrameQueue.size();
}



void KeyFramePublisher::InsertNewKeyFrame(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlpKeyFrameQueue.push_back(pKF);
    mlKeyFrameTargetModule.push_back(mnTargetModule);
}

bool KeyFramePublisher::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlpKeyFrameQueue.empty());
}



void KeyFramePublisher::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool KeyFramePublisher::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void KeyFramePublisher::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    //unique_lock<mutex> lock2(mMutexStop);
    //mbStopped = true;
}

bool KeyFramePublisher::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}
void KeyFramePublisher::AttachObserver(std::shared_ptr<Observer> pObserver)
{
  mpObserver = pObserver; 
}

void KeyFramePublisher::AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM)
{
    pSLAM = mSLAM;
    mpTracker = pSLAM->GetTrackerPtr();
    mpLocalMapper = pSLAM->GetMapperPtr();
    mpLoopCloser = pSLAM->GetLoopClosingPtr();
    mpAtlas = pSLAM->GetAtlas();
    mpKeyFrameDB = pSLAM->GetKeyFrameDatabase();
}

void KeyFramePublisher::AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node)
{
    pSLAMNode = slam_node;
}
//}
