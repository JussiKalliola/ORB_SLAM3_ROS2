#include "./KeyFrameSubscriber.hpp"
#include "./Observer.hpp"
//#include "../slam/slam-wrapper-node.hpp"

//namespace TempDistributor {

KeyFrameSubscriber::KeyFrameSubscriber()
{

}

KeyFrameSubscriber::~KeyFrameSubscriber()
{

}

// main function
void KeyFrameSubscriber::Run()
{
    mbFinished = false;
    
    while(1)
    {
      // Then new keyframes
      // Here, write some logic which checks if the updated KF was in map
      // if so, do not publish again.
      // second check if there is new maps
      if(CheckNewKeyFrames())
      {
        ProcessNewKeyFrame();
      }

      usleep(1000);
      if(CheckFinish())
          break;
    }

    SetFinish();

}

bool KeyFrameSubscriber::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewRosKFs);
    return(!mlpRosKeyFrameQueue.empty());
}

int KeyFrameSubscriber::KeyFramesInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewRosKFs);
    return mlpRosKeyFrameQueue.size();
}

void KeyFrameSubscriber::ProcessNewKeyFrame()
{
    std::cout << "KeyFrameSubscriber :: Processing new KF, KFs in Queue=" << KeyFramesInQueue() << std::endl;
    orbslam3_interfaces::msg::KeyFrame::SharedPtr pRosKF = static_cast<orbslam3_interfaces::msg::KeyFrame::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewRosKFs);
        pRosKF = mlpRosKeyFrameQueue.front();

        mlpRosKeyFrameQueue.pop_front();
    }
  
    if(!pRosKF)
        return;
    
    // Get maps and the map where KF is
    ORB_SLAM3::Map* pCurrentMap = static_cast<ORB_SLAM3::Map*>(NULL);
    std::map<unsigned long int, ORB_SLAM3::Map*> mMaps;
    for(ORB_SLAM3::Map* pM : mpAtlas->GetAllMaps())
    {
        if(pM->GetId() == pRosKF->mp_map_id)
        {
            pCurrentMap = pM;
        }
        mMaps[pM->GetId()] = pM;
    }

    if(!pCurrentMap)
    {
        mpAtlas->CreateNewMap();
        pCurrentMap = mpAtlas->GetCurrentMap(); 
        mMaps[pCurrentMap->GetId()] = pCurrentMap;
    }

    // Create map data structures for postloads
    std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    std::map<std::string, ORB_SLAM3::MapPoint*> mMapMPs; 
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mFusedKFs; 
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    
    for(ORB_SLAM3::KeyFrame* pKF : pCurrentMap->GetAllKeyFrames())
    {
      mFusedKFs[pKF->mnId] = pKF;
    }

    for(ORB_SLAM3::MapPoint* pMP : pCurrentMap->GetAllMapPoints())
    {
      mFusedMPs[pMP->mstrHexId] = pMP;
      mMapMPs[pMP->mstrHexId] = pMP;
    }

    for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas->GetAllCameras())
    {
      mCameras[pCam->GetId()] = pCam;
    }
    
    std::cout << "Got a new keyframe. 1. Existing data is collected." << std::endl;
    
    // Convert ros msg to ORB_SLAM3 Objects
    ORB_SLAM3::KeyFrame* tempKF = mpObserver->ConvertKeyFrame(pRosKF, mMaps);
    std::vector<ORB_SLAM3::MapPoint*> mvpTempMPs;
    mvpTempMPs.resize(pRosKF->mvp_map_points.size());
    
    for(int i=0; i<mvpTempMPs.size(); ++i)
    {
        mvpTempMPs[i] = mpObserver->ConvertMapPoint(std::make_shared<orbslam3_interfaces::msg::MapPoint>(pRosKF->mvp_map_points[i]), mMaps);
        if(mFusedMPs.find(mvpTempMPs[i]->mstrHexId) == mFusedMPs.end())
            mFusedMPs[mvpTempMPs[i]->mstrHexId] = mvpTempMPs[i];
    }
    
    std::cout << "Got a new keyframe. 2. New data is fused with old data." << std::endl;

    // Postloads -> Reassign pointers etc to temporary data
    bool bKFUnprocessed = false;
    tempKF->PostLoad(mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);

    // -------------------- From here on we have unprocessed data -----------------------
    // Here the data is injected into existing KF or the same one is returned. Function handles pointers etc.
    ORB_SLAM3::KeyFrame* pKF = mpObserver->InjectKeyFrame(tempKF, mFusedKFs);
    mFusedKFs[pKF->mnId] = pKF;
    
    std::cout << "Got a new keyframe. 3. KF PostLoad, and injection is completed." << std::endl;
    
    for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
        tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    }
    
    std::cout << "Got a new keyframe. 4. All MP PostLoads Completed." << std::endl;
    
    for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
        mpObserver->InjectMapPoint(tempMP, mMapMPs);
    }
    // -------------------- To here -----------------------
    
    std::cout << "Got a new keyframe. 5. All MPs are injected to ORB_SLAM3." << std::endl;
    
    
    // If KF exists, forward it to target
    if(pKF)
      mpObserver->ForwardKeyFrameToTarget(pKF, pRosKF->target, pRosKF->from);
    
    std::cout << "Got a new keyframe. 6. KF is forwarded to the target. KF is COMPLETE." << std::endl;

}


void KeyFrameSubscriber::InsertNewKeyFrame(orbslam3_interfaces::msg::KeyFrame::SharedPtr pRosKF)
{
    unique_lock<mutex> lock(mMutexNewRosKFs);
    mlpRosKeyFrameQueue.push_back(pRosKF);
    std::cout << "KeyFrameSubscriber :: Got new keyframe." << std::endl;
}

void KeyFrameSubscriber::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool KeyFrameSubscriber::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void KeyFrameSubscriber::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    //unique_lock<mutex> lock2(mMutexStop);
    //mbStopped = true;
}

bool KeyFrameSubscriber::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void KeyFrameSubscriber::AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM)
{
    pSLAM = mSLAM;

    mpTracker = pSLAM->GetTrackerPtr();
    mpLocalMapper = pSLAM->GetMapperPtr();
    mpLoopCloser = pSLAM->GetLoopClosingPtr();
    mpAtlas = pSLAM->GetAtlas();
    mpKeyFrameDB = pSLAM->GetKeyFrameDatabase();
}
  
void KeyFrameSubscriber::AttachObserver(std::shared_ptr<Observer> pObserver)
{
  mpObserver = pObserver; 
}
//}
