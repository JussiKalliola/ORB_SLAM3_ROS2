#include "./KeyFramePublisher.hpp"
#include "./Observer.hpp"
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
  //std::cout << "KeyFramePublisher Thread has been started." << std::endl;
  
  while(1)
  {
    // Then new keyframes
    // Here, write some logic which checks if the updated KF was in map
    // if so, do not publish again.
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

void KeyFramePublisher::ProcessNewKeyFrame()
{
    //std::cout << "KeyFramePublisher :: Processing new KF, KFs in Queue=" << KeyFramesInQueue() << std::endl;
    ORB_SLAM3::KeyFrame* pKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        pKF = mlpKeyFrameQueue.front();
        mlpKeyFrameQueue.pop_front();
    }
    
    // Start of a timer
    std::chrono::steady_clock::time_point time_StartOrb2RosProcKF = std::chrono::steady_clock::now();
    vtTimes.push_back(time_StartOrb2RosProcKF);

    // Here KF Might not hve map for some reason, handle that
    //std::map<unsigned long int, ORB_SLAM3::KeyFrame*> mFusedKFs = mpObserver->GetAllKeyFrames();
    //std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs = mpObserver->GetAllMapPoints();

    std::set<ORB_SLAM3::KeyFrame*> mspKeyFrames;
    for (const auto& tempKF : mpAtlas->GetAllKeyFrames()) {
        mspKeyFrames.insert(tempKF);
    }    

    std::set<ORB_SLAM3::MapPoint*> mspMapPoints;
    for (const auto& tempMP : mpAtlas->GetAllMapPoints()) {
        mspMapPoints.insert(tempMP);
    }    

    std::vector<ORB_SLAM3::GeometricCamera*> mvpCameras = mpAtlas->GetAllCameras();
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras(mvpCameras.begin(), mvpCameras.end());
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPreSaveKF = std::chrono::steady_clock::now();

    pKF->PreSave(mspKeyFrames, mspMapPoints, mspCameras);

    // End of timer
    std::chrono::steady_clock::time_point time_EndPreSaveKF = std::chrono::steady_clock::now();
    double timePreSaveKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreSaveKF - time_StartPreSaveKF).count();
    vdPreSaveKF_ms.push_back(timePreSaveKF);

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPreSaveMP = std::chrono::steady_clock::now();

    if(!msUpdatedMPs.empty())
    {
        for(ORB_SLAM3::MapPoint* pMP : pKF->GetMapPoints())
        {
            pMP->PreSave(mspKeyFrames, mspMapPoints);
        }

    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndPreSaveMP = std::chrono::steady_clock::now();
    double timePreSaveMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreSaveMP - time_StartPreSaveMP).count();
    vdPreSaveMP_ms.push_back(timePreSaveMP);
    
    

    //std::cout << "KFPublisher :: mspUpdatedMapPoints.size()=" << msUpdatedMPs.size() << ", #MPs in KF=" << pKF->GetNumberMPs() << std::endl;
    if(pKF->mnNextTarget == 2)
    {
        // Start of a timer -------------
        std::chrono::steady_clock::time_point time_StartOrb2RosConvKF = std::chrono::steady_clock::now();

        orbslam3_interfaces::msg::KeyFrame::SharedPtr mRosKF;
        mRosKF = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROS(pKF, msUpdatedMPs, false);
        


        // End of timer
        std::chrono::steady_clock::time_point time_EndOrb2RosConvKF = std::chrono::steady_clock::now();
        double timeOrb2RosConvKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOrb2RosConvKF - time_StartOrb2RosConvKF).count();
        vdOrb2RosConvKF_ms.push_back(timeOrb2RosConvKF);

        {
          unique_lock<mutex>(mMutexNewKFs);
          msUpdatedMPs.clear();
        }

        pKF->mnNextTarget = 0;
        mRosKF->from_module_id = mpObserver->GetTaskModule();

        // End of timer
        std::chrono::steady_clock::time_point time_EndOrb2RosProcKF = std::chrono::steady_clock::now();
        double timeOrb2RosProcKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOrb2RosProcKF - time_StartOrb2RosProcKF).count();
        vdOrb2RosProcKF_ms.push_back(timeOrb2RosProcKF);
        
        pSLAMNode->publishKeyFrame(mRosKF);
    } 
    else {

        // Start of a timer -------------
        std::chrono::steady_clock::time_point time_StartOrb2RosConvKF = std::chrono::steady_clock::now();

        {
          unique_lock<mutex>(mMutexNewKFs);
          msUpdatedMPs.clear();
        }

        orbslam3_interfaces::msg::KeyFrameUpdate mRosKFUpdate;
        mRosKFUpdate = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROSKeyFrameUpdate(pKF, msUpdatedMPs, true);
        


        // End of timer
        std::chrono::steady_clock::time_point time_EndOrb2RosConvKF = std::chrono::steady_clock::now();
        double timeOrb2RosConvKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOrb2RosConvKF - time_StartOrb2RosConvKF).count();
        vdOrb2RosConvKF_ms.push_back(timeOrb2RosConvKF);


        pKF->mnNextTarget = 0;
        mRosKFUpdate.from_module_id = mpObserver->GetTaskModule();

        // End of timer
        std::chrono::steady_clock::time_point time_EndOrb2RosProcKF = std::chrono::steady_clock::now();
        double timeOrb2RosProcKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOrb2RosProcKF - time_StartOrb2RosProcKF).count();
        vdOrb2RosProcKF_ms.push_back(timeOrb2RosProcKF);
        
        pSLAMNode->publishKeyFrame(mRosKFUpdate);
    }
}


int KeyFramePublisher::KeyFramesInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlpKeyFrameQueue.size();
}


void KeyFramePublisher::InsertNewKeyFrame(ORB_SLAM3::KeyFrame* pKF, std::set<std::string> msNewMapPointIds)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    std::cout << "***** SEND KF ******" << std::endl;
    std::set<std::string> msErasedMPIds = pKF->GetMap()->GetErasedMPIds();

    for(const auto& mStrId : msNewMapPointIds)
    {
        if(msErasedMPIds.count(mStrId))
            continue;
        msUpdatedMPs.insert(mStrId);
    }
    //msUpdatedMPs.insert(msNewMapPointIds.begin(), msNewMapPointIds.end());
    //pKF->GetMap()->ClearUpdatedMPIds(); 

    mlpKeyFrameQueue.push_back(pKF);
}

void KeyFramePublisher::InsertNewKeyFrame(ORB_SLAM3::KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    std::cout << "***** SEND KF ******" << std::endl;

    std::set<std::string> mspUpdatedMapPoints = pKF->GetMap()->GetUpdatedMPIds();


    msUpdatedMPs.insert(mspUpdatedMapPoints.begin(), mspUpdatedMapPoints.end());
    //pKF->GetMap()->ClearUpdatedMPIds(); 

    mlpKeyFrameQueue.push_back(pKF);
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
