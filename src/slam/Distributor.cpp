
#include "./Distributor.hpp"
#include "./slam-wrapper-node.hpp"


Distributor::Distributor():
    mbFinishRequested(false), mbFinished(true), mbMapUpdateRunning(false)
{
    char* systemId = std::getenv("SLAM_SYSTEM_ID");
    
    if(string(systemId) == "TR") {
        mnTaskModule = 1;
    } else if (string(systemId) == "LM") {
        mnTaskModule = 2;
    } else if (string(systemId) == "LC") {
        mnTaskModule = 3;
    } else {
        mnTaskModule = 0;
    }

}

Distributor::~Distributor()
{
    slam_node_ = nullptr;
}

// Main loop
void Distributor::Run()
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
        slam_node_->publishKeyFrame(mRosKF);
      }
      
      // second check if there is new maps
      if(!IsMapUpdateRunning() && CheckNewLocalMaps())
      {
        unique_lock<mutex> lock(mMutexMapUpdate);

        mbMapUpdateRunning = true;
        mpThreadMapUpdate = new thread(&Distributor::ProcessNewLocalMap, this);
        //orbslam3_interfaces::msg::Map mRosMap = ProcessNewLocalMap();
        //slam_node_->publishMap(mRosMap);
      }

      // First check global map updates (Atlas)
      if(!IsMapUpdateRunning() && CheckNewGlobalMaps())
      {
        unique_lock<mutex> lock(mMutexMapUpdate);
        mbMapUpdateRunning = true;
        mpThreadMapUpdate = new thread(&Distributor::ProcessNewGlobalMap, this);
      }


      usleep(1000);
      if(CheckFinish())
          break;
    }

    SetFinish();
}


void Distributor::ProcessNewGlobalMap()
{
    unique_lock<mutex> lock1(mMutexMapUpdate);
    std::cout << "Processing new Global Map" << std::endl;
    std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate;
    {
        unique_lock<mutex> lock2(mMutexNewAtlas);
        mtAtlasUpdate = mlpAtlasQueue.front();
        mlpAtlasQueue.pop_front();
    }

    
    ORB_SLAM3::Map* mpCurrentMap = mpAtlas->GetCurrentMap();
    
    // Clear all updated data since we want to publish all KFs
    mpCurrentMap->ClearUpdatedKFIds();
    mpCurrentMap->ClearUpdatedMPIds();
    
    std::vector<ORB_SLAM3::GeometricCamera*> mvpCameras = mpAtlas->GetAllCameras();
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras(mvpCameras.begin(), mvpCameras.end());
    
    mpCurrentMap->PreSave(mspCameras);
    
    // Create ros msg and convert map to ros
    orbslam3_interfaces::msg::Atlas mRosAtlas;
    mRosAtlas.mp_current_map = Converter::MapConverter::OrbMapToRosMap(mpCurrentMap);

    mRosAtlas.system_id = std::getenv("SLAM_SYSTEM_ID");
    mRosAtlas.from_module_id = mnTaskModule;
    mRosAtlas.mb_map_merge = std::get<0>(mtAtlasUpdate);
    mRosAtlas.mb_loop_closer = std::get<1>(mtAtlasUpdate);
    mRosAtlas.mv_merged_map_ids = std::get<2>(mtAtlasUpdate);
    
    slam_node_->publishAtlas(mRosAtlas);
    mbMapUpdateRunning = false;
    StopMapThread();
    //return mRosAtlas;
}

void Distributor::ProcessNewLocalMap()
{
    unique_lock<mutex> lock1(mMutexMapUpdate);
    std::cout << "Processing new Local Map" << std::endl;
    ORB_SLAM3::Map* pMap = static_cast<ORB_SLAM3::Map*>(NULL);
    {
        unique_lock<mutex> lock2(mMutexNewMaps);
        pMap = mlpMapQueue.front();
        mlpMapQueue.pop_front();
    }

    std::vector<ORB_SLAM3::GeometricCamera*> mvpCameras = mpAtlas->GetAllCameras();
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras(mvpCameras.begin(), mvpCameras.end());
    
    pMap->PreSave(mspCameras);
    

    orbslam3_interfaces::msg::Map mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap); 
    mRosMap.from_module_id = mnTaskModule; 
    
    pMap->ClearErasedData();
    pMap->ClearUpdatedKFIds();
    pMap->ClearUpdatedMPIds();

    slam_node_->publishMap(mRosMap);
    //usleep(10000);
    mbMapUpdateRunning = false;
    StopMapThread();
    //return mRosMap;
}

orbslam3_interfaces::msg::KeyFrame Distributor::ProcessNewKeyFrame()
{
    std::cout << "Processing new KF" << std::endl;
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
    mRosKF.from_module_id = mnTaskModule;
    
    return mRosKF;
}


int Distributor::GetWorkerNumber()
{
   unique_lock<std::mutex> lock(mMutexWorker); 
   return mspWorkers.size();
}

bool Distributor::CheckIfWorkerExists(unsigned int mnTargetModule)
{
   unique_lock<std::mutex> lock(mMutexWorker); 
   return mspWorkers.count(mnTargetModule);
}

void Distributor::AddNewWorker(unsigned int mnWorkerModule)
{
   unique_lock<std::mutex> lock(mMutexWorker); 
   mspWorkers.insert(mnWorkerModule);
}

int Distributor::KeyFramesInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlpKeyFrameQueue.size();
}

int Distributor::MapsInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewMaps);
    return mlpMapQueue.size();
}

void Distributor::InsertNewLocalMap(ORB_SLAM3::Map* pMap)
{
    unique_lock<mutex> lock(mMutexNewMaps);
    mlpMapQueue.push_back(pMap);
}

void Distributor::InsertNewGlobalMap(std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate)
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    mlpAtlasQueue.push_back(mtAtlasUpdate);
}

void Distributor::InsertNewKeyFrame(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlpKeyFrameQueue.push_back(pKF);
    mlKeyFrameTargetModule.push_back(mnTargetModule);
}

bool Distributor::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlpKeyFrameQueue.empty());
}


bool Distributor::CheckNewLocalMaps()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    return(!mlpMapQueue.empty());
}

bool Distributor::CheckNewGlobalMaps()
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    return(!mlpAtlasQueue.empty());
}

void Distributor::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Distributor::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Distributor::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    //unique_lock<mutex> lock2(mMutexStop);
    //mbStopped = true;
}

bool Distributor::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

// Override from ORBSLAM3
void Distributor::onActiveMapReset(unsigned long int mnMapId)
{
    if(slam_node_)
        slam_node_->publishResetActiveMap(mnMapId);
}

void Distributor::onLMResetRequested()
{
    if(slam_node_)
        slam_node_->publishLMResetRequested();
}

void Distributor::onChangeLMActive(bool bActive)
{
    if(slam_node_)
        slam_node_->publishLMActivityChange(bActive);
}

void Distributor::onKeyframeAdded(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule)
{
    //InsertNewKeyFrame(pKF, mnTargetModule);
    if(CheckIfWorkerExists(mnTargetModule) && slam_node_)
    {
        std::cout << "Worker exists, sending it to the device." << std::endl;
        InsertNewKeyFrame(pKF, mnTargetModule);
    } else if (GetWorkerNumber() > 1) {
        if(mnTargetModule==2)
            mpLocalMapper->InsertKeyFrame(pKF);
        else if(mnTargetModule==3)
        {
            InsertNewKeyFrame(pKF, mnTargetModule);
            mpLoopCloser->InsertKeyFrame(pKF);
        }

    }
    else if(GetWorkerNumber() <= 1) {
        std::cout << "Worker does not exist, Keeping it here." << std::endl;

        if(mnTargetModule==2)
            mpLocalMapper->InsertKeyFrame(pKF);
        else if(mnTargetModule==3)
            mpLoopCloser->InsertKeyFrame(pKF);
    } 
}

void Distributor::onLocalMapUpdated(ORB_SLAM3::Map* pM)
{
    if(GetWorkerNumber() > 1)
        InsertNewLocalMap(pM);
}

void Distributor::onGlobalMapUpdated(bool mbMerged, bool mbLoopClosure, std::vector<unsigned long int> mvMeergedIds)
{
    if(GetWorkerNumber() > 1)
    {
        std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate = std::tuple<bool, bool, std::vector<unsigned long int>>(mbMerged, mbLoopClosure, mvMeergedIds);
        InsertNewGlobalMap(mtAtlasUpdate);
    } 
}

void Distributor::AttachSLAMSystem(ORB_SLAM3::System* mSLAM)
{
    pSLAM = mSLAM;
    
    mpTracker = pSLAM->GetTrackerPtr();
    mpLocalMapper = pSLAM->GetMapperPtr();
    mpLoopCloser = pSLAM->GetLoopClosingPtr();
    mpAtlas = pSLAM->GetAtlas();
    mpKeyFrameDB = pSLAM->GetKeyFrameDatabase();
}

void Distributor::AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node) {
    slam_node_ = slam_node;
}

void Distributor::deleteSlamNode(std::shared_ptr<SlamWrapperNode> slam_node) {
    slam_node_.reset();
}
