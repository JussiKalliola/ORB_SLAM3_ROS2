#include "./Observer.hpp"

//#include "./KeyFramePublisher.hpp"
#include "../slam/slam-wrapper-node.hpp"
#include <chrono>

//namespace TempDistributor
//{

Observer::Observer(MapHandler* pMapHandler, KeyFramePublisher* pKeyFramePublisher, KeyFrameSubscriber* pKeyFrameSubscriber)
  : mdSinceReset(0.0)
{
    // init task
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
    
    UpdateLastResetTime();

    // Attach handlers internally
    mpKeyFramePublisher = pKeyFramePublisher;
    mpKeyFrameSubscriber = pKeyFrameSubscriber;
    mpMapHandler = pMapHandler;
}

Observer::~Observer()
{

}


void Observer::AddMapPoint(ORB_SLAM3::MapPoint* pMP)
{
    unique_lock<std::mutex> lock(mMutexMapPoint); 
    mOrbMapPoints[pMP->mstrHexId] = pMP;
}

void Observer::EraseMapPoint(ORB_SLAM3::MapPoint* pMP)
{
    unique_lock<std::mutex> lock(mMutexMapPoint); 
    mOrbMapPoints.erase(pMP->mstrHexId);
}

bool Observer::CheckIfMapPointExists(ORB_SLAM3::MapPoint* pMP)
{
    unique_lock<std::mutex> lock(mMutexMapPoint); 
    return mOrbMapPoints.find(pMP->mstrHexId) != mOrbMapPoints.end();
}

void Observer::ClearMapPointsFromMap(ORB_SLAM3::Map* pM)
{
    for(const auto& pMP : pM->GetAllMapPoints())
    {
      EraseMapPoint(pMP);
    }
}

std::map<std::string, ORB_SLAM3::MapPoint*>& Observer::GetAllMapPoints()
{
    //unique_lock<std::mutex> lock(mMutexMapPoint); 
    return mOrbMapPoints;
}

void Observer::AddMap(ORB_SLAM3::Map* pM)
{
    unique_lock<std::mutex> lock(mMutexMap); 
    mOrbMaps[pM->GetId()] = pM;
}

void Observer::EraseMap(ORB_SLAM3::Map* pM)
{
    unique_lock<std::mutex> lock(mMutexMap); 
    mOrbMaps.erase(pM->GetId());
}

std::map<unsigned long int, ORB_SLAM3::Map*>& Observer::GetAllMaps()
{
    //unique_lock<std::mutex> lock(mMutexMapPoint); 
    return mOrbMaps;
}





void Observer::AddKeyFrame(ORB_SLAM3::KeyFrame* pKF)
{
    unique_lock<std::mutex> lock(mMutexKeyFrame); 
    mOrbKeyFrames[pKF->mnId] = pKF;
}

void Observer::EraseKeyFrame(ORB_SLAM3::KeyFrame* pKF)
{
    unique_lock<std::mutex> lock(mMutexKeyFrame); 
    mOrbKeyFrames.erase(pKF->mnId);
}

bool Observer::CheckIfKeyFrameExists(ORB_SLAM3::KeyFrame* pKF)
{
    //unique_lock<std::mutex> lock(mMutexKeyFrame); 
    return mOrbKeyFrames.find(pKF->mnId) != mOrbKeyFrames.end();
}

void Observer::ClearKeyFramesFromMap(ORB_SLAM3::Map* pM)
{
    for(const auto& pKF : pM->GetAllKeyFrames())
    {
      EraseKeyFrame(pKF);
    }
}

std::map<unsigned long int, ORB_SLAM3::KeyFrame*>& Observer::GetAllKeyFrames()
{
    //unique_lock<std::mutex> lock(mMutexKeyFrame); 
    return mOrbKeyFrames;
}




void Observer::InjectMap(ORB_SLAM3::Map* tempMap, ORB_SLAM3::Map* pCurrentMap)
{
    // copy tempMP -> MP
    // Create copy constructor
    // Delete tempMP
    pCurrentMap->UpdateMap(*tempMap);
    delete tempMap;
}

ORB_SLAM3::KeyFrame* Observer::InjectKeyFrame(ORB_SLAM3::KeyFrame* tempKF, ORB_SLAM3::KeyFrame* mpExistingKF, const int nFromModule)
{ 
    //unique_lock<std::mutex> lock(mMutexKeyFrame); 
    mpExistingKF->UpdateKeyFrame(*tempKF, nFromModule);
    delete tempKF;
    return mpExistingKF; 
}

void Observer::InjectMapPoint(ORB_SLAM3::MapPoint* tempMP, ORB_SLAM3::MapPoint* mpExistingMP, const bool mbNew)
{
    
    if(mbNew)
    {
        mpAtlas->AddMapPoint(tempMP);
        //AddMapPoint(tempMP);
    }
    else 
    {
        //unique_lock<std::mutex> lock(mMutexMapPoint); 
        mpExistingMP->UpdateMapPoint(*tempMP);
        delete tempMP;
    }
}


void Observer::ForwardKeyFrameToTarget(ORB_SLAM3::KeyFrame* pKF, const unsigned int nFromModule)
{
    
    std::cout << " ----- Got new KF=" << pKF->mnId << ", from=" << nFromModule << ", to=" << pKF->mnNextTarget << " ----- " << std::endl;
    if(!pKF)
        return;

    mpAtlas->AddKeyFrame(pKF);


    // Just update state and do not insert to any module
    if(mnTaskModule == 0) {
        if(nFromModule == 3)
        {
            mpKeyFrameDB->erase(pKF);
            mpKeyFrameDB->add(pKF);
        }

    } else if (mnTaskModule == 1) {
        // in this case the system is mainly performing tracking
        // For now, tracking does not grab any KFs, only Maps 
        if(nFromModule==2)
        {
            std::cout << "#MPs in KF=" << pKF->GetMapPoints().size() << " **************** " << std::endl;
            //pKF->UpdateConnections();
            pKF->ComputeBoW();
            //mpAtlas->AddKeyFrame(pKF);
            mpKeyFrameDB->erase(pKF);
            mpKeyFrameDB->add(pKF);
            //if(nFromModule == 3)
            //{
            //    mpKeyFrameDB->erase(pKF);
            //    mpKeyFrameDB->add(pKF);
            //}

        }
    } else if (mnTaskModule == 2) {
        // in this case the system mainly performing local mapping
        if(pKF->mnNextTarget == 2) {
            // if the KF is meant to be inserted to LM
            // Insert to Local Mapping
            mpLocalMapper->InsertKeyframeFromRos(pKF);
        } else {

            if(nFromModule==3)
            {
                //pKF->ComputeBoW();
                //pKF->UpdateConnections();
                mpKeyFrameDB->erase(pKF);
                mpKeyFrameDB->add(pKF);
                //if(nFromModule == 3)
                //{
                //    mpKeyFrameDB->erase(pKF);
                //    mpKeyFrameDB->add(pKF);
                //}
            }
        }    
    } else if (mnTaskModule == 3) {
        // in this case the system mainly performing loop closing
        // not sure what to do here yet. 
        
        if(nFromModule==2)
            pKF->ComputeBoW();

        if(pKF->mnNextTarget == 3) {
            //pKF->UpdateConnections();
            mpLoopCloser->InsertKeyFrame(pKF);
        } 
    }

    pKF->mnNextTarget = 0;
}


ORB_SLAM3::MapPoint* Observer::ConvertMapPoint(const std::shared_ptr<orbslam3_interfaces::msg::MapPointUpdate>& mpRosMP, ORB_SLAM3::MapPoint* mpExistingMP)
{

    ORB_SLAM3::MapPoint* tempMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
    if(mpExistingMP)
    {
        //unique_lock<std::mutex> lock(mMutexMapPoint); 
        tempMP = new ORB_SLAM3::MapPoint(*mpExistingMP);
        Converter::MapPointConverter::RosMapPointUpdateToOrb(mpRosMP, tempMP);
    } 
    //else {
    //    ORB_SLAM3::MapPoint* mpCopyMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
    //    tempMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, mpCopyMP);
    //}

    if(mOrbMaps[mpRosMP->mp_map_id])
      tempMP->UpdateMap(mOrbMaps[mpRosMP->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    else
      tempMP->UpdateMap(static_cast<ORB_SLAM3::Map*>(NULL));

    return tempMP;
}

ORB_SLAM3::MapPoint* Observer::ConvertMapPoint(const std::shared_ptr<orbslam3_interfaces::msg::MapPoint>& mpRosMP, ORB_SLAM3::MapPoint* mpExistingMP)
{

    ORB_SLAM3::MapPoint* tempMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
    if(mpExistingMP)
    {
        //unique_lock<std::mutex> lock(mMutexMapPoint); 
        //ORB_SLAM3::MapPoint* mpCopyMP = new ORB_SLAM3::MapPoint(*mpExistingMP);
        tempMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, mpExistingMP);
    } else {
        //ORB_SLAM3::MapPoint* mpCopyMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
        tempMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, mpExistingMP);
        //AddMapPoint(tempMP);
    }

    if(mOrbMaps[mpRosMP->mp_map_id])
      tempMP->UpdateMap(mOrbMaps[mpRosMP->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    else
      tempMP->UpdateMap(static_cast<ORB_SLAM3::Map*>(NULL));

    return tempMP;
}


ORB_SLAM3::KeyFrame* Observer::ConvertKeyFrame(const std::shared_ptr<orbslam3_interfaces::msg::KeyFrame>& mpRosKF, ORB_SLAM3::KeyFrame* mpExistingKF)
{
    ORB_SLAM3::KeyFrame* tempKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
    //std::chrono::steady_clock::time_point time_StartConvKF = std::chrono::steady_clock::now();
    if(mpExistingKF)
    {
      //unique_lock<std::mutex> lock(mMutexKeyFrame); 
      tempKF = mpExistingKF;
      Converter::KeyFrameConverter::UpdateORBKeyFrame(mpRosKF, tempKF);
      tempKF->UpdateMap(mOrbMaps[mpRosKF->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    }
    else
    {
      tempKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, tempKF);
      tempKF->SetORBVocabulary(pSLAM->GetORBVocabulary());
      tempKF->SetKeyFrameDatabase(mpKeyFrameDB);
      if(mOrbMaps[mpRosKF->mp_map_id])
        tempKF->UpdateMap(mOrbMaps[mpRosKF->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
      else
        tempKF->UpdateMap(static_cast<ORB_SLAM3::Map*>(NULL));
    }


    return tempKF;
}

void Observer::ConvertKeyFrame(const std::shared_ptr<orbslam3_interfaces::msg::KeyFrameUpdate>& mpRosKF, ORB_SLAM3::KeyFrame* mpExistingKF)
{
    //std::chrono::steady_clock::time_point time_StartConvKF = std::chrono::steady_clock::now();
    if(mpExistingKF)
    {
      //unique_lock<std::mutex> lock(mMutexKeyFrame); 
      Converter::KeyFrameConverter::UpdateORBKeyFrame(mpRosKF, mpExistingKF);
      mpExistingKF->UpdateMap(mOrbMaps[mpRosKF->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    }
    //else
    //{
    //  tempKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, tempKF);
    //  tempKF->SetORBVocabulary(pSLAM->GetORBVocabulary());
    //  tempKF->SetKeyFrameDatabase(mpKeyFrameDB);
    //  if(mMaps[mpRosKF->mp_map_id])
    //    tempKF->UpdateMap(mMaps[mpRosKF->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    //  else
    //    tempKF->UpdateMap(static_cast<ORB_SLAM3::Map*>(NULL));
    //  tempKF->ComputeBoW(); 
    //}


    //return tempKF;
}


double Observer::TimeSinceReset()
{
  //std::unique_lock<std::mutex> lock(mMutexTimes);
  mdSinceReset = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - mtLastResetTime).count();
  return mdSinceReset;
}

std::chrono::system_clock::time_point Observer::GetLastResetTime()
{
  std::unique_lock<std::mutex> lock(mMutexTimes);
  return mtLastResetTime;
}

void Observer::UpdateLastResetTime()
{
  std::unique_lock<std::mutex> lock(mMutexTimes);
  mtLastResetTime = std::chrono::system_clock::now();
}

int Observer::GetTaskModule()
{
  std::unique_lock<std::mutex> lock(mMutexTask);
  return mnTaskModule;
}


void Observer::AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM)
{
    pSLAM = mSLAM;

}

void Observer::AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node)
{
    pSLAMNode = slam_node;

    mpTracker = pSLAM->GetTrackerPtr();
    mpLocalMapper = pSLAM->GetMapperPtr();
    mpLoopCloser = pSLAM->GetLoopClosingPtr();
    mpAtlas = pSLAM->GetAtlas();
    mpKeyFrameDB = pSLAM->GetKeyFrameDatabase();

    for(const auto map : mpAtlas->GetAllMaps())
    {
      AddMap(map);
    }
}

void Observer::AddNewWorker(unsigned int mnWorkerModule)
{
   unique_lock<std::mutex> lock(mMutexWorker); 
   mspWorkers.insert(mnWorkerModule);
}

int Observer::GetWorkerNumber()
{
   unique_lock<std::mutex> lock(mMutexWorker); 
   return mspWorkers.size();
}


bool Observer::CheckIfWorkerExists(const unsigned int mnTargetID)
{
   unique_lock<std::mutex> lock(mMutexWorker); 
   return mspWorkers.count(mnTargetID);
}


// Override from ORBSLAM3
int Observer::KeyFramesInQueue()
{
    //unique_lock<std::mutex> lock(mMutexNewKFs);
    return mpKeyFrameSubscriber->KeyFramesInQueue();
    //return 0;
}

int Observer::MapsInQueue()
{
    //unique_lock<std::mutex> lock(mMutexNewMaps);
    //return mlpMapQueue.size();
    return mpMapHandler->LocalMapsInQueue();
}

void Observer::onNewMap(ORB_SLAM3::Map* pM)
{
    std::cout << "Observer::onNewMap" << std::endl;
    AddMap(pM);
}

void Observer::onNewKeyFrame(ORB_SLAM3::KeyFrame* pKF)
{
    //if(!CheckIfKeyFrameExists(pKF))
    //{
    //AddKeyFrame(pKF);
    //}

}

void Observer::onNewMapPoint(ORB_SLAM3::MapPoint* pMP)
{
      //AddMapPoint(pMP);
}

void Observer::onActiveMapReset(unsigned long int mnMapId)
{
  if(pSLAMNode)
      pSLAMNode->publishResetActiveMap(mnMapId);
}

void Observer::onLMResetRequested()
{
  if(pSLAMNode)
      pSLAMNode->publishLMResetRequested();
}

//void Observer::onChangeLMActive(bool bActive)
//{
//  if(pSLAMNode)
//      pSLAMNode->publishLMActivityChange(bActive);
//}
void Observer::onKeyframeAdded(ORB_SLAM3::KeyFrame* pKF, std::set<std::string> msNewMapPointIds)
{
  const unsigned int nTarget = pKF->mnNextTarget;
  if (GetWorkerNumber() >= 1 && pSLAMNode) {
      if(CheckIfWorkerExists(nTarget))
      {
          std::cout << "Worker exists, sending it to the device." << std::endl;
          //if(nTarget == 2)
          mpKeyFramePublisher->InsertNewKeyFrame(pKF, msNewMapPointIds);
          // Other wise its send with map
      } 
  }
  else {
      std::cout << "Worker does not exist, Keeping it here." << std::endl;

      if(nTarget==2)
          mpLocalMapper->InsertKeyFrame(pKF);
      else if(nTarget==3)
          mpLoopCloser->InsertKeyFrame(pKF);
  } 

}

void Observer::onKeyframeAdded(ORB_SLAM3::KeyFrame* pKF)
{
  //if(!CheckIfKeyFrameExists(pKF))
  //{
  //  AddKeyFrame(pKF);
  //}


  const unsigned int nTarget = pKF->mnNextTarget;
  if (GetWorkerNumber() >= 1 && pSLAMNode) {
      if(CheckIfWorkerExists(nTarget))
      {
          std::cout << "Worker exists, sending it to the device." << std::endl;
          //if(nTarget == 2)
          mpKeyFramePublisher->InsertNewKeyFrame(pKF);
          // Other wise its send with map
      } 
  }
  else {
      std::cout << "Worker does not exist, Keeping it here." << std::endl;

      if(nTarget==2)
          mpLocalMapper->InsertKeyFrame(pKF);
      else if(nTarget==3)
          mpLoopCloser->InsertKeyFrame(pKF);
  } 
}

void Observer::onLocalMapUpdated(ORB_SLAM3::Map* pM)
{
  if(GetWorkerNumber() >= 1 && pSLAMNode)
      mpMapHandler->InsertNewPubLocalMap(pM);
}

void Observer::onGlobalMapUpdated(bool mbMerged, bool mbLoopClosure, std::vector<unsigned long int> mvMeergedIds)
{
  if(GetWorkerNumber() >= 1&& pSLAMNode)
  {
      std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate = std::tuple<bool, bool, std::vector<unsigned long int>>(mbMerged, mbLoopClosure, mvMeergedIds);
      mpMapHandler->InsertNewPubGlobalMap(mtAtlasUpdate);
  } 
}

//} // namespave TempDistributor

