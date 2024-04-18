#include "./Observer.hpp"

//#include "./KeyFramePublisher.hpp"
#include "../slam/slam-wrapper-node.hpp"

//namespace TempDistributor
//{

Observer::Observer(MapHandler* pMapHandler, KeyFramePublisher* pKeyFramePublisher, KeyFrameSubscriber* pKeyFrameSubscriber)
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

    // Attach handlers internally
    mpKeyFramePublisher = pKeyFramePublisher;
    mpKeyFrameSubscriber = pKeyFrameSubscriber;
    mpMapHandler = pMapHandler;
}

Observer::~Observer()
{

}

void Observer::InjectMap(ORB_SLAM3::Map* tempMap, ORB_SLAM3::Map* pCurrentMap)
{
    // copy tempMP -> MP
    // Create copy constructor
    // Delete tempMP
    pCurrentMap->UpdateMap(*tempMap);
    delete tempMap;
}

ORB_SLAM3::KeyFrame* Observer::InjectKeyFrame(ORB_SLAM3::KeyFrame* tempKF, std::map<unsigned long int, ORB_SLAM3::KeyFrame*> mMapKFs)
{ 
    if(mMapKFs.find(tempKF->mnId) == mMapKFs.end())
    {
        return tempKF;
    } else {
        // copy tempKF -> KF
        // Create copy constructor
        // Delete tempKF
        ORB_SLAM3::KeyFrame* pKF = mMapKFs[tempKF->mnId];
        pKF->UpdateKeyFrame(*tempKF);
        delete tempKF;
        return pKF; 
    }
}

void Observer::InjectMapPoint(ORB_SLAM3::MapPoint* tempMP, std::map<std::string, ORB_SLAM3::MapPoint*> mMapMPs)
{
    if(mMapMPs.find(tempMP->mstrHexId) == mMapMPs.end())
    {
        ORB_SLAM3::Map* pMap = tempMP->GetMap();
        if(pMap)
          pMap->AddMapPoint(tempMP);
    } else {
        // copy tempMP -> MP
        // Create copy constructor
        // Delete tempMP
        ORB_SLAM3::MapPoint* pMP = mMapMPs[tempMP->mstrHexId];
        pMP->UpdateMapPoint(*tempMP);
        delete tempMP;
    }
}

void Observer::ForwardKeyFrameToTarget(ORB_SLAM3::KeyFrame* pKF, const unsigned int mnTargetModule, const unsigned int nFromModule)
{
    // Just update state and do not insert to any module
    if(mnTaskModule == 0) {
        //for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
        //{
        //    if(pMPi)
        //        mpAtlas_->AddMapPoint(pMPi);
        //}
        
        if(pKF)
        {
            //IncreaseKFCount(1);  
            if(nFromModule == 3)
            {
                mpKeyFrameDB->erase(pKF);
                mpKeyFrameDB->add(pKF);
            }

            mpAtlas->AddKeyFrame(pKF);
        }
    } else if (mnTaskModule == 1) {
        // in this case the system is mainly performing tracking
        // For now, tracking does not grab any KFs, only Maps 
        if(pKF)
        {
            if(nFromModule == 3)
            {
                mpKeyFrameDB->erase(pKF);
                mpKeyFrameDB->add(pKF);
            }
            mpAtlas->AddKeyFrame(pKF);
        }
    } else if (mnTaskModule == 2) {
        // in this case the system mainly performing local mapping
        if(mnTargetModule == 2) {
            // if the KF is meant to be inserted to LM
            // Insert to Local Mapping
            if(mpLocalMapper->NeedNewKeyFrame(pKF))
            {
                //mpKeyFrameDB->add(pKF);
                //IncreaseKFCount(1);  
                mpLocalMapper->InsertKeyframeFromRos(pKF);
            } 
            else {
                //for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
                //{
                //    if(pMPi)
                //        mpAtlas_->AddMapPoint(pMPi);
                //}
                
                if(pKF)
                {
                    //IncreaseKFCount(1);  
                    if(nFromModule == 3)
                    {
                        mpKeyFrameDB->erase(pKF);
                        mpKeyFrameDB->add(pKF);
                    }
                    mpAtlas->AddKeyFrame(pKF);
                }
            //    mspKFsReadyForLM.insert(pKF->mnId);
            //    startTimer();
                //pKF->SetBadFlag();
            }  
        } else {

            if(pKF)
            {
                if(nFromModule == 3)
                {
                    mpKeyFrameDB->erase(pKF);
                    mpKeyFrameDB->add(pKF);
                }
                mpAtlas->AddKeyFrame(pKF);
            }
        }    
    } else if (mnTaskModule == 3) {
        // in this case the system mainly performing loop closing
        // not sure what to do here yet. 
        if(mnTargetModule == 3) {
            if(pKF)
            {
                //for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
                //{
                //    if(pMPi)
                //        mpAtlas_->AddMapPoint(pMPi);
                //}
                //IncreaseKFCount(1);  
                mpAtlas->AddKeyFrame(pKF);
                pKF->UpdateConnections();
                mpLoopCloser->InsertKeyFrame(pKF);
            }
        } else {

            //for(ORB_SLAM3::MapPoint* pMPi : pKF->GetMapPoints())
            //{
            //    if(pMPi)
            //        mpAtlas_->AddMapPoint(pMPi);
            //}
            
            if(pKF)
            {
                //mpKeyFrameDB->add(pKF);
                mpAtlas->AddKeyFrame(pKF);
            }
        }
    }
}

ORB_SLAM3::MapPoint* Observer::ConvertMapPoint(std::shared_ptr<orbslam3_interfaces::msg::MapPoint> mpRosMP, std::map<unsigned long int, ORB_SLAM3::Map*> mMaps)
{
    ORB_SLAM3::MapPoint* tempMP = Converter::MapPointConverter::RosMapPointToOrb(mpRosMP, static_cast<ORB_SLAM3::MapPoint*>(NULL));
    if(mMaps[mpRosMP->mp_map_id])
      tempMP->UpdateMap(mMaps[mpRosMP->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    else
      tempMP->UpdateMap(static_cast<ORB_SLAM3::Map*>(NULL));
    return tempMP;
}

ORB_SLAM3::KeyFrame* Observer::ConvertKeyFrame(std::shared_ptr<orbslam3_interfaces::msg::KeyFrame> mpRosKF, std::map<unsigned long int, ORB_SLAM3::Map*> mMaps)
{
    ORB_SLAM3::KeyFrame* tempKF = Converter::KeyFrameConverter::ROSKeyFrameToORBSLAM3(mpRosKF, static_cast<ORB_SLAM3::KeyFrame*>(NULL));
    tempKF->SetORBVocabulary(pSLAM->GetORBVocabulary());
    tempKF->SetKeyFrameDatabase(mpKeyFrameDB);
    if(mMaps[mpRosKF->mp_map_id])
      tempKF->UpdateMap(mMaps[mpRosKF->mp_map_id]); // This one should be with mp_map_id and mpAtlas_->GetAllMaps();
    else
      tempKF->UpdateMap(static_cast<ORB_SLAM3::Map*>(NULL));
    tempKF->ComputeBoW(); 
    return tempKF;
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


// Override from ORBSLAM3
int Observer::KeyFramesInQueue()
{
    //unique_lock<std::mutex> lock(mMutexNewKFs);
    //return mlpKeyFrameQueue.size();
    return 0;
}

int Observer::MapsInQueue()
{
    //unique_lock<std::mutex> lock(mMutexNewMaps);
    //return mlpMapQueue.size();
    return 0;
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

void Observer::onChangeLMActive(bool bActive)
{
  if(pSLAMNode)
      pSLAMNode->publishLMActivityChange(bActive);
}

void Observer::onKeyframeAdded(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule)
{
  mpKeyFramePublisher->InsertNewKeyFrame(pKF, mnTargetModule);
  //if(CheckIfWorkerExists(mnTargetModule) && slam_node_)
  //{
  //    std::cout << "Worker exists, sending it to the device." << std::endl;
  //    InsertNewKeyFrame(pKF, mnTargetModule);
  //} else if (GetWorkerNumber() > 1) {
  //    if(mnTargetModule==2)
  //        mpLocalMapper->InsertKeyFrame(pKF);
  //    else if(mnTargetModule==3)
  //    {
  //        InsertNewKeyFrame(pKF, mnTargetModule);
  //        mpLoopCloser->InsertKeyFrame(pKF);
  //    }

  //}
  //else if(GetWorkerNumber() <= 1) {
  //    std::cout << "Worker does not exist, Keeping it here." << std::endl;

  //    if(mnTargetModule==2)
  //        mpLocalMapper->InsertKeyFrame(pKF);
  //    else if(mnTargetModule==3)
  //        mpLoopCloser->InsertKeyFrame(pKF);
  //} 
}

void Observer::onLocalMapUpdated(ORB_SLAM3::Map* pM)
{
  if(GetWorkerNumber() > 1)
      mpMapHandler->InsertNewPubLocalMap(pM);
}

void Observer::onGlobalMapUpdated(bool mbMerged, bool mbLoopClosure, std::vector<unsigned long int> mvMeergedIds)
{
  if(GetWorkerNumber() > 1)
  {
      std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate = std::tuple<bool, bool, std::vector<unsigned long int>>(mbMerged, mbLoopClosure, mvMeergedIds);
      mpMapHandler->InsertNewPubGlobalMap(mtAtlasUpdate);
  } 
}

//} // namespave TempDistributor

