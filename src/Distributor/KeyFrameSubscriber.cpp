#include "./KeyFrameSubscriber.hpp"
#include "./Observer.hpp"
//#include "../slam/slam-wrapper-node.hpp"

//namespace TempDistributor {

KeyFrameSubscriber::KeyFrameSubscriber()
{

  mlpReadyKeyFrames = std::vector<ORB_SLAM3::KeyFrame*>();
  mlpReadyMapPoints= std::vector<ORB_SLAM3::MapPoint*>();
  mlpReadyMapPoints.reserve(5000);
  mlpReadyKeyFrames.reserve(200);

}

KeyFrameSubscriber::~KeyFrameSubscriber()
{

}

// main function
void KeyFrameSubscriber::Run()
{
    mbFinished = false;

    //std::cout << "KeyFrameSubscriber Thread has been started." << std::endl;
    
    while(1)
    {
      if(!mpLoopCloser->CheckIfRunning() && !mpLoopCloser->isRunningGBA())//  && !mpLocalMapper->IsLMRunning())
      {
          // Then new keyframes
          // Here, write some logic which checks if the updated KF was in map
          // if so, do not publish again.
          // second check if there is new maps
          if(CheckNewKeyFrames())
          {
            ProcessNewKeyFrame();
          }
          else if(CheckNewKeyFrameUpdates()) //&& mpObserver->mbMapIsUpToDate)
          {
            ProcessNewKeyFrameUpdate();
          }
          
          //if((!CheckNewKeyFrameUpdates() && !mlpReadyMapPoints.empty()) || mlpReadyMapPoints.size() >= 1000 )
          //{
          //    for(const auto& pMP : mlpReadyMapPoints)
          //    {
          //        //std::cout << "adding ready MP" << std::endl;
          //        ORB_SLAM3::MapPoint* existingMP = mpObserver->GetMapPoint(pMP->mstrHexId);
          //        if(existingMP)
          //        {
          //          mpObserver->InjectMapPoint(pMP, existingMP, false);
          //        }
          //    }

          //    mlpReadyMapPoints.clear();
          //}

          //if((!CheckNewKeyFrameUpdates()&& !mlpReadyMapPoints.empty()) || mlpReadyKeyFrames.size() >= 50)
          //{
          //    for(const auto& pKF : mlpReadyKeyFrames)
          //    {
          //        ORB_SLAM3::KeyFrame* existingKF = mpObserver->GetKeyFrame(pKF->mnId);
          //        if(existingKF)
          //        {
          //          //std::cout << "adding ready KF" << std::endl;
          //          ORB_SLAM3::KeyFrame* pInjectedKF = mpObserver->InjectKeyFrame(pKF, existingKF, existingKF->GetLastModule());
          //          mpObserver->ForwardKeyFrameToTarget(pInjectedKF, pInjectedKF->GetLastModule(), false);
          //        }
          //    }
          //    mlpReadyKeyFrames.clear();

          //}
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

bool KeyFrameSubscriber::CheckNewKeyFrameUpdates()
{
    unique_lock<mutex> lock(mMutexNewRosKFs);
    return(!mpRosKeyFrameUpdateQueue.empty());
}

int KeyFrameSubscriber::KeyFramesInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewRosKFs);
    return mlpRosKeyFrameQueue.size();
}

int KeyFrameSubscriber::KeyFrameUpdatesInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewRosKFs);
    return mpRosKeyFrameUpdateQueue.size();
}

void KeyFrameSubscriber::ProcessNewKeyFrameUpdate()
{


    //std::cout << "KeyFrameSubscriber :: Processing new KF, KFs in Queue=" << KeyFramesInQueue() << std::endl;
    orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr pRosKF = static_cast<orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewRosKFs);
        if(mpObserver->GetTaskModule() == 3)
        {
            auto it = mpRosKeyFrameUpdateQueue.begin();//mlpRosKeyFrameUpdateQueue.front();
            pRosKF = it->second;
            mpRosKeyFrameUpdateQueue.erase(pRosKF->mn_id);
            if(pRosKF->mn_last_module == 3)
                return;

        } else 
        {
            
            auto it = mpRosKeyFrameUpdateQueue.rbegin();//mlpRosKeyFrameUpdateQueue.front();
            pRosKF = it->second;
            mpRosKeyFrameUpdateQueue.erase(pRosKF->mn_id);
            //for(std::map<unsigned long int, orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr>::reverse_iterator it = mpRosKeyFrameUpdateQueue.rbegin(); it != mpRosKeyFrameUpdateQueue.rend(); ++it)
            //{
            //    if(it->first != mpTracker->GetReferenceID())
            //    {
            //        pRosKF = it->second;
            //        mpRosKeyFrameUpdateQueue.erase(pRosKF->mn_id);
            //        break;
            //    } else {
            //        if(mpRosKeyFrameUpdateQueue.size()==1)
            //        {
            //            return;
            //        } 
            //    }

            //}
        }

        //mlpRosKeyFrameUpdateQueue.pop_front();
    }
  
    if(!pRosKF)
    {
        std::cout << "- KeyFrame Update is null. Returning...";
        return;
    }
    
    double timeSinceReset = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - mpObserver->GetLastResetTime()).count();
    if(timeSinceReset < 50)
        return;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartRos2OrbProcKF = std::chrono::steady_clock::now();
    std::chrono::system_clock::time_point time_Start = std::chrono::system_clock::now();
    vtTimes.push_back(time_Start);
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPrepDataKF = std::chrono::steady_clock::now();


    // Get maps and the map where KF is
    std::map<unsigned long int, ORB_SLAM3::Map*>& mMaps = mpObserver->GetAllMaps();
    ORB_SLAM3::Map* pCurrentMap = mMaps[pRosKF->mp_map_id];

    if(!pCurrentMap)
    {
        mpObserver->UpdateLastResetTime();
        mpAtlas->CreateNewMap();
        pCurrentMap = mpAtlas->GetCurrentMap(); 
        pCurrentMap->attachDistributor(mpObserver);
        mpLocalMapper->Release();
    } 
    //else if(pCurrentMap && pCurrentMap != mpAtlas->GetCurrentMap() && pCurrentMap->GetId() < mpAtlas->GetCurrentMap()->GetId()) 
    //{
    //  //mpAtlas->ChangeMap(pCurrentMap);
    //  //pCurrentMap->SetCurrentMap();
    //  pRosKF->mp_map_id = pCurrentMap->GetId(); //mpAtlas->GetCurrentMap()->GetId();
    //  pCurrentMap = mpAtlas->GetCurrentMap();
    //}

    // Create map data structures for postloads
    std::map<std::string, ORB_SLAM3::MapPoint*>& mFusedMPs = mpObserver->GetAllMapPoints(); 
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*>& mFusedKFs = mpObserver->GetAllKeyFrames(); 
    std::vector<bool> mvbNewMPs;
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    
    //std::vector<ORB_SLAM3::KeyFrame*> mvpKFs = pCurrentMap->GetAllKeyFrames(); 
    //for(int i=0;i<mvpKFs.size();++i)
    //{
    //  ORB_SLAM3::KeyFrame* pKFi = mvpKFs[i];
    //  if(!pKFi)
    //      continue;
    //  mFusedKFs[pKFi->mnId] = pKFi;
    //}

    //mFusedKFs = mpObserver->GetAllKeyFrames();

    vnKFAmount.push_back(mFusedKFs.size());

    //mFusedMPs = mpObserver->GetAllMapPoints();
    //std::vector<ORB_SLAM3::MapPoint*> mvpMPs = pCurrentMap->GetAllMapPoints(); 

    //for(int i=0;i<mvpMPs.size();++i)
    //{
    //  ORB_SLAM3::MapPoint* pMPi = mvpMPs[i];
    //  if(!pMPi)
    //      continue;
    //  mFusedMPs[pMPi->mstrHexId] = pMPi;
    //}

    vnMPAmount.push_back(mFusedMPs.size());

    for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas->GetAllCameras())
    {
      mCameras[pCam->GetId()] = pCam;
    }
    
    // End of timer
    std::chrono::steady_clock::time_point time_EndPrepDataKF = std::chrono::steady_clock::now();
    double timePrepDataKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPrepDataKF - time_StartPrepDataKF).count();
    vdPrepDataKF_ms.push_back(timePrepDataKF);
    //std::cout << "Got a new keyframe update. 1. Existing data is collected." << std::endl;
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartConvKF = std::chrono::steady_clock::now();
    
    bool mbNewKF = true;

    //timeSinceReset = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - mpObserver->GetLastResetTime()).count();
    //if(timeSinceReset < 50)
    //    return;

    // Convert ros msg to ORB_SLAM3 Objects
    ORB_SLAM3::KeyFrame* tempKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL); //mpObserver->ConvertKeyFrame(pRosKF, mMaps);
    ORB_SLAM3::KeyFrame* mpCopyKF = mpObserver->GetKeyFrame(pRosKF->mn_id);
    if(mpCopyKF) //mFusedKFs.find(pRosKF->mn_id) != mFusedKFs.end())
    {
      tempKF = new ORB_SLAM3::KeyFrame(*mpCopyKF);
      mpObserver->ConvertKeyFrame(pRosKF, tempKF);
      mpCopyKF->SetPose(tempKF->GetPose());
      mbNewKF=false;
      //mvbNewKF.push_back(false);

    } 

    if(!tempKF)
      return;
    // TODO: NO NEED SINCE ITS UPDATE AND SHOULD NEVER BE NEW, ERASE HERE IF NOT FOUND 
    //else {

    //  ORB_SLAM3::KeyFrame* mpExistingKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
    //  tempKF = mpObserver->ConvertKeyFrame(pRosKF, mpExistingKF);
    //  //mvbNewKF.push_back(true);
    //  mpObserver->AddKeyFrame(tempKF);
    //  //mFusedKFs[tempKF->mnId] = tempKF;
    //}

    // End of timer
    std::chrono::steady_clock::time_point time_EndConvKF = std::chrono::steady_clock::now();
    double timeConvKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvKF - time_StartConvKF).count();
    vdRos2OrbConvKF_ms.push_back(timeConvKF);


    std::vector<ORB_SLAM3::MapPoint*> mvpTempMPs;
    mvpTempMPs.reserve(pRosKF->mvp_map_points.size());
    for(size_t i=0;i<pRosKF->mvp_map_points.size();++i)
    {
        unsigned long int mnId = pRosKF->mvp_map_points[i].mn_id;
        mpObserver->UpdateMaxMPId(mnId);
    }

    mpObserver->UpdateMaxMPId(ORB_SLAM3::MapPoint::nNextId);

    //std::cout << "Max MP ID=" << mpObserver->GetMaxMPId() << std::endl;
    ORB_SLAM3::MapPoint::nNextId=mpObserver->GetMaxMPId()+1;
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartConvMP = std::chrono::steady_clock::now();
    
    //std::cout << mMapMPs.size() << " ------------------------------------" << std::endl;
    int mnTaskModule = mpObserver->GetTaskModule();
    int updates=0;
    int news=0;
    for(int i=0; i<pRosKF->mvp_map_points.size(); ++i)
    {
        //if(pRosKF->mvp_map_points[i].mn_last_module > mnTaskModule || mFusedMPs.find(pRosKF->mvp_map_points[i].m_str_hex_id) == mFusedMPs.end()) 
        //{
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(pRosKF->mvp_map_points[i]);
        if(mnTaskModule==3 && mpRosMP->mn_last_module == 3)
            continue;

        mpRosMP->mp_map_id = mpAtlas->GetCurrentMap()->GetId();
        ORB_SLAM3::MapPoint* tempMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
        ORB_SLAM3::MapPoint* mpExistingMP = mpObserver->GetMapPoint(mpRosMP->m_str_hex_id);
        if(mpExistingMP)//mFusedMPs.find(mpRosMP->m_str_hex_id) != mFusedMPs.end())
        {
          //ORB_SLAM3::MapPoint* mpCopyMP = mpObserver->GetMapPoint(mpRosMP->m_str_hex_id); //mFusedMPs[mpRosMP->m_str_hex_id];
          //if(!mpCopyMP)
          //    continue;
          ORB_SLAM3::MapPoint* mpCopyMP = new ORB_SLAM3::MapPoint(*mpExistingMP);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpCopyMP);
          mpExistingMP->SetWorldPos(tempMP->GetWorldPos());
          mpExistingMP->SetNormalVector(tempMP->GetNormal());

          if(mMaps[mpRosMP->mp_map_id])
              mpExistingMP->UpdateMap(mMaps[mpRosMP->mp_map_id]);

          mvbNewMPs.push_back(false);
          updates++;

        } else {

          mpExistingMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpExistingMP);
          mvbNewMPs.push_back(true);
          mpObserver->AddMapPoint(tempMP);
          //if(mMaps[mpRosMP->mp_map_id])
          //    mpExistingMP->UpdateMap(mMaps[mpRosMP->mp_map_id]);
          //mFusedMPs[tempMP->mstrHexId] = tempMP;
          news++;
        }

        //mMapMPs[tempMP->mstrHexId] = tempMP;
        mvpTempMPs.push_back(tempMP);
            //ORB_SLAM3::MapPoint* tempMPi = mpObserver->ConvertMapPoint(std::make_shared<orbslam3_interfaces::msg::MapPoint>(pRosKF->mvp_map_points[i]), mMaps);
            //if(tempMPi)
            //{
            //    mMapMPs[tempMPi->mstrHexId] = mFusedMPs[tempMPi->mstrHexId];
            //    mvpTempMPs.push_back(tempMPi);
            //    if(mFusedMPs.find(tempMPi->mstrHexId) == mFusedMPs.end())
            //        mFusedMPs[tempMPi->mstrHexId] = tempMPi;
            //}
        //}
    }
    //std::cout << " ================ KF=" << pRosKF->mn_id << ", MPs=" << mvpTempMPs.size() << " ============ " << std::endl;
    vnNewMPAmount.push_back(news);
    vnUpdateMPAmount.push_back(updates);

    // End of timer
    std::chrono::steady_clock::time_point time_EndConvMP = std::chrono::steady_clock::now();
    double timeConvMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvMP - time_StartConvMP).count();
    vdRos2OrbConvMP_ms.push_back(timeConvMP);
    
    //std::cout << "Got a new keyframe. 2. New data is fused with old data." << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPostLoadKF = std::chrono::steady_clock::now();

    // Postloads -> Reassign pointers etc to temporary data
    bool bKFUnprocessed = false;

    mpObserver->LoadKeyFrame(tempKF, mCameras);

    // End of timer
    std::chrono::steady_clock::time_point time_EndPostLoadKF = std::chrono::steady_clock::now();

    // Calculate difference
    double timePostLoadKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPostLoadKF - time_StartPostLoadKF).count();
    vdPostLoadKF_ms.push_back(timePostLoadKF);

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPostLoadMP = std::chrono::steady_clock::now();
    
    for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
        mpObserver->LoadMapPoint(tempMP);
    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndPostLoadMP = std::chrono::steady_clock::now();


    //mFusedKFs[pKF->mnId] = pKF;
    
    //std::cout << "Got a new keyframe. 3. KF PostLoad, and injection is completed." << std::endl;



    // Calculate difference
    double timePostLoadMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPostLoadMP - time_StartPostLoadMP).count();
    vdPostLoadMP_ms.push_back(timePostLoadMP);
    
    //std::cout << "Got a new keyframe. 4. All MP PostLoads Completed." << std::endl;

    ORB_SLAM3::KeyFrame* pKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL); 
    {
        //unique_lock<std::mutex> lock(ORB_SLAM3::MapPoint::mGlobalMutex);
        //unique_lock<mutex> lock2(pCurrentMap->mMutexMapUpdate);


        // Start of a timer -------------
        std::chrono::steady_clock::time_point time_StartInjectKF = std::chrono::steady_clock::now();

        // -------------------- From here on we have unprocessed data -----------------------
        // Here the data is injected into existing KF or the same one is returned. Function handles pointers etc.
        if(!mbNewKF && tempKF)
        {
            ORB_SLAM3::KeyFrame* mpExistingKF = mpObserver->GetKeyFrame(tempKF->mnId);//mFusedKFs[tempKF->mnId];
            if(mpExistingKF)
                pKF = mpObserver->InjectKeyFrame(tempKF, mpExistingKF, pRosKF->from_module_id);
          
            //if(pKF && pKF->isBad())
            //    pKF->SetBadFlag();

            //if(tempKF->mnNextTarget==0)
            //    mlpReadyKeyFrames.push_back(tempKF);
            //else
        }
        //else
        //    pKF = tempKF;
        //ORB_SLAM3::KeyFrame* pKF = mpObserver->InjectKeyFrame(tempKF, mFusedKFs);

        // End of timer
        std::chrono::steady_clock::time_point time_EndInjectKF = std::chrono::steady_clock::now();

        // Calculate difference
        double timeInjectKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndInjectKF - time_StartInjectKF).count();
        vdInjectKF_ms.push_back(timeInjectKF);
        //std::cout << "Got a new keyframe. 5. All MPs are injected to ORB_SLAM3." << std::endl;




        // Start of a timer -------------
        std::chrono::steady_clock::time_point time_StartInjectMP = std::chrono::steady_clock::now();
        
        //for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
        for(size_t i=0;i<mvpTempMPs.size();++i)
        {
          ORB_SLAM3::MapPoint* tempMP = mvpTempMPs[i];
          bool mbNew = mvbNewMPs[i];
          if(!tempMP)
            continue;

          ORB_SLAM3::MapPoint* mpExistingMP = mpObserver->GetMapPoint(tempMP->mstrHexId); //mFusedMPs[mpRosMP->m_str_hex_id];
          if(!mpExistingMP)
          {
              mpExistingMP=static_cast<ORB_SLAM3::MapPoint*>(NULL);
              mbNew = true;
          }

          if(pKF)
              tempMP->SetReferenceKeyFrame(pKF);

          //if(mvbNewMPs[i])
          mpObserver->InjectMapPoint(tempMP, mpExistingMP, mbNew);
          if(mpExistingMP && mpExistingMP->isBad())
              mpExistingMP->SetBadFlag();
          else if(tempMP && tempMP->isBad())
              tempMP->SetBadFlag();
          //else
          //   mlpReadyMapPoints.push_back(tempMP); 
            //mpObserver->InjectMapPoint(tempMP, mMapMPs);
        }

        // End of timer
        std::chrono::steady_clock::time_point time_EndInjectMP = std::chrono::steady_clock::now();

        // Calculate difference
        double timeInjectMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndInjectMP - time_StartInjectMP).count();
        vdInjectMP_ms.push_back(timeInjectMP);
        // -------------------- To here -----------------------
        

    }
    
    
    // If KF exists, forward it to target
    if(pKF)
    {
        //if(mpObserver->GetTaskModule() == 1)
        //    mpTracker->UpdateReference(pKF);
        //if(pKF->GetMap())
        //    std::cout << " --- pKF->Map=" << pKF->GetMap()->GetId() << ", pRosKF->mp_map_id=" << pRosKF->mp_map_id << std::endl;
        mpObserver->ForwardKeyFrameToTarget(pKF, pRosKF->from_module_id, true);
    }
    
    //if(pCurrentMap->KeyFramesInMap() < 10)
    //{
    //    mpTracker->UpdateReference(pKF);
    //}
    //std::cout << "Got a new keyframe Update. 6. KF is forwarded to the target. KF is COMPLETE." << std::endl;

    // End of timer
    std::chrono::steady_clock::time_point time_EndRos2OrbProcKF = std::chrono::steady_clock::now();

    // Calculate difference
    double timeRos2OrbProcKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndRos2OrbProcKF - time_StartRos2OrbProcKF).count();
    vdRos2OrbProcKF_ms.push_back(timeRos2OrbProcKF);
}



void KeyFrameSubscriber::ProcessNewKeyFrame()
{


    //std::cout << "KeyFrameSubscriber :: Processing new KF, KFs in Queue=" << KeyFramesInQueue() << std::endl;
    orbslam3_interfaces::msg::KeyFrame::SharedPtr pRosKF = static_cast<orbslam3_interfaces::msg::KeyFrame::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewRosKFs);
        //if(mpAtlas->GetCurrentMap()->KeyFramesInMap() > 10)
        //{
        //    pRosKF = mlpRosKeyFrameQueue.back();
        //    mlpRosKeyFrameQueue.pop_back();
        //}
        //else 
        //{
        pRosKF = mlpRosKeyFrameQueue.front();
        mlpRosKeyFrameQueue.pop_front();

        //}

        std::cout << "processing new KF=" << pRosKF->mn_id << "." << std::endl;
    }
  
    if(!pRosKF)
    {
        std::cout << "- New KeyFrame is null. Returning...";

        return;
    }

    
    double timeSinceReset = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - mpObserver->GetLastResetTime()).count();
    if(timeSinceReset < 50)
        return;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartRos2OrbProcKF = std::chrono::steady_clock::now();
    std::chrono::system_clock::time_point time_Start = std::chrono::system_clock::now();
    vtTimes.push_back(time_Start);
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPrepDataKF = std::chrono::steady_clock::now();


    // Get maps and the map where KF is
    std::map<unsigned long int, ORB_SLAM3::Map*>& mMaps = mpObserver->GetAllMaps();
    ORB_SLAM3::Map* pCurrentMap = mMaps[pRosKF->mp_map_id];

    if(!pCurrentMap)
    {
        mpObserver->UpdateLastResetTime();
        mpAtlas->CreateNewMap();
        pCurrentMap = mpAtlas->GetCurrentMap(); 
        pCurrentMap->attachDistributor(mpObserver);
        mpLocalMapper->Release();
    } 
    //else if(pCurrentMap && pCurrentMap != mpAtlas->GetCurrentMap() && pCurrentMap->GetId() < mpAtlas->GetCurrentMap()->GetId()) 
    //{
    //  //mpAtlas->ChangeMap(pCurrentMap);
    //  //pCurrentMap->SetCurrentMap();
    //  pRosKF->mp_map_id = pCurrentMap->GetId(); //mpAtlas->GetCurrentMap()->GetId();
    //  pCurrentMap = mpAtlas->GetCurrentMap();
    //}
    
    //if(pCurrentMap->KeyFramesInMap() >= 10 && mpObserver->HasKeyFrameBeenErased(pRosKF->mn_id))
    //    return;

    // Create map data structures for postloads
    std::map<std::string, ORB_SLAM3::MapPoint*>& mFusedMPs = mpObserver->GetAllMapPoints(); 
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*>& mFusedKFs = mpObserver->GetAllKeyFrames(); 
    std::vector<bool> mvbNewMPs;
    mvbNewMPs.reserve(pRosKF->mvp_map_points.size());
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    
    //std::vector<ORB_SLAM3::KeyFrame*> mvpKFs = pCurrentMap->GetAllKeyFrames(); 
    //for(int i=0;i<mvpKFs.size();++i)
    //{
    //  ORB_SLAM3::KeyFrame* pKFi = mvpKFs[i];
    //  if(!pKFi)
    //      continue;
    //  mFusedKFs[pKFi->mnId] = pKFi;
    //}

    //mFusedKFs = mpObserver->GetAllKeyFrames();

    vnKFAmount.push_back(mFusedKFs.size());

    //mFusedMPs = mpObserver->GetAllMapPoints();
    //std::vector<ORB_SLAM3::MapPoint*> mvpMPs = pCurrentMap->GetAllMapPoints(); 

    //for(int i=0;i<mvpMPs.size();++i)
    //{
    //  ORB_SLAM3::MapPoint* pMPi = mvpMPs[i];
    //  if(!pMPi)
    //      continue;
    //  mFusedMPs[pMPi->mstrHexId] = pMPi;
    //}

    vnMPAmount.push_back(mFusedMPs.size());

    for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas->GetAllCameras())
    {
      mCameras[pCam->GetId()] = pCam;
    }
    
    // End of timer
    std::chrono::steady_clock::time_point time_EndPrepDataKF = std::chrono::steady_clock::now();
    double timePrepDataKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPrepDataKF - time_StartPrepDataKF).count();
    vdPrepDataKF_ms.push_back(timePrepDataKF);
    //std::cout << "Got a new keyframe. 1. Existing data is collected." << std::endl;
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartConvKF = std::chrono::steady_clock::now();
    
    bool mbNewKF = true;

    //timeSinceReset = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - mpObserver->GetLastResetTime()).count();
    //if(timeSinceReset < 50)
    //    return;

    // Convert ros msg to ORB_SLAM3 Objects
    ORB_SLAM3::KeyFrame* tempKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL); //mpObserver->ConvertKeyFrame(pRosKF, mMaps);
    ORB_SLAM3::KeyFrame* mpCopyKF = mpObserver->GetKeyFrame(pRosKF->mn_id);//mFusedKFs[pRosKF->mn_id];
    
    if(mpCopyKF) //mFusedKFs.find(pRosKF->mn_id) != mFusedKFs.end())
    {
      tempKF = new ORB_SLAM3::KeyFrame(*mpCopyKF);
      mpObserver->ConvertKeyFrame(pRosKF, tempKF);
      mbNewKF=false;
      //mvbNewKF.push_back(false);

    } else {

      ORB_SLAM3::KeyFrame* mpExistingKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
      tempKF = mpObserver->ConvertKeyFrame(pRosKF, mpExistingKF);
      //mvbNewKF.push_back(true);
      //mpObserver->AddKeyFrame(tempKF);
      //mFusedKFs[tempKF->mnId] = tempKF;
    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndConvKF = std::chrono::steady_clock::now();
    double timeConvKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvKF - time_StartConvKF).count();
    vdRos2OrbConvKF_ms.push_back(timeConvKF);


    std::vector<ORB_SLAM3::MapPoint*> mvpTempMPs;
    mvpTempMPs.reserve(pRosKF->mvp_map_points.size());

    for(size_t i=0;i<pRosKF->mvp_map_points.size();++i)
    {
        unsigned long int mnId = pRosKF->mvp_map_points[i].mn_id;
        mpObserver->UpdateMaxMPId(mnId);
        
    }

    mpObserver->UpdateMaxMPId(ORB_SLAM3::MapPoint::nNextId);

    ORB_SLAM3::MapPoint::nNextId=mpObserver->GetMaxMPId()+1;
    std::cout << "max mp updated..." << std::endl;

    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartConvMP = std::chrono::steady_clock::now();
    
    //std::cout << mMapMPs.size() << " ------------------------------------" << std::endl;
    int mnTaskModule = mpObserver->GetTaskModule();
    int updates=0;
    int news=0;
    for(int i=0; i<pRosKF->mvp_map_points.size(); ++i)
    {
        //if(pCurrentMap->KeyFramesInMap() >= 10 && mpObserver->HasMapPointBeenErased(pRosKF->mvp_map_points[i].m_str_hex_id))
        //    continue;
        //if(pRosKF->mvp_map_points[i].mn_last_module > mnTaskModule || mFusedMPs.find(pRosKF->mvp_map_points[i].m_str_hex_id) == mFusedMPs.end()) 
        //{
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(pRosKF->mvp_map_points[i]);

        mpRosMP->mp_map_id = mpAtlas->GetCurrentMap()->GetId();
        ORB_SLAM3::MapPoint* tempMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
        ORB_SLAM3::MapPoint* mpCopyMP = mpObserver->GetMapPoint(mpRosMP->m_str_hex_id); //mFusedMPs[mpRosMP->m_str_hex_id];
        if(mpCopyMP)//mFusedMPs.find(mpRosMP->m_str_hex_id) != mFusedMPs.end())
        {
          ORB_SLAM3::MapPoint* mpExistingMP = new ORB_SLAM3::MapPoint(*mpCopyMP);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpExistingMP);
          mpExistingMP->SetWorldPos(tempMP->GetWorldPos());
          mpExistingMP->SetNormalVector(tempMP->GetNormal());

          if(mMaps[mpRosMP->mp_map_id])
              mpExistingMP->UpdateMap(mMaps[mpRosMP->mp_map_id]);
          mvbNewMPs.emplace_back(false);
          updates++;

        } else {

          ORB_SLAM3::MapPoint* mpExistingMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpExistingMP);
          mvbNewMPs.emplace_back(true);
          news++;
          mpObserver->AddMapPoint(tempMP);
          //mFusedMPs[tempMP->mstrHexId] = tempMP;
        }

        //mMapMPs[tempMP->mstrHexId] = tempMP;
        mvpTempMPs.emplace_back(tempMP);
            //ORB_SLAM3::MapPoint* tempMPi = mpObserver->ConvertMapPoint(std::make_shared<orbslam3_interfaces::msg::MapPoint>(pRosKF->mvp_map_points[i]), mMaps);
            //if(tempMPi)
            //{
            //    mMapMPs[tempMPi->mstrHexId] = mFusedMPs[tempMPi->mstrHexId];
            //    mvpTempMPs.push_back(tempMPi);
            //    if(mFusedMPs.find(tempMPi->mstrHexId) == mFusedMPs.end())
            //        mFusedMPs[tempMPi->mstrHexId] = tempMPi;
            //}
        //}
    }

    vnNewMPAmount.push_back(news);
    vnUpdateMPAmount.push_back(updates);
    // End of timer
    std::chrono::steady_clock::time_point time_EndConvMP = std::chrono::steady_clock::now();
    double timeConvMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvMP - time_StartConvMP).count();
    vdRos2OrbConvMP_ms.push_back(timeConvMP);
    
    std::cout << "conversions done..." << std::endl;
    //std::cout << "Got a new keyframe. 2. New data is fused with old data." << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPostLoadKF = std::chrono::steady_clock::now();

    // Postloads -> Reassign pointers etc to temporary data
    bool bKFUnprocessed = false;
    
    mpObserver->LoadKeyFrame(tempKF, mCameras);

    // End of timer
    std::chrono::steady_clock::time_point time_EndPostLoadKF = std::chrono::steady_clock::now();

    // Calculate difference
    double timePostLoadKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPostLoadKF - time_StartPostLoadKF).count();
    vdPostLoadKF_ms.push_back(timePostLoadKF);

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPostLoadMP = std::chrono::steady_clock::now();
    
    for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
        mpObserver->LoadMapPoint(tempMP);
    }

    std::cout << "postloads done..." << std::endl;
    // End of timer
    std::chrono::steady_clock::time_point time_EndPostLoadMP = std::chrono::steady_clock::now();

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartInjectKF = std::chrono::steady_clock::now();

    // -------------------- From here on we have unprocessed data -----------------------
    // Here the data is injected into existing KF or the same one is returned. Function handles pointers etc.
    ORB_SLAM3::KeyFrame* pKF; 
    if(!mbNewKF)
    {
        ORB_SLAM3::KeyFrame* mpExistingKF = mpObserver->GetKeyFrame(tempKF->mnId);//mFusedKFs[tempKF->mnId];
        pKF = mpObserver->InjectKeyFrame(tempKF, mpExistingKF, pRosKF->from_module_id);
        //pKF = mpObserver->InjectKeyFrame(tempKF, mpExistingKF);
        //std::cout << pKF << "," << mFusedKFs[tempKF->mnId] << std::endl;
    }
    else
    {
        pKF = tempKF;

    }
    //ORB_SLAM3::KeyFrame* pKF = mpObserver->InjectKeyFrame(tempKF, mFusedKFs);

    // End of timer
    std::chrono::steady_clock::time_point time_EndInjectKF = std::chrono::steady_clock::now();

    // Calculate difference
    double timeInjectKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndInjectKF - time_StartInjectKF).count();
    vdInjectKF_ms.push_back(timeInjectKF);

    //mFusedKFs[pKF->mnId] = pKF;
    
    //std::cout << "Got a new keyframe. 3. KF PostLoad, and injection is completed." << std::endl;



    // Calculate difference
    double timePostLoadMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPostLoadMP - time_StartPostLoadMP).count();
    vdPostLoadMP_ms.push_back(timePostLoadMP);
    
    //std::cout << "Got a new keyframe. 4. All MP PostLoads Completed." << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartInjectMP = std::chrono::steady_clock::now();
    
    //for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    for(size_t i=0;i<mvpTempMPs.size();++i)
    {
      //ORB_SLAM3::MapPoint* tempMP = mvpTempMPs[i];
      //if(mvbNewMPs[i])
      //{
      //    mpAtlas->AddMapPoint(tempMP);
      //}
      //else
      //{
      //    ORB_SLAM3::MapPoint* mpExistingMP = mFusedMPs[tempMP->mstrHexId];
      //    mpExistingMP->UpdateMapPoint(*tempMP);
      //    //std::cout << "MPi addrs: " << mpExistingMP << "," << mFusedMPs[tempMP->mstrHexId] << std::endl;
      //    delete tempMP;
      //}
      ORB_SLAM3::MapPoint* tempMP = mvpTempMPs[i];
      bool mbNew = mvbNewMPs[i];
      if(!tempMP)
        continue;

      ORB_SLAM3::MapPoint* mpExistingMP = mpObserver->GetMapPoint(tempMP->mstrHexId); //mFusedMPs[mpRosMP->m_str_hex_id];
      if(!mpExistingMP)
      {
          mpExistingMP=static_cast<ORB_SLAM3::MapPoint*>(NULL);
          mbNew = true;
      }
      if(pKF)
          tempMP->SetReferenceKeyFrame(pKF);
      mpObserver->InjectMapPoint(tempMP, mpExistingMP, mbNew);
        //mpObserver->InjectMapPoint(tempMP, mMapMPs);
    }

    std::cout << "injections done..." << std::endl;
    // End of timer
    std::chrono::steady_clock::time_point time_EndInjectMP = std::chrono::steady_clock::now();

    // Calculate difference
    double timeInjectMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndInjectMP - time_StartInjectMP).count();
    vdInjectMP_ms.push_back(timeInjectMP);
    // -------------------- To here -----------------------
    
    //std::cout << "Got a new keyframe. 5. All MPs are injected to ORB_SLAM3." << std::endl;
    
    if(mpObserver->mnLastKFId < tempKF->mnId)
        mpObserver->mpLastKeyFrame = tempKF;
    
    // If KF exists, forward it to target
    if(pKF)
    {
        //if(pKF->isBad())
        //    pKF->SetBadFlag();
        // Update reference for all modules
        //if(pKF->GetMap())
        //    std::cout << " --- pKF->Map=" << pKF->GetMap()->GetId() << ", pRosKF->mp_map_id=" << pRosKF->mp_map_id << std::endl;
        mpObserver->ForwardKeyFrameToTarget(pKF, pRosKF->from_module_id, mbNewKF);
    }
    
    //std::cout << "Got a new keyframe. 6. KF is forwarded to the target. KF is COMPLETE." << std::endl;

    std::cout << "ending process..." << std::endl;
    // End of timer
    std::chrono::steady_clock::time_point time_EndRos2OrbProcKF = std::chrono::steady_clock::now();

    // Calculate difference
    double timeRos2OrbProcKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndRos2OrbProcKF - time_StartRos2OrbProcKF).count();
    vdRos2OrbProcKF_ms.push_back(timeRos2OrbProcKF);
}


void KeyFrameSubscriber::InsertNewKeyFrame(orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr& pRosKFUpdate)
{

    //if(mpObserver->GetTaskModule()==1 && mpObserver->GetTaskModule()!=pRosKF->target)
    //    return;

    //if(mpObserver->GetTaskModule()==3 && pRosKF->target != mpObserver->GetTaskModule())
    //    return;

    //if(KeyFramesInQueue() > 10)
    //    return;
    
    double timeSinceReset = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - mpObserver->GetLastResetTime()).count();
    if(timeSinceReset < 50)
        return;

    //std::cout << "***** GOT KF UPDATE ******" << std::endl;
    {
        unique_lock<mutex> lock(mMutexNewRosKFs);
        //mlpRosKeyFrameUpdateQueue.push_back(pRosKFUpdate);
        //if(mpRosKeyFrameUpdateQueue[pRosKFUpdate->mn_id] && (mpRosKeyFrameUpdateQueue[pRosKFUpdate->mn_id]->mn_last_module > pRosKFUpdate->mn_last_module))
        //    return;
        
        mpRosKeyFrameUpdateQueue[pRosKFUpdate->mn_id] = pRosKFUpdate;
    }
}

void KeyFrameSubscriber::InsertNewKeyFrame(orbslam3_interfaces::msg::KeyFrame::SharedPtr pRosKF)
{

    //if(mpObserver->GetTaskModule()==1 && mpObserver->GetTaskModule()!=pRosKF->target)
    //    return;

    //if(mpObserver->GetTaskModule()==3 && pRosKF->target != mpObserver->GetTaskModule())
    //    return;

    //if(KeyFramesInQueue() > 10)
    //    return;
    
    double timeSinceReset = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::system_clock::now() - mpObserver->GetLastResetTime()).count();
    if(timeSinceReset < 50)
        return;



    if(mpLocalMapper->isStopped())
    {
        mpAtlas->GetCurrentMap()->EraseKeyFrame(pRosKF->mn_id);
        return;
    }

    {
        unique_lock<mutex> lock(mMutexNewRosKFs);
        mlpRosKeyFrameQueue.push_back(pRosKF);
    }
    std::cout << "***** GOT KF ******" << std::endl;
}

void KeyFrameSubscriber::ResetQueue(const bool bAll)
{
  unique_lock<mutex> lock(mMutexNewRosKFs);

  if(bAll)
  {
      mlpRosKeyFrameQueue.clear();
      mlpRosKeyFrameUpdateQueue.clear();
      mpRosKeyFrameUpdateQueue.clear();
  } else 
  {
      mlpRosKeyFrameUpdateQueue.clear();
      mpRosKeyFrameUpdateQueue.clear();
  }

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
