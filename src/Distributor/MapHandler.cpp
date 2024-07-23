
#include "./MapHandler.hpp"
#include "./Observer.hpp"
#include "./KeyFrameSubscriber.hpp"
#include "../slam/slam-wrapper-node.hpp"
//#include "../slam/slam-wrapper-node.hpp"
#include "../Converter/KeyFrameConverter.hpp"
#include "../Converter/MapConverter.hpp"
//#include "orbslam3_interfaces/Converter.hpp"

//namespace TempDistributor {

MapHandler::MapHandler()
  : mbFinished(false), mbFinishRequested(false), mnMapFreq_ms(100), mnGlobalMapFreq_ms(100), mnPubIters(0), mnAtlasBatchNumber(0), 
    maxUpdateN(5), maxUpdateGlobalN(3)
{
  mpNewRosMap = std::shared_ptr<orbslam3_interfaces::msg::Map>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);
  mpNewRosAtlas = std::shared_ptr<orbslam3_interfaces::msg::Atlas>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);
  msLastMUStart = std::chrono::high_resolution_clock::now();
  msLastGlobalMUStart = std::chrono::high_resolution_clock::now();

}

MapHandler::~MapHandler()
{

}

// main function
void MapHandler::Run()
{
    std::cout << "MapHandler Thread has been started." << std::endl;

    mbFinished = false;
    
    while(1)
    {
      if(!mpLoopCloser->CheckIfRunning() && !mpLoopCloser->isRunningGBA()  && !mpLocalMapper->IsLMRunning())
      {
          // Local map publish/subscription 
          if(CheckPubLocalMaps())
              ProcessNewPubLocalMap();
          else if(CheckSubLocalMaps())
              ProcessNewSubLocalMap2();
          // Global map (Atlas - map merging, full map update, loop closing) publish/subscription 
          else if(CheckPubGlobalMaps())
              ProcessNewPubGlobalMap();
          else if(CheckSubGlobalMaps())
              ProcessNewSubGlobalMap2();

      }


      usleep(1000);
      if(CheckFinish())
          break;
    }

    SetFinish();
}


//bool MapHandler::CheckIfGMURunning()
//{
//
//}

void MapHandler::ProcessNewPubGlobalMap()
{
    //unique_lock<mutex> lock1(mMutexMapUpdate);
    //std::cout << "Processing new Global Map" << std::endl;
    std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate;
    {
        unique_lock<mutex> lock2(mMutexNewAtlas);
        mtAtlasUpdate = mlpAtlasPubQueue.front();
        //mlpAtlasPubQueue.pop_front();
    }
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartProcAtlas = std::chrono::steady_clock::now();
    vtTimesPubAtlas.push_back(time_StartProcAtlas);

    
    ORB_SLAM3::Map* mpCurrentMap = mpAtlas->GetCurrentMap();
    
    // Clear all updated data since we want to publish all KFs
    if(std::get<0>(mtAtlasUpdate) || std::get<1>(mtAtlasUpdate))
    {
        mpCurrentMap->ClearUpdatedKFIds();
        mpCurrentMap->ClearUpdatedMPIds();
    }
    
    std::vector<ORB_SLAM3::GeometricCamera*> mvpCameras = mpAtlas->GetAllCameras();
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras(mvpCameras.begin(), mvpCameras.end());
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPreSaveAtlas = std::chrono::steady_clock::now();

    mpCurrentMap->PreSave(mspCameras);
    
    // End of timer
    std::chrono::steady_clock::time_point time_EndPreSaveAtlas = std::chrono::steady_clock::now();
    double timePreSaveAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreSaveAtlas - time_StartPreSaveAtlas).count();
    vdPreSaveAtlas_ms.push_back(timePreSaveAtlas);

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartConvAtlas = std::chrono::steady_clock::now();

    // Create ros msg and convert map to ros
    orbslam3_interfaces::msg::Atlas::SharedPtr mRosAtlas = std::make_shared<orbslam3_interfaces::msg::Atlas>();


    // Start of a timer -------------
    //std::chrono::steady_clock::time_point time_StartConvMap = std::chrono::steady_clock::now();
    orbslam3_interfaces::msg::Map::SharedPtr mRosMap;
    

    //mRosAtlas.mp_current_map = Converter::MapConverter::OrbMapToRosMap(mpCurrentMap);
    

    {
        unique_lock<mutex> lock(mMutexUpdates);

        ORB_SLAM3::Map* pMap = mpAtlas->GetCurrentMap();
        if(msUpdatedGlobalKFs.size() > maxUpdateGlobalN)
        {
            std::set<unsigned long int> s;
            for(int i=0;i<maxUpdateGlobalN;i++)
            {
                s.insert(*msUpdatedGlobalKFs.rbegin());
                msUpdatedGlobalKFs.erase(*msUpdatedGlobalKFs.rbegin());
                //msErasedKFs.erase(*msUpdatedLocalKFs.rbegin());
            }

            mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap, s, msUpdatedGlobalMPs, msErasedKFs, msErasedMPs); 
            // Decrease the rate of publishing so that the network does not congest
            mnGlobalMapFreq_ms=100;
            maxUpdateGlobalN=10;
            ++mnAtlasBatchNumber;
        }
        else {
            mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap, msUpdatedGlobalKFs, msUpdatedGlobalMPs, msErasedKFs, msErasedMPs); 

            msUpdatedGlobalKFs.clear();
            msUpdatedGlobalMPs.clear();
            msErasedKFs.clear();
            msErasedMPs.clear();
            
            // Make next update instant
            mnGlobalMapFreq_ms=50;
            maxUpdateGlobalN=3;
            mRosAtlas->mb_last_batch = true;
            mnAtlasBatchNumber=0;
            mlpAtlasPubQueue.pop_front();
        }
    }
    //mRosMap.from_module_id = mnTaskModule; 

    mRosAtlas->mn_batch_number = mnAtlasBatchNumber;
    mRosAtlas->mp_current_map = *mRosMap;//Converter::MapConverter::OrbMapToRosMap(mpCurrentMap);
    
    // End of timer
    //std::chrono::steady_clock::time_point time_EndConvMap = std::chrono::steady_clock::now();
    //double timeConvMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvMap - time_StartConvMap).count();
    //vdOrb2RosConvMap_ms.push_back(timeConvMap);





    mRosAtlas->system_id = std::getenv("SLAM_SYSTEM_ID");
    mRosAtlas->from_module_id = mpObserver->GetTaskModule();
    mRosAtlas->mb_map_merge = std::get<0>(mtAtlasUpdate);
    mRosAtlas->mb_loop_closer = std::get<1>(mtAtlasUpdate);
    mRosAtlas->mv_merged_map_ids = std::get<2>(mtAtlasUpdate);
    

    // End of timer
    std::chrono::steady_clock::time_point time_EndConvAtlas = std::chrono::steady_clock::now();
    double timeConvAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvAtlas - time_StartConvAtlas).count();
    vdOrb2RosConvAtlas_ms.push_back(timeConvAtlas);


    pSLAMNode->publishAtlas(mRosAtlas);

    // All what has happened in between is irrelevant
    //{
    //    unique_lock<std::mutex> lock(mMutexNewMaps);
    //    mlpMapSubQueue.clear();
    //}


    // End of timer
    std::chrono::steady_clock::time_point time_EndProcAtlas = std::chrono::steady_clock::now();
    double timeProcAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcAtlas - time_StartProcAtlas).count();
    vdOrb2RosProcAtlas_ms.push_back(timeProcAtlas);
    msLastGlobalMUStart = std::chrono::high_resolution_clock::now();
}

void MapHandler::ProcessNewSubGlobalMap()
{

    orbslam3_interfaces::msg::Atlas::SharedPtr mpRosAtlas = static_cast<orbslam3_interfaces::msg::Atlas::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewAtlas);
        mpRosAtlas = mlpAtlasSubQueue.front();

        //mlpAtlasSubQueue.pop_front();
    }
    
    double timeSinceReset = mpObserver->TimeSinceReset();
    if(timeSinceReset < 200)
        return;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartRos2OrbProcAtlas = std::chrono::steady_clock::now();
    vtTimesSubAtlas.push_back(time_StartRos2OrbProcAtlas);

    orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = std::make_shared<orbslam3_interfaces::msg::Map>(mpRosAtlas->mp_current_map);
    
    // Create map data structures for postloads
    std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    std::unordered_map<std::string, ORB_SLAM3::MapPoint*> mMapMPs;
    std::vector<bool> mvbNewMPs;

    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mFusedKFs; 
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mMapKFs; 
    std::vector<bool> mvbNewKF;
    
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    // Get maps and the map where KF is
    ORB_SLAM3::Map* pMergeMap = static_cast<ORB_SLAM3::Map*>(NULL);
    ORB_SLAM3::Map* pCurrentMap = static_cast<ORB_SLAM3::Map*>(NULL);
    std::map<unsigned long int, ORB_SLAM3::Map*> mMaps = mpObserver->GetAllMaps();

    if(mpRosAtlas->mb_map_merge && mpRosAtlas->mv_merged_map_ids.size() == 2)
    {

        std::cout << "Got Atlas Update. 0. Merge map current map=" << mpRosAtlas->mv_merged_map_ids[1] << ", merge map=" << mpRosAtlas->mv_merged_map_ids[0] << std::endl;
        pCurrentMap = mMaps[mpRosAtlas->mv_merged_map_ids[1]];
        pMergeMap = mMaps[mpRosAtlas->mv_merged_map_ids[0]];

    } else {
        pCurrentMap = mMaps[mpRosMap->mn_id];
    }

    if(!pCurrentMap)
    {
        mpAtlas->CreateNewMap();
        pCurrentMap = mpAtlas->GetCurrentMap(); 
        pCurrentMap->attachDistributor(mpObserver);
    }
    
    std::vector<ORB_SLAM3::Map*> mvpAtlasMaps = mpAtlas->GetAllMaps();
    for(int k=0;k<mvpAtlasMaps.size();++k)
    {
        std::vector<ORB_SLAM3::KeyFrame*> mvpKFs = mvpAtlasMaps[k]->GetAllKeyFrames(); 
        for(int i=0;i<mvpKFs.size();++i)
        {
          ORB_SLAM3::KeyFrame* pKFi = mvpKFs[i];
          if(!pKFi)
              continue;
          mFusedKFs[pKFi->mnId] = pKFi;
        }

        std::vector<ORB_SLAM3::MapPoint*> mvpMPs = mvpAtlasMaps[k]->GetAllMapPoints(); 

        for(int i=0;i<mvpMPs.size();++i)
        {
          ORB_SLAM3::MapPoint* pMPi = mvpMPs[i];
          if(!pMPi)
              continue;
          mFusedMPs[pMPi->mstrHexId] = pMPi;
        }
    }
    
    //std::vector<ORB_SLAM3::KeyFrame*> mvpKFs = pCurrentMap->GetAllKeyFrames(); 
    //for(int i=0;i<mvpKFs.size();++i)
    //{
    //  ORB_SLAM3::KeyFrame* pKFi = mvpKFs[i];
    //  if(!pKFi)
    //      continue;
    //  mFusedKFs[pKFi->mnId] = pKFi;
    //}

    //mFusedKFs = mpObserver->GetAllKeyFrames();

    //vnKFAmount.push_back(mFusedKFs.size());

    //mFusedMPs = mpObserver->GetAllMapPoints();

    std::cout << "Got Atlas Update. 1. Maps collected." << std::endl;

    //if(!pCurrentMap)
    //{
    //    mpAtlas->CreateNewMap();
    //    pCurrentMap = mpAtlas->GetCurrentMap(); 
    //    mMaps[pCurrentMap->GetId()] = pCurrentMap;
    //}


    std::cout << "Got Atlas Update. 2. ORB_SLAM3 data collected." << std::endl;

    
    std::vector<ORB_SLAM3::KeyFrame*> mvpTempKFs;
    mvpTempKFs.resize(mpRosMap->msp_keyframes.size());
    std::vector<ORB_SLAM3::MapPoint*> mvpTempMPs;
    std::unordered_map<std::string, ORB_SLAM3::MapPoint*> mTempMPs;
    
    // Sort the keyframes from smallest id to largest
    // So that postloads should be done in order
    std::vector<orbslam3_interfaces::msg::KeyFrameUpdate> mvRosKF = mpRosMap->msp_keyframes;
    std::sort(mvRosKF.begin(), mvRosKF.end(), [](const orbslam3_interfaces::msg::KeyFrameUpdate& a, const orbslam3_interfaces::msg::KeyFrameUpdate& b) {
        return a.mn_id > b.mn_id;
    });
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartDataConvAtlas = std::chrono::steady_clock::now();

    double timeConvKFTotal = 0;
    // Loop through each KF
    // Convert to ORB
    // Add to mFusedKFs if not found -> if its not there then its a new object and can be used as it is for other objects
    // if its there, then use original pointer in postload and just update that object afterwards
    for(size_t i = 0; i<mvRosKF.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartConvKF = std::chrono::steady_clock::now();
        orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(mvRosKF[i]);
        
        ORB_SLAM3::KeyFrame* tempKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
        if(mFusedKFs.find(mpRosKF->mn_id) != mFusedKFs.end())
        {
          ORB_SLAM3::KeyFrame* mpCopyKF = mFusedKFs[mpRosKF->mn_id];
          tempKF = new ORB_SLAM3::KeyFrame(*mpCopyKF);
          mpObserver->ConvertKeyFrame(mpRosKF, tempKF);

          if(!tempKF)
              continue;
          mvbNewKF.push_back(false);

        } else {

          mpAtlas->GetCurrentMap()->EraseKeyFrame(mpRosKF->mn_id);
          //ORB_SLAM3::KeyFrame* mpExistingKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
          //// TODO: THIS IS A TEST WITH SMALLER MSG SIZE
          ////tempKF = mpObserver->ConvertKeyFrame(mpRosKF, mpExistingKF, mMaps);
          //mvbNewKF.push_back(true);
          //mFusedKFs[tempKF->mnId] = tempKF;
        }
        mvpTempKFs[i] = tempKF;
        
        std::chrono::steady_clock::time_point time_EndConvKF = std::chrono::steady_clock::now();
        double timeConvKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvKF - time_StartConvKF).count();
        timeConvKFTotal+=timeConvKF;


        // Do same stuff for each MP
        // Skip MPs which were already found in this map update (same objects in original map)
        for(size_t j = 0; j<mpRosKF->mvp_map_points.size(); ++j)
        {
            orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[j]);
            // This MP was already seen in the map, skip since both points to the same pointer in the original
            if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
                continue;
            
            ORB_SLAM3::MapPoint* tempMP = mFusedMPs[mpRosMP->m_str_hex_id]; //static_cast<ORB_SLAM3::MapPoint*>(NULL);
            if(tempMP)
            {
              //ORB_SLAM3::MapPoint* mpCopyMP = mFusedMPs[mpRosMP->m_str_hex_id];
              ORB_SLAM3::MapPoint* mpExistingMP = new ORB_SLAM3::MapPoint(*tempMP);
              tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpExistingMP);
              mvbNewMPs.push_back(false);

            } else {

              //ORB_SLAM3::MapPoint* mpExistingMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
              tempMP = mpObserver->ConvertMapPoint(mpRosMP, tempMP);
              mvbNewMPs.push_back(true);
              mFusedMPs[tempMP->mstrHexId] = tempMP;
            }

            mTempMPs[tempMP->mstrHexId] = tempMP;
            mvpTempMPs.push_back(tempMP);
        }
    }

    vdRos2OrbKFConvAtlas_ms.push_back(timeConvKFTotal);

    // There might be MPs which were not associated with KFs
    // Process those as well
    for(size_t i = 0; i<mpRosMap->msp_map_points.size(); ++i)
    {
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosMap->msp_map_points[i]);

        if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
            continue;

        ORB_SLAM3::MapPoint* tempMP = mFusedMPs[mpRosMP->m_str_hex_id]; //static_cast<ORB_SLAM3::MapPoint*>(NULL);
        if(tempMP)
        {
          //ORB_SLAM3::MapPoint* mpCopyMP = mFusedMPs[mpRosMP->m_str_hex_id];
          ORB_SLAM3::MapPoint* mpExistingMP = new ORB_SLAM3::MapPoint(*tempMP);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpExistingMP);
          mvbNewMPs.push_back(false);

        } else {

          //ORB_SLAM3::MapPoint* mpExistingMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, tempMP);
          mvbNewMPs.push_back(true);
          //std::cout << "Number of MPs before=" << mFusedMPs.size();
          //mpObserver->AddMapPoint(tempMP);
          mFusedMPs[tempMP->mstrHexId] = tempMP;
          //std::cout << ", after=" << mFusedMPs.size() << std::endl;
        }

        mTempMPs[tempMP->mstrHexId] = tempMP;
        mvpTempMPs.push_back(tempMP);
    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndDataConvAtlas = std::chrono::steady_clock::now();
    double timeDataConvAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndDataConvAtlas - time_StartDataConvAtlas).count();
    vdRos2OrbDataConvAtlas_ms.push_back(timeDataConvAtlas);


    std::cout << "Got Atlas Update. 3. New data fused with the existing." << std::endl;
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPostLoadKFAtlas = std::chrono::steady_clock::now();

    // PostLoad for all the KFs
    bool bKFUnprocessed = false;
    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];

      if(!tempKF)
          continue;

      tempKF->PostLoad(mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    }

    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];

      if(!tempKF)
          continue;

      tempKF->UpdateBestCovisibles();
      //tempKF->UpdateConnections();
    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndPostLoadKFAtlas = std::chrono::steady_clock::now();
    double timePostLoadKFAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPostLoadKFAtlas - time_StartPostLoadKFAtlas).count();
    vdPostLoadKFAtlas_ms.push_back(timePostLoadKFAtlas);

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPostLoadMPAtlas = std::chrono::steady_clock::now();

    // PostLoad for all the MPs
    //for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
      tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    }

    std::cout << "Got Atlas Update. 4. PostLoads (KF + MPs) completed." << std::endl;
    
    // End of timer
    std::chrono::steady_clock::time_point time_EndPostLoadMPAtlas = std::chrono::steady_clock::now();
    double timePostLoadMPAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPostLoadMPAtlas - time_StartPostLoadMPAtlas).count();
    vdPostLoadMPAtlas_ms.push_back(timePostLoadMPAtlas);
    //mpLocalMapper_->RequestStop();
    //// Wait till stopped
    //while(!mpLocalMapper_->isStopped())
    //{
    //    usleep(1000);
    //}

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartInjectMPAtlas = std::chrono::steady_clock::now();

    // Inject KFs
    //for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    for(size_t i = 0; i<mvpTempMPs.size();++i)//ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
      ORB_SLAM3::MapPoint* tempMP = mvpTempMPs[i];
      //ORB_SLAM3::MapPoint* mpExistingMP = mFusedMPs[tempMP->mstrHexId];
      //mpObserver->InjectMapPoint(tempMP, mpExistingMP, mvbNewMPs[i]);
      if(mvbNewMPs[i])
      {
          mpAtlas->AddMapPoint(tempMP);
      }
    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndInjectMPAtlas = std::chrono::steady_clock::now();
    double timeInjectMPAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndInjectMPAtlas - time_StartInjectMPAtlas).count();
    vdInjectMPAtlas_ms.push_back(timeInjectMPAtlas);
    
    std::cout << "Got Atlas Update. 5. MPs injected into ORB_SLAM3" << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartInjectKFAtlas = std::chrono::steady_clock::now();
    
    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
      if(!tempKF)
          continue;

      if(!mvbNewKF[i])
      {
          ORB_SLAM3::KeyFrame* mpExistingKF = mFusedKFs[tempKF->mnId];
          ORB_SLAM3::KeyFrame* pKF = mpObserver->InjectKeyFrame(tempKF, mpExistingKF, mpRosAtlas->from_module_id);
          
          if(pKF)
              mpObserver->ForwardKeyFrameToTarget(pKF, mpRosAtlas->from_module_id);
      }
      // TODO: THIS IS A TEST WITH SMALLER MSG SIZE
      //else 
      //{
      //    if(tempKF)
      //    {
      //        mpObserver->ForwardKeyFrameToTarget(tempKF, mpRosAtlas->from_module_id);

      //        //mpKeyFrameDB->erase(tempKF);
      //        //mpKeyFrameDB->add(tempKF);

      //    }
      //}

    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndInjectKFAtlas = std::chrono::steady_clock::now();
    double timeInjectKFAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndInjectKFAtlas - time_StartInjectKFAtlas).count();
    vdInjectKFAtlas_ms.push_back(timeInjectKFAtlas);
    
    std::cout << "Got Atlas Update. 6. KFs injected into ORB_SLAM3" << std::endl;
    
    //mpLocalMapper_->Release();
    // Remove KFs and MPs
    //for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
    //{
    //    unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
    //    ORB_SLAM3::KeyFrame* pEraseKF=mFusedKFs[mnId];
    //    if(pEraseKF)
    //      mpKeyFrameDB->erase(pEraseKF);
    //        pEraseKF->SetBadFlag();
    //}

    for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
    {
      std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
        ORB_SLAM3::MapPoint* pEraseMP=mFusedMPs[mnId];
        if(pEraseMP)
        {
            mpObserver->EraseMapPoint(pEraseMP);
            pEraseMP->SetBadFlag();
        }
    }
    
    std::cout << "Got Atlas Update. 7. KFs and MPs removed" << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartUpdateAtlas = std::chrono::steady_clock::now();

    // Finally update Map object 
    // First create temporary Map from ROS data
    // Then Inject the data into real map (do not update KFs and MPs since those are already added)
    ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);
    tempMap->PostLoad(mpKeyFrameDB, pSLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    mpObserver->InjectMap(tempMap, pCurrentMap, mpRosMap->from_module_id);
    
    // End of timer
    std::chrono::steady_clock::time_point time_EndUpdateAtlas = std::chrono::steady_clock::now();
    double timeUpdateAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateAtlas - time_StartUpdateAtlas).count();
    vdUpdateAtlas_ms.push_back(timeUpdateAtlas);

    //std::cout << "Got Atlas Update. 8. Map Injection completed" << std::endl;

    if(mpRosAtlas->mb_map_merge && pMergeMap)
    {
        std::cout << "Got Atlas Update. 8. Remove merged map." << std::endl;
        pCurrentMap->SetCurrentMap();
        mpAtlas->SetMapBad(pMergeMap);
        mpAtlas->RemoveBadMaps();
    }

    // All what has happened in between is irrelevant
    {
        unique_lock<std::mutex> lock(mMutexNewMaps);
        unique_lock<mutex> lock2(mMutexNewAtlas);
        // This removes new subscription maps (LM)
        //msErasedKFs.clear();
        //msErasedMPs.clear();
        //msUpdatedLocalKFs.clear();
        //msUpdatedLocalMPs.clear();

        //mlpMapPubQueue.clear();
        mlpAtlasSubQueue.clear();
    }

    std::cout << "Got Atlas Update. 9. Bad maps are removed and GrabAtlas COMPLETE." << std::endl;

    // End of timer
    std::chrono::steady_clock::time_point time_EndRos2OrbProcAtlas = std::chrono::steady_clock::now();

    // Calculate difference
    double timeRos2OrbProcAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndRos2OrbProcAtlas - time_StartRos2OrbProcAtlas).count();
    vdRos2OrbProcAtlas_ms.push_back(timeRos2OrbProcAtlas);
}

void MapHandler::ProcessNewPubLocalMap()
{

    //unique_lock<mutex> lock1(mMutexMapUpdate);
    //std::cout << "Processing new Local Map" << std::endl;
    ORB_SLAM3::Map* pMap = static_cast<ORB_SLAM3::Map*>(NULL);
    {
        unique_lock<mutex> lock2(mMutexNewMaps);
        pMap = mpAtlas->GetCurrentMap(); //mlpMapPubQueue.front();
        //mlpMapPubQueue.pop_front();
    }

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartOrb2RosProcMap = std::chrono::steady_clock::now();
    vtTimesPubMap.push_back(time_StartOrb2RosProcMap);

    std::vector<ORB_SLAM3::GeometricCamera*> mvpCameras = mpAtlas->GetAllCameras();
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras(mvpCameras.begin(), mvpCameras.end());
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPreSaveMap = std::chrono::steady_clock::now();

    pMap->PreSave(mspCameras);

    // End of timer
    std::chrono::steady_clock::time_point time_EndPreSaveMap = std::chrono::steady_clock::now();
    double timePreSaveMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreSaveMap - time_StartPreSaveMap).count();
    vdPreSaveMap_ms.push_back(timePreSaveMap);
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartConvMap = std::chrono::steady_clock::now();
    orbslam3_interfaces::msg::Map::SharedPtr mRosMap;
    {
        unique_lock<mutex> lock(mMutexUpdates);
        std::cout << "msUpdatedLocalKFs.size()=" << msUpdatedLocalKFs.size() << ", maxUpdateN=" << maxUpdateN << ", mnPubIters=" << mnPubIters << ", mnMapFreq_ms=" << mnMapFreq_ms << std::endl;
        if(msUpdatedLocalKFs.size() > maxUpdateN)
        {
            std::set<unsigned long int> s;
            // First 20 kfs taken from covisible keyframes (KFs which needs the fastest updates)
            if(mnPubIters==0) //&& mpObserver->mpLastKeyFrame)
            {
                //std::vector<ORB_SLAM3::KeyFrame*> vpConnected = mpObserver->mpLastKeyFrame->GetVectorCovisibleKeyFrames();
                //int iters = maxUpdateN;
                //if(vpConnected.size() < maxUpdateN)
                //    iters = vpConnected.size();
                //    
                //for(int i=0;i<iters;i++)
                //{
                //    ORB_SLAM3::KeyFrame* pConnKF = vpConnected[i];
                //    if(!pConnKF)
                //        continue;
                //    if(msUpdatedLocalKFs.find(pConnKF->mnId) != msUpdatedLocalKFs.end())
                //    {
                //        s.insert(pConnKF->mnId);
                //        msUpdatedLocalKFs.erase(pConnKF->mnId);
                //        msErasedKFs.erase(pConnKF->mnId);
                //    }
                //}
                for(int i=0;i<maxUpdateN;i++)
                {
                    s.insert(*msUpdatedLocalKFs.rbegin());
                    msUpdatedLocalKFs.erase(*msUpdatedLocalKFs.rbegin());
                    //msErasedKFs.erase(*msUpdatedLocalKFs.rbegin());
                }

            }
            // Others can be updated in order
            else {
                for(int i=0;i<maxUpdateN;i++)
                {
                    s.insert(*msUpdatedLocalKFs.rbegin());
                    msUpdatedLocalKFs.erase(*msUpdatedLocalKFs.rbegin());
                    //msErasedKFs.erase(*msUpdatedLocalKFs.rbegin());
                }
            }

            std::cout << "before conversion" << std::endl;
            mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap, s, msUpdatedLocalMPs, msErasedKFs, msErasedMPs); 
            if(mnPubIters==0)
                mRosMap->mb_first_batch = true;
            std::cout << "after conversion" << std::endl;
            // Decrease the rate of publishing so that the network does not congest
            mnMapFreq_ms=100;
            maxUpdateN=10;
            mnPubIters++;
        }
        else {
            mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap, msUpdatedLocalKFs, msUpdatedLocalMPs, msErasedKFs, msErasedMPs); 

            if(mnPubIters == 0)
                mRosMap->mb_first_batch = true;

            msUpdatedLocalKFs.clear();
            msUpdatedLocalMPs.clear();
            msErasedKFs.clear();
            msErasedMPs.clear();
            
            // Make next update instant
            maxUpdateN=5;
            mnMapFreq_ms=50;
            mnPubIters=0;
        }
    }
    //mRosMap.from_module_id = mnTaskModule; 

    
    // End of timer
    std::chrono::steady_clock::time_point time_EndConvMap = std::chrono::steady_clock::now();
    double timeConvMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvMap - time_StartConvMap).count();
    vdOrb2RosConvMap_ms.push_back(timeConvMap);

    mRosMap->from_module_id = mpObserver->GetTaskModule();
    
    //pMap->ClearErasedData();
    //pMap->ClearUpdatedKFIds();
    //pMap->ClearUpdatedMPIds();

    pSLAMNode->publishMap(mRosMap);


    // End of timer
    std::chrono::steady_clock::time_point time_EndOrb2RosMap = std::chrono::steady_clock::now();
    double timeOrb2RosProcMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOrb2RosMap - time_StartOrb2RosProcMap).count();
    vdOrb2RosProcMap_ms.push_back(timeOrb2RosProcMap);
    
    msLastMUStart = std::chrono::high_resolution_clock::now();
}

void MapHandler::ProcessNewSubGlobalMap2()
{

    orbslam3_interfaces::msg::Atlas::SharedPtr mpRosAtlas = static_cast<orbslam3_interfaces::msg::Atlas::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewAtlas);
        mpRosAtlas = mpNewRosAtlas; //mlpAtlasSubQueue.front();

        //mlpAtlasSubQueue.pop_front();
        
    }
    
    double timeSinceReset = mpObserver->TimeSinceReset();
    if(timeSinceReset < 200)
        return;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartRos2OrbProcAtlas = std::chrono::steady_clock::now();
    vtTimesSubAtlas.push_back(time_StartRos2OrbProcAtlas);

    orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = std::make_shared<orbslam3_interfaces::msg::Map>(mpRosAtlas->mp_current_map);
    
    // Create map data structures for postloads
    std::unordered_map<std::string, ORB_SLAM3::MapPoint*> mMapMPs;
    std::vector<bool> mvbNewMPs;

    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mMapKFs; 
    std::vector<bool> mvbNewKF;
    
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    // Get maps and the map where KF is
    ORB_SLAM3::Map* pMergeMap = static_cast<ORB_SLAM3::Map*>(NULL);
    ORB_SLAM3::Map* pCurrentMap = static_cast<ORB_SLAM3::Map*>(NULL);
    std::map<unsigned long int, ORB_SLAM3::Map*>& mMaps = mpObserver->GetAllMaps();
    //std::vector<ORB_SLAM3::Map*> mvpMaps = mpAtlas->GetAllMaps();

    if(mpRosAtlas->mb_map_merge && mpRosAtlas->mv_merged_map_ids.size() == 2)
    {

        std::cout << "Got Atlas Update. 0. Merge map current map=" << mpRosAtlas->mv_merged_map_ids[1] << ", merge map=" << mpRosAtlas->mv_merged_map_ids[0] << std::endl;

        //for(const auto& mpMap : mvpMaps)
        //{
        //  if(mpMap->GetId() == mpRosAtlas->mv_merged_map_ids[1])
        //      pCurrentMap = mpMap;
        //  if(mpMap->GetId() == mpRosAtlas->mv_merged_map_ids[0])
        //      pMergeMap = mpMap;
        //}
        if(mMaps[mpRosAtlas->mv_merged_map_ids[1]])
            pCurrentMap = mMaps[mpRosAtlas->mv_merged_map_ids[1]];
        if(mMaps[mpRosAtlas->mv_merged_map_ids[0]])
            pMergeMap = mMaps[mpRosAtlas->mv_merged_map_ids[0]];

    } else {
        //for(const auto& mpMap : mvpMaps)
        //{
        //  if(mpMap->GetId() == mpRosMap->mn_id)
        //      pCurrentMap = mpMap;
        //}
        pCurrentMap = mMaps[mpRosMap->mn_id];
    }

    if(!pCurrentMap)
    {
        mpAtlas->CreateNewMap();
        pCurrentMap = mpAtlas->GetCurrentMap(); 
        pCurrentMap->attachDistributor(mpObserver);
    }
    
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*>& mFusedKFs = mpObserver->GetAllKeyFrames(); 
    std::map<std::string, ORB_SLAM3::MapPoint*>& mFusedMPs = mpObserver->GetAllMapPoints(); 
    //std::vector<ORB_SLAM3::Map*> mvpAtlasMaps = mpAtlas->GetAllMaps();
    //for(int k=0;k<mvpMaps.size();++k)
    //{
    //    std::vector<ORB_SLAM3::KeyFrame*> mvpKFs = mvpMaps[k]->GetAllKeyFrames(); 
    //    for(int i=0;i<mvpKFs.size();++i)
    //    {
    //      ORB_SLAM3::KeyFrame* pKFi = mvpKFs[i];
    //      if(!pKFi)
    //          continue;
    //      mFusedKFs[pKFi->mnId] = pKFi;
    //    }

    //    std::vector<ORB_SLAM3::MapPoint*> mvpMPs = mvpMaps[k]->GetAllMapPoints(); 

    //    for(int i=0;i<mvpMPs.size();++i)
    //    {
    //      ORB_SLAM3::MapPoint* pMPi = mvpMPs[i];
    //      if(!pMPi)
    //          continue;
    //      mFusedMPs[pMPi->mstrHexId] = pMPi;
    //    }
    //}
    
    //std::vector<ORB_SLAM3::KeyFrame*> mvpKFs = pCurrentMap->GetAllKeyFrames(); 
    //for(int i=0;i<mvpKFs.size();++i)
    //{
    //  ORB_SLAM3::KeyFrame* pKFi = mvpKFs[i];
    //  if(!pKFi)
    //      continue;
    //  mFusedKFs[pKFi->mnId] = pKFi;
    //}

    //mFusedKFs = mpObserver->GetAllKeyFrames();

    //vnKFAmount.push_back(mFusedKFs.size());

    //mFusedMPs = mpObserver->GetAllMapPoints();

    std::cout << "Got Atlas Update. 1. Maps collected." << std::endl;

    //if(!pCurrentMap)
    //{
    //    mpAtlas->CreateNewMap();
    //    pCurrentMap = mpAtlas->GetCurrentMap(); 
    //    mMaps[pCurrentMap->GetId()] = pCurrentMap;
    //}


    
    //mpLocalMapper_->Release();
    // Remove KFs and MPs
    for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
    {
        unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
        if(mpObserver->CheckIfKeyFrameExists(mnId))
        {
            ORB_SLAM3::KeyFrame* pEraseKF= mpObserver->GetKeyFrame(mnId);//mFusedKFs[mnId];
            //mpKeyFrameDB->erase(pEraseKF);
            pEraseKF->SetBadFlag();
            mpObserver->EraseKeyFrame(mnId);
        }
    }

    for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
    {
        std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
        if(mpObserver->CheckIfMapPointExists(mnId))
        {
            ORB_SLAM3::MapPoint* pEraseMP=mFusedMPs[mnId];
            pEraseMP->SetBadFlag();
            mpObserver->EraseMapPoint(mnId);
        }
    }



    std::cout << "Got Atlas Update. 7. KFs and MPs removed" << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartUpdateAtlas = std::chrono::steady_clock::now();

    // Finally update Map object 
    // First create temporary Map from ROS data
    // Then Inject the data into real map (do not update KFs and MPs since those are already added)
    ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);

    bool bKFUnprocessed = false;
    //tempMap->PostLoad(mpKeyFrameDB, pSLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    mpObserver->InjectMap(tempMap, pCurrentMap, mpRosMap->from_module_id);
    
    // End of timer
    std::chrono::steady_clock::time_point time_EndUpdateAtlas = std::chrono::steady_clock::now();
    double timeUpdateAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateAtlas - time_StartUpdateAtlas).count();
    vdUpdateAtlas_ms.push_back(timeUpdateAtlas);

    //std::cout << "Got Atlas Update. 8. Map Injection completed" << std::endl;

    if(mpRosAtlas->mb_map_merge && pMergeMap)
    {
        std::cout << "Got Atlas Update. 8. Remove merged map." << std::endl;
        pCurrentMap->SetCurrentMap();
        mpAtlas->SetMapBad(pMergeMap);
        mpAtlas->RemoveBadMaps();
    }

    // All what has happened in between is irrelevant
    {
        unique_lock<std::mutex> lock(mMutexNewMaps);
        unique_lock<mutex> lock2(mMutexNewAtlas);
        // This removes new subscription maps (LM)
        //msErasedKFs.clear();
        //msErasedMPs.clear();
        //msUpdatedLocalKFs.clear();
        //msUpdatedLocalMPs.clear();

        //mlpMapPubQueue.clear();
        //mlpAtlasSubQueue.clear();
        mpNewRosAtlas = std::shared_ptr<orbslam3_interfaces::msg::Atlas>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);
    }

    std::cout << "Got Atlas Update. 9. Bad maps are removed and GrabAtlas COMPLETE." << std::endl;

    // End of timer
    std::chrono::steady_clock::time_point time_EndRos2OrbProcAtlas = std::chrono::steady_clock::now();

    // Calculate difference
    double timeRos2OrbProcAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndRos2OrbProcAtlas - time_StartRos2OrbProcAtlas).count();
    vdRos2OrbProcAtlas_ms.push_back(timeRos2OrbProcAtlas);
}

void MapHandler::ProcessNewSubLocalMap2()
{
    orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = static_cast<orbslam3_interfaces::msg::Map::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewMaps);
        mpRosMap = mpNewRosMap;
        mpNewRosMap = std::shared_ptr<orbslam3_interfaces::msg::Map>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);
    }

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartRos2OrbProcMap = std::chrono::steady_clock::now();
    vtTimesSubMap.push_back(time_StartRos2OrbProcMap);
  

    // Get maps and the map where KF is
    // Get maps and the map where KF is
    std::map<unsigned long int, ORB_SLAM3::Map*> mMaps = mpObserver->GetAllMaps();
    ORB_SLAM3::Map* pCurrentMap = mMaps[mpRosMap->mn_id];

    if(!pCurrentMap)
    {
        mpAtlas->CreateNewMap();
        pCurrentMap = mpAtlas->GetCurrentMap(); 
        pCurrentMap->attachDistributor(mpObserver);
    }

    // Create map data structures for postloads
    //std::unordered_map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    std::map<std::string, ORB_SLAM3::MapPoint*>& mFusedMPs = mpObserver->GetAllMapPoints();
    std::unordered_map<std::string, ORB_SLAM3::MapPoint*> mMapMPs;

    std::map<long unsigned int, ORB_SLAM3::KeyFrame*>& mFusedKFs = mpObserver->GetAllKeyFrames();
    std::vector<bool> mvbNewKF;
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mMapKFs; 
    
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    
    //std::vector<ORB_SLAM3::KeyFrame*> mvpKFs = pCurrentMap->GetAllKeyFrames(); 
    //for(int i=0;i<mvpKFs.size();++i)
    //{
    //  ORB_SLAM3::KeyFrame* pKFi = mvpKFs[i];
    //  if(!pKFi)
    //      continue;
    //  mFusedKFs[pKFi->mnId] = pKFi;
    //}


    //vnKFAmount.push_back(mFusedKFs.size());

    //std::vector<ORB_SLAM3::MapPoint*> mvpMPs = pCurrentMap->GetAllMapPoints(); 

    //for(int i=0;i<mvpMPs.size();++i)
    //{
    //  ORB_SLAM3::MapPoint* pMPi = mvpMPs[i];
    //  if(!pMPi)
    //      continue;
    //  mFusedMPs[pMPi->mstrHexId] = pMPi;
    //}

    for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas->GetAllCameras())
    {
      mCameras[pCam->GetId()] = pCam;
    }


    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartUpdateMap = std::chrono::steady_clock::now();

    // Finally update Map object 
    // First create temporary Map from ROS data
    // Then Inject the data into real map (do not update KFs and MPs since those are already added)
    ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);

    bool bKFUnprocessed = false;
    tempMap->PostLoad(mpKeyFrameDB, pSLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    mpObserver->InjectMap(tempMap, pCurrentMap, mpRosMap->from_module_id);


    vnNumberOfKFsTotal.push_back(pCurrentMap->GetAllKeyFrames().size());
    vnNumberOfMPsTotal.push_back(pCurrentMap->GetAllMapPoints().size());

    // End of timer
    std::chrono::steady_clock::time_point time_EndUpdateMap = std::chrono::steady_clock::now();
    double timeUpdateMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateMap- time_StartUpdateMap).count();
    vdUpdateMap_ms.push_back(timeUpdateMap);

    //std::cout << "Got an update for a map. 7. Map is injected into ORB_SLAM3 and Map update is COMPLETE." << std::endl;


    // Remove KFs and MPs
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() > 10)
    {
        //if(mpObserver->GetTaskModule() != 3)
        //{
        for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
        {
            unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
            if(mpObserver->CheckIfKeyFrameExists(mnId))
            {
                ORB_SLAM3::KeyFrame* pEraseKF= mpObserver->GetKeyFrame(mnId);//mFusedKFs[mnId];
                //mpKeyFrameDB->erase(pEraseKF);
                pEraseKF->SetBadFlag();
                mpObserver->EraseKeyFrame(mnId);
            }
        }

        //}

        for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
        {
            std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
            if(mpObserver->CheckIfMapPointExists(mnId))
            {
                ORB_SLAM3::MapPoint* pEraseMP=mFusedMPs[mnId];
                pEraseMP->SetBadFlag();
                mpObserver->EraseMapPoint(mnId);
            }
        }

    }


}



void MapHandler::ProcessNewSubLocalMap()
{

    orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = static_cast<orbslam3_interfaces::msg::Map::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewMaps);
        mpRosMap = mlpMapSubQueue.front();

        mlpMapSubQueue.pop_front();
    }

    double timeSinceReset = mpObserver->TimeSinceReset();
    if(timeSinceReset < 200)
        return;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartRos2OrbProcMap = std::chrono::steady_clock::now();
    vtTimesSubMap.push_back(time_StartRos2OrbProcMap);
  

    // Get maps and the map where KF is
    // Get maps and the map where KF is
    std::map<unsigned long int, ORB_SLAM3::Map*> mMaps = mpObserver->GetAllMaps();
    ORB_SLAM3::Map* pCurrentMap = mMaps[mpRosMap->mn_id];

    if(!pCurrentMap)
    {
        mpAtlas->CreateNewMap();
        pCurrentMap = mpAtlas->GetCurrentMap(); 
        pCurrentMap->attachDistributor(mpObserver);
    }

    // Create map data structures for postloads
    //std::unordered_map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    std::unordered_map<std::string, ORB_SLAM3::MapPoint*> mMapMPs;

    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mFusedKFs; 
    std::vector<bool> mvbNewKF;
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mMapKFs; 
    
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpKFs = pCurrentMap->GetAllKeyFrames(); 
    for(int i=0;i<mvpKFs.size();++i)
    {
      ORB_SLAM3::KeyFrame* pKFi = mvpKFs[i];
      if(!pKFi)
          continue;
      mFusedKFs[pKFi->mnId] = pKFi;
    }

    //mFusedKFs = mpObserver->GetAllKeyFrames();

    //vnKFAmount.push_back(mFusedKFs.size());

    //mFusedMPs = mpObserver->GetAllMapPoints();
    std::vector<ORB_SLAM3::MapPoint*> mvpMPs = pCurrentMap->GetAllMapPoints(); 

    for(int i=0;i<mvpMPs.size();++i)
    {
      ORB_SLAM3::MapPoint* pMPi = mvpMPs[i];
      if(!pMPi)
          continue;
      mFusedMPs[pMPi->mstrHexId] = pMPi;
    }

    for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas->GetAllCameras())
    {
      mCameras[pCam->GetId()] = pCam;
    }
    
    //std::cout << "Got an update for a map. 1. Existing data collected." << std::endl;
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpTempKFs;
    //mvpTempKFs.resize(mpRosMap->msp_keyframes.size());
    std::vector<ORB_SLAM3::MapPoint*> mvpTempMPs;
    std::vector<bool> mvbNewMPs;
    std::unordered_map<std::string, ORB_SLAM3::MapPoint*> mTempMPs;
    
    // Sort the keyframes from smallest id to largest
    // So that postloads should be done in order
    std::vector<orbslam3_interfaces::msg::KeyFrameUpdate> mvRosKF = mpRosMap->msp_keyframes;
    std::sort(mvRosKF.begin(), mvRosKF.end(), [](const orbslam3_interfaces::msg::KeyFrameUpdate& a, const orbslam3_interfaces::msg::KeyFrameUpdate& b) {
        return a.mn_id > b.mn_id;
    });

    // Remove KFs and MPs
    //for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
    //{
    //    unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
    //    ORB_SLAM3::KeyFrame* pEraseKF=mFusedKFs[mnId];
    //    if(pEraseKF)
    //    {
    //        //mpKeyFrameDB->erase(pEraseKF);
    //        pEraseKF->SetBadFlag();
    //    }
    //}

    for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
    {
        std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
        ORB_SLAM3::MapPoint* pEraseMP=mFusedMPs[mnId];
        if(pEraseMP)
        {
            //mpObserver->EraseMapPoint(pEraseMP);
            pEraseMP->SetBadFlag();
        }
    }


    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartConvKFMP = std::chrono::steady_clock::now();

    double timeConvKFTotal = 0;
    double timeConvMPTotal = 0;

    int nTaskModule = mpObserver->GetTaskModule();

    // Loop through each KF
    // Convert to ORB
    // Add to mFusedKFs if not found -> if its not there then its a new object and can be used as it is for other objects
    // if its there, then use original pointer in postload and just update that object afterwards
    for(size_t i = 0; i<mvRosKF.size(); ++i)
    {
        orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(mvRosKF[i]);
        double timeConvKF=0;
        ORB_SLAM3::KeyFrame* tempKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
        if(mFusedKFs.find(mpRosKF->mn_id) != mFusedKFs.end())
        {
          ORB_SLAM3::KeyFrame* mpCopyKF = mFusedKFs[mpRosKF->mn_id];
          if(nTaskModule != 3 && mpCopyKF->GetLastModule() > mpRosKF->mn_last_module)
          {
             mpCopyKF->SetLastModule(mpObserver->GetTaskModule()); 
             continue;
          }
          tempKF = new ORB_SLAM3::KeyFrame(*mpCopyKF);
          std::chrono::steady_clock::time_point time_StartConvKF = std::chrono::steady_clock::now();
          mpObserver->ConvertKeyFrame(mpRosKF, tempKF);
          std::chrono::steady_clock::time_point time_EndConvKF = std::chrono::steady_clock::now();
          timeConvKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvKF - time_StartConvKF).count();
          if(!tempKF)
              continue;
          mvbNewKF.push_back(false);

        } else {

          mpAtlas->GetCurrentMap()->EraseKeyFrame(mpRosKF->mn_id);
          //ORB_SLAM3::KeyFrame* mpExistingKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
          //std::chrono::steady_clock::time_point time_StartConvKF = std::chrono::steady_clock::now();
          //// TODO: THIS IS A TEST WITH SMALLER MSG SIZE
          ////tempKF = mpObserver->ConvertKeyFrame(mpRosKF, mpExistingKF, mMaps);
          //std::chrono::steady_clock::time_point time_EndConvKF = std::chrono::steady_clock::now();
          //timeConvKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvKF - time_StartConvKF).count();
          //mvbNewKF.push_back(true);
          //mFusedKFs[tempKF->mnId] = tempKF;
        }
        mvpTempKFs.push_back(tempKF);
        
        //std::cout << "KF CONVERSION IN MAPHANDLER=" << timeConvKF << std::endl;
        timeConvKFTotal+=timeConvKF;


        // Do same stuff for each MP
        // Skip MPs which were already found in this map update (same objects in original map)
        for(size_t j = 0; j<mpRosKF->mvp_map_points.size(); ++j)
        {
            orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[j]);
            // This MP was already seen in the map, skip since both points to the same pointer in the original
            //if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
            //    continue;

            double timeConvMP=0;
            ORB_SLAM3::MapPoint* tempMP = mFusedMPs[mpRosMP->m_str_hex_id]; //static_cast<ORB_SLAM3::MapPoint*>(NULL);
            std::chrono::steady_clock::time_point time_StartConvMP = std::chrono::steady_clock::now();
            if(tempMP)
            {
              // Check if it was last updated in higher prio module.
              if(nTaskModule != 3  && tempMP->GetLastModule() > mpRosMP->mn_last_module)
              {
                 tempMP->SetLastModule(mpObserver->GetTaskModule()); 
                 continue;
              }
              //ORB_SLAM3::MapPoint* mpCopyMP = mFusedMPs[mpRosMP->m_str_hex_id];
              ORB_SLAM3::MapPoint* mpExistingMP = new ORB_SLAM3::MapPoint(*tempMP);
              tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpExistingMP);
              mvbNewMPs.push_back(false);

            } else {

              //ORB_SLAM3::MapPoint* mpExistingMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
              tempMP = mpObserver->ConvertMapPoint(mpRosMP, tempMP);
              mvbNewMPs.push_back(true);
              mFusedMPs[tempMP->mstrHexId] = tempMP;
            }

            mTempMPs[tempMP->mstrHexId] = tempMP;
            mvpTempMPs.push_back(tempMP);
            std::chrono::steady_clock::time_point time_EndConvMP = std::chrono::steady_clock::now();
            timeConvMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvMP - time_StartConvMP).count();
            timeConvMPTotal+=timeConvMP;
            
            //ORB_SLAM3::MapPoint* tempMP = ConvertMapPoint(mpRosMP, mMaps);
            //if(tempMP)
            //{
            //    mTempMPs[tempMP->mstrHexId] = tempMP;
            //    mvpTempMPs.push_back(tempMP);

            //    if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
            //        mFusedMPs[tempMP->mstrHexId] = tempMP;
            //    else
            //        mMapMPs[tempMP->mstrHexId] = mFusedMPs[tempMP->mstrHexId];

            //}
        }
    }



    vdRos2OrbKFConvMap_ms.push_back(timeConvKFTotal);

    
    //std::cout << "Got an update for a map. 2. Data fusion complete." << std::endl;

    // There might be MPs which were not associated with KFs
    // Process those as well
    for(size_t i = 0; i<mpRosMap->msp_map_points.size(); ++i)
    {
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosMap->msp_map_points[i]);

        //if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
        //    continue;

        double timeConvMP=0;
        std::chrono::steady_clock::time_point time_StartConvMP = std::chrono::steady_clock::now();
        ORB_SLAM3::MapPoint* tempMP = mFusedMPs[mpRosMP->m_str_hex_id]; //static_cast<ORB_SLAM3::MapPoint*>(NULL);
        
        if(tempMP)
        {
          // Check if it was last updated in higher prio module.
          if(nTaskModule != 3 && tempMP->GetLastModule() > mpRosMP->mn_last_module)
          {
             tempMP->SetLastModule(mpObserver->GetTaskModule()); 
             continue;
          }

          //ORB_SLAM3::MapPoint* mpCopyMP = mFusedMPs[mpRosMP->m_str_hex_id];
          ORB_SLAM3::MapPoint* mpExistingMP = new ORB_SLAM3::MapPoint(*tempMP);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpExistingMP);
          mvbNewMPs.push_back(false);

        } else {

          //ORB_SLAM3::MapPoint* mpExistingMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, tempMP);
          mvbNewMPs.push_back(true);
          //std::cout << "Number of MPs before=" << mFusedMPs.size();
          //mpObserver->AddMapPoint(tempMP);
          mFusedMPs[tempMP->mstrHexId] = tempMP;
          //std::cout << ", after=" << mFusedMPs.size() << std::endl;
        }

        mTempMPs[tempMP->mstrHexId] = tempMP;
        mvpTempMPs.push_back(tempMP);

        std::chrono::steady_clock::time_point time_EndConvMP = std::chrono::steady_clock::now();
        timeConvMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvMP - time_StartConvMP).count();
        timeConvMPTotal+=timeConvMP;
    }

    vdRos2OrbMPConvMap_ms.push_back(timeConvMPTotal);

    // End of timer
    std::chrono::steady_clock::time_point time_EndConvKFMP = std::chrono::steady_clock::now();
    double timeConvKFMP= std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvKFMP - time_StartConvKFMP).count();
    vdRos2OrbDataConvMap_ms.push_back(timeConvKFMP);

    //std::cout << "Got an update for a map. 3. MPs processed." << std::endl;
    
    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPostLoadKF = std::chrono::steady_clock::now();

    // PostLoad for all the KFs
    bool bKFUnprocessed = false;
    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
      if(!tempKF)
          continue;

      tempKF->PostLoad(mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    }

    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];

      if(!tempKF)
          continue;

      tempKF->UpdateBestCovisibles();
      //tempKF->UpdateConnections();
    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndPostLoadKF = std::chrono::steady_clock::now();
    double timePostLoadKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPostLoadKF - time_StartPostLoadKF).count();
    vdPostLoadKFMap_ms.push_back(timePostLoadKF);


    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartPostLoadMP = std::chrono::steady_clock::now();

    vnNumberOfKFs.push_back(mvRosKF.size());
    vnNumberOfMPs.push_back(mvpTempMPs.size());
    // PostLoad for all the MPs
    //for(std::unordered_map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
      //ORB_SLAM3::MapPoint* tempMP = it->second;
      tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndPostLoadMP = std::chrono::steady_clock::now();
    double timePostLoadMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPostLoadMP - time_StartPostLoadMP).count();
    vdPostLoadMPMap_ms.push_back(timePostLoadMP);
    
    //std::cout << "Got an update for a map. 4. PostLoads completed." << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartInjectMP = std::chrono::steady_clock::now();

    // Inject KFs
    //for(std::unordered_map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    for(size_t i = 0; i<mvpTempMPs.size();++i)//ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
      ORB_SLAM3::MapPoint* tempMP = mvpTempMPs[i];
      ORB_SLAM3::MapPoint* mpExistingMP = mFusedMPs[tempMP->mstrHexId];
      mpObserver->InjectMapPoint(tempMP, mpExistingMP, mvbNewMPs[i]);
      //std::cout << "MPi addrs: " << mpExistingMP << "," << mFusedMPs[tempMP->mstrHexId] << ", new=" << mvbNewMPs[i] << std::endl;
      //if(mvbNewMPs[i])
      //{
      //    mpAtlas->AddMapPoint(tempMP);
      //}
    }

    //vnNumberOfNewMapPoints.push_back(newMPs);
    //vnNumberOfUpdatedMapPoints.push_back(updatedMPs);

    // End of timer
    std::chrono::steady_clock::time_point time_EndInjectMP = std::chrono::steady_clock::now();
    double timeInjectMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndInjectMP - time_StartInjectMP).count();
    vdInjectMPMap_ms.push_back(timeInjectMP);

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartInjectKF = std::chrono::steady_clock::now();

    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
      if(!tempKF)
          continue;

      if(!mvbNewKF[i])
      {
          ORB_SLAM3::KeyFrame* mpExistingKF = mFusedKFs[tempKF->mnId];
          ORB_SLAM3::KeyFrame* pKF = mpObserver->InjectKeyFrame(tempKF, mpExistingKF, mpRosMap->from_module_id);
          //std::cout << "KF addrs: " << tempKF << "," << mFusedKFs[tempKF->mnId] << std::endl;
          //pKF->mnNextTarget=0;
          
          if(pKF)
              mpObserver->ForwardKeyFrameToTarget(pKF, mpRosMap->from_module_id);
      }
      // TODO: THIS IS A TEST WITH SMALLER MSG SIZE
      //else 
      //{
      //    if(tempKF)
      //        mpObserver->ForwardKeyFrameToTarget(tempKF, mpRosMap->from_module_id);

      //}

    }

    // End of timer
    std::chrono::steady_clock::time_point time_EndInjectKF = std::chrono::steady_clock::now();
    double timeInjectKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndInjectKF - time_StartInjectKF).count();
    vdInjectKFMap_ms.push_back(timeInjectKF);
    
    //std::cout << "Got an update for a map. 5. All the data is injected into ORB_SLAM3." << std::endl;


    //std::cout << "Got an update for a map. 6. Data which was deleted elsewhere is removed from the system." << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartUpdateMap = std::chrono::steady_clock::now();

    // Finally update Map object 
    // First create temporary Map from ROS data
    // Then Inject the data into real map (do not update KFs and MPs since those are already added)
    ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);
    tempMap->PostLoad(mpKeyFrameDB, pSLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    mpObserver->InjectMap(tempMap, pCurrentMap, mpRosMap->from_module_id);

    ORB_SLAM3::KeyFrame* pPlaceHolderKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
    mpTracker->UpdateReference(pPlaceHolderKF);

    vnNumberOfKFsTotal.push_back(pCurrentMap->GetAllKeyFrames().size());
    vnNumberOfMPsTotal.push_back(pCurrentMap->GetAllMapPoints().size());

    // End of timer
    std::chrono::steady_clock::time_point time_EndUpdateMap = std::chrono::steady_clock::now();
    double timeUpdateMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateMap- time_StartUpdateMap).count();
    vdUpdateMap_ms.push_back(timeUpdateMap);

    std::cout << "Got an update for a map. 7. Map is injected into ORB_SLAM3 and Map update is COMPLETE." << std::endl;

    // End of timer
    std::chrono::steady_clock::time_point time_EndRos2OrbProcMap = std::chrono::steady_clock::now();
    double timeRos2OrbProcMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndRos2OrbProcMap - time_StartRos2OrbProcMap).count();
    vdRos2OrbProcMap_ms.push_back(timeRos2OrbProcMap);
}


int MapHandler::GlobalMapsInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewMaps);
    return mlpAtlasPubQueue.size();
}

int MapHandler::LocalMapsInQueue()
{
    unique_lock<std::mutex> lock(mMutexNewMaps);
    return mlpMapPubQueue.size();
}

bool MapHandler::CheckPubLocalMaps()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    std::chrono::high_resolution_clock::time_point msLastMUStop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(msLastMUStop - msLastMUStart);
    auto dCount = duration.count();

    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() <= 2)
      return(msUpdatedLocalKFs.size() >= 2 && dCount > mnMapFreq_ms);
    else
      return(msUpdatedLocalKFs.size() >= 1 && dCount > mnMapFreq_ms);
}


bool MapHandler::CheckSubLocalMaps()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    return(mpNewRosMap != NULL);
}


bool MapHandler::CheckPubGlobalMaps()
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    std::chrono::high_resolution_clock::time_point msLastMUStop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(msLastMUStop - msLastGlobalMUStart);
    auto dCount = duration.count();

    return(!mlpAtlasPubQueue.empty() && dCount > mnGlobalMapFreq_ms);
}

bool MapHandler::CheckSubGlobalMaps()
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    return(mpNewRosAtlas != NULL && mpKeyFrameSubscriber->KeyFrameUpdatesInQueue() < 10);
}


void MapHandler::InsertNewPubLocalMap(ORB_SLAM3::Map* pMap)
{
    unique_lock<mutex> lock(mMutexNewMaps);

    //if(!mlpAtlasSubQueue.empty())
    //    return;
    
    //if(mpNewRosAtlas != NULL)
    //    return;

    std::cout << "***** SEND LOCAL MAP ******" << std::endl;

    std::set<unsigned long int> tempUpdatedKFs=pMap->GetUpdatedKFIds();
    std::set<std::string> tempUpdatedMPs=pMap->GetUpdatedMPIds();
    
    std::set<std::string> tempErasedMPs=pMap->GetErasedMPIds();
    std::set<unsigned long int > tempErasedKFs=pMap->GetErasedKFIds();

    {
        unique_lock<mutex> lock2(mMutexUpdates);
        msErasedKFs.insert(tempErasedKFs.begin(), tempErasedKFs.end());
        msErasedMPs.insert(tempErasedMPs.begin(), tempErasedMPs.end());
        
        msUpdatedLocalKFs.insert(tempUpdatedKFs.begin(), tempUpdatedKFs.end());
        msUpdatedLocalMPs.insert(tempUpdatedMPs.begin(), tempUpdatedMPs.end());
    }

    pMap->ClearUpdatedMPIds();
    pMap->ClearUpdatedKFIds();
    
    // Make next update instant
    mnMapFreq_ms=50;
    maxUpdateN=5;

    // Make next update instant
    //mnMapFreq_ms=100;

    //std::cout << "Insert new pub local map, updated KFs=" << msUpdatedLocalKFs.size() << ", MPs=" << msUpdatedLocalMPs.size() << std::endl;
    //mlpMapPubQueue.push_back(pMap);
}

void MapHandler::InsertNewSubLocalMap(orbslam3_interfaces::msg::Map::SharedPtr pRosMap)
{

    //if(!mlpAtlasSubQueue.empty())
    //    return;

    //if(mpNewRosAtlas != NULL)
    //    return;

    double timeSinceReset = mpObserver->TimeSinceReset();
    if(timeSinceReset < 200)
    {
        std::cout << "********************** time since reset < 200 ***********************" << std::endl;
        return;
    }
    
    std::cout << "***** GOT LOCAL MAP ******" << std::endl;
    
    //std::map<unsigned long int, orbslam3_interfaces::msg::KeyFrame::SharedPtr> mpKeyFrameQueue;
    //std::map<std::string, orbslam3_interfaces::msg::MapPoint::SharedPtr> mpMapPointQueue;
    //orbslam3_interfaces::msg::Map::SharedPtr mpNewRosMap;
    {
      unique_lock<mutex> lock(mMutexNewMaps);
      for(orbslam3_interfaces::msg::KeyFrameUpdate& pRosKF : pRosMap->msp_keyframes)
      {
        //if(!mpKeyFrameQueue[pRosKF.mn_id]) //|| (mpObserver && mpKeyFrameQueue[pRosKF.mn_id] && mpKeyFrameQueue[pRosKF.mn_id]->mn_last_module < mpObserver->GetTaskModule()))
            orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(pRosKF);
            mpRosKF->from_module_id = pRosMap->from_module_id;
            mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKF);
            //mpKeyFrameQueue[pRosKF.mn_id] = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(pRosKF);
      }

      //for(const orbslam3_interfaces::msg::MapPoint& pRosMP : pRosMap->msp_map_points)
      //{
      //  if(!mpMapPointQueue[pRosMP.m_str_hex_id]) //|| (mpObserver && mpMapPointQueue[pRosMP.m_str_hex_id] && mpMapPointQueue[pRosMP.m_str_hex_id]->mn_last_module < mpObserver->GetTaskModule()))
      //      mpMapPointQueue[pRosMP.m_str_hex_id] = std::make_shared<orbslam3_interfaces::msg::MapPoint>(pRosMP);
      //}

      pRosMap->msp_keyframes.clear();
      pRosMap->msp_map_points.clear();

      //if(pRosMap->mb_first_batch)
      mpNewRosMap = pRosMap;
    }

    //if(CheckSubLocalMaps())
    //{
    //  unique_lock<mutex> lock(mMutexNewMaps);
    //  mlpMapSubQueue.clear();
    //  std::cout << "Remove first map and replace it with the new one." << std::endl;
    //  mlpMapSubQueue.push_back(pRosMap);
    //} else {
    //  unique_lock<mutex> lock(mMutexNewMaps);
    //  std::cout << "No Sub local maps. Insert it to the queue. Queue size=" << mlpMapSubQueue.size() << std::endl;
    //  mlpMapSubQueue.push_back(pRosMap);

    //}
}

void MapHandler::InsertNewPubGlobalMap(std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate)
{
    unique_lock<mutex> lock(mMutexNewAtlas);

    std::cout << "***** SEND GLOBAL MAP ******" << std::endl;
    std::set<unsigned long int> tempUpdatedKFs;
    std::set<std::string> tempUpdatedMPs;

    for(const auto& pKF : mpAtlas->GetCurrentMap()->GetAllKeyFrames())
    {
        tempUpdatedKFs.insert(pKF->mnId);

        for(const auto& pMP : pKF->GetMapPoints())
        {
            tempUpdatedMPs.insert(pMP->mstrHexId);
        }
    }

    {
        unique_lock<mutex> lock2(mMutexUpdates);
        msErasedKFs = std::set<unsigned long int>();//.insert(tempErasedKFs.begin(), tempErasedKFs.end());
        msErasedMPs = std::set<std::string>();//.insert(tempErasedMPs.begin(), tempErasedMPs.end());
        
        msUpdatedGlobalKFs.insert(tempUpdatedKFs.begin(), tempUpdatedKFs.end());
        msUpdatedGlobalMPs.insert(tempUpdatedMPs.begin(), tempUpdatedMPs.end());
    }

    // Make next update instant
    mnGlobalMapFreq_ms=50;
    mnAtlasBatchNumber=0;
    maxUpdateGlobalN=3;
    
    mlpAtlasPubQueue.push_back(mtAtlasUpdate);
}

void MapHandler::InsertNewSubGlobalMap(orbslam3_interfaces::msg::Atlas::SharedPtr pRosAtlas)
{
    double timeSinceReset = mpObserver->TimeSinceReset();
    if(timeSinceReset < 200)
        return;

    std::cout << "***** GOT GLOBAL MAP, BATCH=" << pRosAtlas->mn_batch_number << " ******" << std::endl;
    // If loop closer is received, empty all map updates and just perform atlas update
    if(pRosAtlas->mn_batch_number == 0)//if(pRosAtlas->mb_loop_closer)
    {
      unique_lock<mutex> lock(mMutexNewMaps);
      unique_lock<mutex> lock2(mMutexNewAtlas);

      //mpLocalMapper->RequestStop();
      //while(!mpLocalMapper->isStopped())
      //    usleep(3000);

      {
          unique_lock<mutex> lock3(mMutexUpdates);
          msErasedMPs.clear();
          msErasedKFs.clear();
          msUpdatedLocalKFs.clear();
          msUpdatedLocalMPs.clear();
          mpKeyFrameSubscriber->ResetQueue();

      }
      
      mpNewRosAtlas = pRosAtlas;



      //mlpMapSubQueue.clear();

      //mlpAtlasPubQueue.clear();
      //mlpAtlasSubQueue.clear();

      //mlpAtlasSubQueue.push_back(pRosAtlas);
    }
    //else if(CheckSubGlobalMaps())
    //{
    //  unique_lock<mutex> lock(mMutexNewAtlas);
    //  std::cout << "Remove first atlas and replace it with the new one." << std::endl;
    //  mlpAtlasSubQueue.clear();
    //  mlpAtlasSubQueue.push_back(pRosAtlas);
    //} else {
    //  unique_lock<mutex> lock(mMutexNewAtlas);
    //  mlpAtlasSubQueue.push_back(pRosAtlas);
    //}


    {
      unique_lock<mutex> lock(mMutexNewMaps);
      unique_lock<mutex> lock2(mMutexNewAtlas);

      orbslam3_interfaces::msg::Map::SharedPtr pRosMap = std::make_shared<orbslam3_interfaces::msg::Map>(pRosAtlas->mp_current_map);
      for(orbslam3_interfaces::msg::KeyFrameUpdate& pRosKF : pRosMap->msp_keyframes)
      {
        //if(!mpKeyFrameQueue[pRosKF.mn_id]) //|| (mpObserver && mpKeyFrameQueue[pRosKF.mn_id] && mpKeyFrameQueue[pRosKF.mn_id]->mn_last_module < mpObserver->GetTaskModule()))
            orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(pRosKF);
            mpRosKF->from_module_id = pRosAtlas->from_module_id;
            mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKF);
            //mpKeyFrameQueue[pRosKF.mn_id] = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(pRosKF);
      }
      
      //if(mpLocalMapper->isStopped())
      mpLocalMapper->Release();
      mpLocalMapper->mbGBARunning = false;

      //for(const orbslam3_interfaces::msg::MapPoint& pRosMP : pRosMap->msp_map_points)
      //{
      //  if(!mpMapPointQueue[pRosMP.m_str_hex_id]) //|| (mpObserver && mpMapPointQueue[pRosMP.m_str_hex_id] && mpMapPointQueue[pRosMP.m_str_hex_id]->mn_last_module < mpObserver->GetTaskModule()))
      //      mpMapPointQueue[pRosMP.m_str_hex_id] = std::make_shared<orbslam3_interfaces::msg::MapPoint>(pRosMP);
      //}

      pRosMap->msp_keyframes.clear();
      pRosMap->msp_map_points.clear();

      //if(pRosAtlas->mb_last_batch)//&& mpObserver->GetTaskModule() != 1)

    }
}

void MapHandler::ResetQueue()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    unique_lock<mutex> lock2(mMutexNewAtlas);
    
    mlpAtlasPubQueue.clear();
    mlpMapPubQueue.clear();

    msErasedKFs.clear();
    msErasedMPs.clear();
    msUpdatedLocalMPs.clear();
    msUpdatedLocalKFs.clear();

    mpNewRosMap = std::shared_ptr<orbslam3_interfaces::msg::Map>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);
    mpNewRosAtlas = std::shared_ptr<orbslam3_interfaces::msg::Atlas>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);
}


void MapHandler::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool MapHandler::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void MapHandler::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    //unique_lock<mutex> lock2(mMutexStop);
    //mbStopped = true;
}

bool MapHandler::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}



void MapHandler::AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM)
{
    pSLAM = mSLAM;

    mpTracker = pSLAM->GetTrackerPtr();
    mpLocalMapper = pSLAM->GetMapperPtr();
    mpLoopCloser = pSLAM->GetLoopClosingPtr();
    mpAtlas = pSLAM->GetAtlas();
    mpKeyFrameDB = pSLAM->GetKeyFrameDatabase();
}


void MapHandler::AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node)
{
    pSLAMNode = slam_node;
}

void MapHandler::AttachObserver(std::shared_ptr<Observer> pObserver)
{
  mpObserver = pObserver; 
}
  
void MapHandler::AttachKFSubscriber(KeyFrameSubscriber* pKFSubscriber)
{
  mpKeyFrameSubscriber = pKFSubscriber; 
}
//}
