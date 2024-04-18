
#include "./MapHandler.hpp"
#include "./Observer.hpp"
#include "../slam/slam-wrapper-node.hpp"
//#include "../slam/slam-wrapper-node.hpp"
//#include "orbslam3_interfaces/Converter.hpp"
//#include "orbslam3_interfaces/KeyFrameConverter.hpp"
//#include "orbslam3_interfaces/Converter.hpp"

//namespace TempDistributor {

MapHandler::MapHandler()
{

}

MapHandler::~MapHandler()
{

}

// main function
void MapHandler::Run()
{

    mbFinished = false;
    
    while(1)
    {
      // Local map publish/subscription 
      if(CheckPubLocalMaps())
      {
        ProcessNewPubLocalMap();
      }

      if(CheckSubLocalMaps())
      {
        ProcessNewSubLocalMap();
      }

      // Global map (Atlas - map merging, full map update, loop closing) publish/subscription 
      if(CheckPubGlobalMaps())
      {
        ProcessNewPubGlobalMap();
      }

      if(CheckSubGlobalMaps())
      {
        ProcessNewSubGlobalMap();
      }

      usleep(1000);
      if(CheckFinish())
          break;
    }

    SetFinish();
}

void MapHandler::ProcessNewPubGlobalMap()
{
    //unique_lock<mutex> lock1(mMutexMapUpdate);
    std::cout << "Processing new Global Map" << std::endl;
    std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate;
    {
        unique_lock<mutex> lock2(mMutexNewAtlas);
        mtAtlasUpdate = mlpAtlasPubQueue.front();
        mlpAtlasPubQueue.pop_front();
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
    //mRosAtlas.from_module_id = mnTaskModule;
    mRosAtlas.mb_map_merge = std::get<0>(mtAtlasUpdate);
    mRosAtlas.mb_loop_closer = std::get<1>(mtAtlasUpdate);
    mRosAtlas.mv_merged_map_ids = std::get<2>(mtAtlasUpdate);
    
    pSLAMNode->publishAtlas(mRosAtlas);
    //mbMapUpdateRunning = false;
    //return mRosAtlas;
}

void MapHandler::ProcessNewSubGlobalMap()
{
    orbslam3_interfaces::msg::Atlas::SharedPtr mpRosAtlas = static_cast<orbslam3_interfaces::msg::Atlas::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewAtlas);
        mpRosAtlas = mlpAtlasSubQueue.front();

        mlpAtlasSubQueue.pop_front();
    }

    orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = std::make_shared<orbslam3_interfaces::msg::Map>(mpRosAtlas->mp_current_map);
    
    // Create map data structures for postloads
    std::map<std::string, ORB_SLAM3::MapPoint*> mFusedMPs; 
    std::map<std::string, ORB_SLAM3::MapPoint*> mMapMPs;

    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mFusedKFs; 
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mMapKFs; 
    
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    // Get maps and the map where KF is
    ORB_SLAM3::Map* pMergeMap = static_cast<ORB_SLAM3::Map*>(NULL);
    ORB_SLAM3::Map* pCurrentMap = static_cast<ORB_SLAM3::Map*>(NULL);
    std::map<unsigned long int, ORB_SLAM3::Map*> mMaps;
    for(ORB_SLAM3::Map* pM : mpAtlas->GetAllMaps())
    {
        if(mpRosAtlas->mb_map_merge && mpRosAtlas->mv_merged_map_ids.size() >= 2)
        {

            if(pM->GetId() == mpRosAtlas->mv_merged_map_ids[1])
            {
                pCurrentMap = pM;
            }
            
            if(pM->GetId() == mpRosAtlas->mv_merged_map_ids[0])
            {
                pMergeMap = pM;
            }
        } else {
            if(pM->GetId() == mpRosMap->mn_id)
            {
                pCurrentMap = pM;
            }
        }

        mMaps[pM->GetId()] = pM;
    }
    
    std::cout << "Got Atlas Update. 1. Maps collected." << std::endl;

    //if(!pCurrentMap)
    //{
    //    mpAtlas->CreateNewMap();
    //    pCurrentMap = mpAtlas->GetCurrentMap(); 
    //    mMaps[pCurrentMap->GetId()] = pCurrentMap;
    //}

    
    if(mpRosAtlas->mb_map_merge)
    {

        
        // Collect all KFs from current and merged maps
        for(ORB_SLAM3::KeyFrame* pKF : pCurrentMap->GetAllKeyFrames())
        {
          mFusedKFs[pKF->mnId] = pKF;
          mMapKFs[pKF->mnId] = pKF;
        }
        
        for(ORB_SLAM3::KeyFrame* pKF : pMergeMap->GetAllKeyFrames())
        {
          mFusedKFs[pKF->mnId] = pKF;
          mMapKFs[pKF->mnId] = pKF;
        }

        // Collect all MPs from current and merged maps
        for(ORB_SLAM3::MapPoint* pMP : pCurrentMap->GetAllMapPoints())
        {
          mFusedMPs[pMP->mstrHexId] = pMP;
          mMapMPs[pMP->mstrHexId] = pMP;
        }

        for(ORB_SLAM3::MapPoint* pMP : pMergeMap->GetAllMapPoints())
        {
          mFusedMPs[pMP->mstrHexId] = pMP;
          mMapMPs[pMP->mstrHexId] = pMP;
        }

        // Collect all cameras
        for(ORB_SLAM3::GeometricCamera* pCam : mpAtlas->GetAllCameras())
        {
          mCameras[pCam->GetId()] = pCam;
        }

    } else {

        for(ORB_SLAM3::KeyFrame* pKF : pCurrentMap->GetAllKeyFrames())
        {
          mFusedKFs[pKF->mnId] = pKF;
          mMapKFs[pKF->mnId] = pKF;
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
    }

    std::cout << "Got Atlas Update. 2. ORB_SLAM3 data collected." << std::endl;

    
    std::vector<ORB_SLAM3::KeyFrame*> mvpTempKFs;
    mvpTempKFs.resize(mpRosMap->msp_keyframes.size());
    std::map<std::string, ORB_SLAM3::MapPoint*> mTempMPs;
    
    // Sort the keyframes from smallest id to largest
    // So that postloads should be done in order
    std::vector<orbslam3_interfaces::msg::KeyFrame> mvRosKF = mpRosMap->msp_keyframes;
    std::sort(mvRosKF.begin(), mvRosKF.end(), [](const orbslam3_interfaces::msg::KeyFrame& a, const orbslam3_interfaces::msg::KeyFrame& b) {
        return a.mn_id < b.mn_id;
    });

    // Loop through each KF
    // Convert to ORB
    // Add to mFusedKFs if not found -> if its not there then its a new object and can be used as it is for other objects
    // if its there, then use original pointer in postload and just update that object afterwards
    for(size_t i = 0; i<mvRosKF.size(); ++i)
    {
        orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrame>(mvRosKF[i]);
        ORB_SLAM3::KeyFrame* tempKF = mpObserver->ConvertKeyFrame(mpRosKF, mMaps);
        mvpTempKFs[i] = tempKF;

        if(mFusedKFs.find(tempKF->mnId) == mFusedKFs.end())
            mFusedKFs[tempKF->mnId] = tempKF;
        
        // Do same stuff for each MP
        // Skip MPs which were already found in this map update (same objects in original map)
        for(size_t j = 0; j<mpRosKF->mvp_map_points.size(); ++j)
        {
            orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[j]);
            // This MP was already seen in the map, skip since both points to the same pointer in the original
            if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
                continue;
            
            ORB_SLAM3::MapPoint* tempMP = mpObserver->ConvertMapPoint(mpRosMP, mMaps);
            mTempMPs[tempMP->mstrHexId] = tempMP;

            if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
                mFusedMPs[tempMP->mstrHexId] = tempMP;
        }
    }


    // There might be MPs which were not associated with KFs
    // Process those as well
    for(size_t i = 0; i<mpRosMap->msp_map_points.size(); ++i)
    {
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosMap->msp_map_points[i]);

        if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
            continue;

        ORB_SLAM3::MapPoint* tempMP = mpObserver->ConvertMapPoint(mpRosMP, mMaps);
        mTempMPs[tempMP->mstrHexId] = tempMP;

        if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
            mFusedMPs[tempMP->mstrHexId] = tempMP;
    }

    std::cout << "Got Atlas Update. 3. New data fused with the existing." << std::endl;
    
    // PostLoad for all the KFs
    bool bKFUnprocessed = false;
    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
      tempKF->PostLoad(mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    }

    // PostLoad for all the MPs
    for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    {
      ORB_SLAM3::MapPoint* tempMP = it->second;
      tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    }

    std::cout << "Got Atlas Update. 4. PostLoads (KF + MPs) completed." << std::endl;
    
    //mpLocalMapper_->RequestStop();
    //// Wait till stopped
    //while(!mpLocalMapper_->isStopped())
    //{
    //    usleep(1000);
    //}

    // Inject KFs
    for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    {
      ORB_SLAM3::MapPoint* tempMP = it->second;
      mpObserver->InjectMapPoint(tempMP, mMapMPs);
    }
    
    std::cout << "Got Atlas Update. 5. MPs injected into ORB_SLAM3" << std::endl;
    
    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
      ORB_SLAM3::KeyFrame* pKF = mpObserver->InjectKeyFrame(tempKF, mMapKFs);
      if(pKF)
      {
          mpObserver->ForwardKeyFrameToTarget(pKF, 0, mpRosAtlas->from_module_id);
      }
    }
    
    std::cout << "Got Atlas Update. 6. KFs injected into ORB_SLAM3" << std::endl;
    
    //mpLocalMapper_->Release();
    // Remove KFs and MPs
    //for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
    //{
    //    unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
    //    ORB_SLAM3::KeyFrame* pEraseKF=mFusedKFs[mnId];
    //    if(pEraseKF)
    //        pEraseKF->SetBadFlag();
    //}

    //for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
    //{
    //  std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
    //    ORB_SLAM3::MapPoint* pEraseMP=mFusedMPs[mnId];
    //    if(pEraseMP)
    //        pEraseMP->SetBadFlag();
    //}
    
    std::cout << "Got Atlas Update. 7. KFs and MPs removed" << std::endl;

    // Finally update Map object 
    // First create temporary Map from ROS data
    // Then Inject the data into real map (do not update KFs and MPs since those are already added)
    ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);
    tempMap->PostLoad(mpKeyFrameDB, pSLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    mpObserver->InjectMap(tempMap, pCurrentMap);
    
    std::cout << "Got Atlas Update. 8. Map Injection completed" << std::endl;

    if(mpRosAtlas->mb_map_merge && pMergeMap)
    {
        mpAtlas->SetMapBad(pMergeMap);
        mpAtlas->RemoveBadMaps();
    }

    std::cout << "Got Atlas Update. 9. Bad maps are removed and GrabAtlas COMPLETE." << std::endl;

}

void MapHandler::ProcessNewPubLocalMap()
{
    //unique_lock<mutex> lock1(mMutexMapUpdate);
    std::cout << "Processing new Local Map" << std::endl;
    ORB_SLAM3::Map* pMap = static_cast<ORB_SLAM3::Map*>(NULL);
    {
        unique_lock<mutex> lock2(mMutexNewMaps);
        pMap = mlpMapPubQueue.front();
        mlpMapPubQueue.pop_front();
    }

    std::vector<ORB_SLAM3::GeometricCamera*> mvpCameras = mpAtlas->GetAllCameras();
    std::set<ORB_SLAM3::GeometricCamera*> mspCameras(mvpCameras.begin(), mvpCameras.end());
    
    pMap->PreSave(mspCameras);
    

    orbslam3_interfaces::msg::Map mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap); 
    //mRosMap.from_module_id = mnTaskModule; 
    
    pMap->ClearErasedData();
    pMap->ClearUpdatedKFIds();
    pMap->ClearUpdatedMPIds();

    pSLAMNode->publishMap(mRosMap);
    //usleep(10000);
    //mbMapUpdateRunning = false;
    //return mRosMap;
}

void MapHandler::ProcessNewSubLocalMap()
{
    orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = static_cast<orbslam3_interfaces::msg::Map::SharedPtr>(NULL);
    {
        unique_lock<mutex> lock(mMutexNewMaps);
        mpRosMap = mlpMapSubQueue.front();

        mlpMapSubQueue.pop_front();
    }

    // Get maps and the map where KF is
    ORB_SLAM3::Map* pCurrentMap = static_cast<ORB_SLAM3::Map*>(NULL);
    std::map<unsigned long int, ORB_SLAM3::Map*> mMaps;
    for(ORB_SLAM3::Map* pM : mpAtlas->GetAllMaps())
    {
        if(pM->GetId() == mpRosMap->mn_id)
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
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mMapKFs; 
    
    std::map<unsigned int, ORB_SLAM3::GeometricCamera*> mCameras; 
    
    for(ORB_SLAM3::KeyFrame* pKF : pCurrentMap->GetAllKeyFrames())
    {
      mFusedKFs[pKF->mnId] = pKF;
      mMapKFs[pKF->mnId] = pKF;
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
    
    std::cout << "Got an update for a map. 1. Existing data collected." << std::endl;
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpTempKFs;
    mvpTempKFs.resize(mpRosMap->msp_keyframes.size());
    std::map<std::string, ORB_SLAM3::MapPoint*> mTempMPs;
    
    // Sort the keyframes from smallest id to largest
    // So that postloads should be done in order
    std::vector<orbslam3_interfaces::msg::KeyFrame> mvRosKF = mpRosMap->msp_keyframes;
    std::sort(mvRosKF.begin(), mvRosKF.end(), [](const orbslam3_interfaces::msg::KeyFrame& a, const orbslam3_interfaces::msg::KeyFrame& b) {
        return a.mn_id < b.mn_id;
    });

    // Loop through each KF
    // Convert to ORB
    // Add to mFusedKFs if not found -> if its not there then its a new object and can be used as it is for other objects
    // if its there, then use original pointer in postload and just update that object afterwards
    for(size_t i = 0; i<mvRosKF.size(); ++i)
    {
        orbslam3_interfaces::msg::KeyFrame::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrame>(mvRosKF[i]);
        ORB_SLAM3::KeyFrame* tempKF = mpObserver->ConvertKeyFrame(mpRosKF, mMaps);
        mvpTempKFs[i] = tempKF;

        if(mFusedKFs.find(tempKF->mnId) == mFusedKFs.end())
            mFusedKFs[tempKF->mnId] = tempKF;
        
        // Do same stuff for each MP
        // Skip MPs which were already found in this map update (same objects in original map)
        for(size_t j = 0; j<mpRosKF->mvp_map_points.size(); ++j)
        {
            orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosKF->mvp_map_points[j]);
            // This MP was already seen in the map, skip since both points to the same pointer in the original
            if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
                continue;
            
            ORB_SLAM3::MapPoint* tempMP = mpObserver->ConvertMapPoint(mpRosMP, mMaps);
            mTempMPs[tempMP->mstrHexId] = tempMP;

            if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
                mFusedMPs[tempMP->mstrHexId] = tempMP;
        }
    }
    
    std::cout << "Got an update for a map. 2. Data fusion complete." << std::endl;

    // There might be MPs which were not associated with KFs
    // Process those as well
    for(size_t i = 0; i<mpRosMap->msp_map_points.size(); ++i)
    {
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosMap->msp_map_points[i]);

        if(mTempMPs.find(mpRosMP->m_str_hex_id) != mTempMPs.end())
            continue;

        ORB_SLAM3::MapPoint* tempMP = mpObserver->ConvertMapPoint(mpRosMP, mMaps);
        mTempMPs[tempMP->mstrHexId] = tempMP;

        if(mFusedMPs.find(tempMP->mstrHexId) == mFusedMPs.end())
            mFusedMPs[tempMP->mstrHexId] = tempMP;
    }

    std::cout << "Got an update for a map. 3. MPs processed." << std::endl;
    
    // PostLoad for all the KFs
    bool bKFUnprocessed = false;
    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
      tempKF->PostLoad(mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    }

    // PostLoad for all the MPs
    for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    {
      ORB_SLAM3::MapPoint* tempMP = it->second;
      tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    }
    
    std::cout << "Got an update for a map. 4. PostLoads completed." << std::endl;

    // Inject KFs
    for(std::map<std::string, ORB_SLAM3::MapPoint*>::iterator it = mTempMPs.begin(); it != mTempMPs.end(); ++it)
    {
      ORB_SLAM3::MapPoint* tempMP = it->second;
      mpObserver->InjectMapPoint(tempMP, mMapMPs);
    }

    for(size_t i = 0; i<mvpTempKFs.size(); ++i)
    {
      ORB_SLAM3::KeyFrame* tempKF = mvpTempKFs[i];
      ORB_SLAM3::KeyFrame* pKF = mpObserver->InjectKeyFrame(tempKF, mMapKFs);

      if(pKF)
          mpObserver->ForwardKeyFrameToTarget(pKF, 0, mpRosMap->from_module_id);
    }
    
    std::cout << "Got an update for a map. 5. All the data is injected into ORB_SLAM3." << std::endl;

    // Remove KFs and MPs
    //for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
    //{
    //    unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
    //    ORB_SLAM3::KeyFrame* pEraseKF=mFusedKFs[mnId];
    //    if(pEraseKF)
    //        pEraseKF->SetBadFlag();
    //}

    //for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
    //{
    //  std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
    //    ORB_SLAM3::MapPoint* pEraseMP=mFusedMPs[mnId];
    //    if(pEraseMP)
    //        pEraseMP->SetBadFlag();
    //}

    std::cout << "Got an update for a map. 6. Data which was deleted elsewhere is removed from the system." << std::endl;

    // Finally update Map object 
    // First create temporary Map from ROS data
    // Then Inject the data into real map (do not update KFs and MPs since those are already added)
    ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);
    tempMap->PostLoad(mpKeyFrameDB, pSLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    mpObserver->InjectMap(tempMap, pCurrentMap);

    std::cout << "Got an update for a map. 7. Map is injected into ORB_SLAM3 and Map update is COMPLETE." << std::endl;

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
    return(!mlpMapPubQueue.empty());
}


bool MapHandler::CheckSubLocalMaps()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    return(!mlpMapSubQueue.empty());
}


bool MapHandler::CheckPubGlobalMaps()
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    return(!mlpAtlasPubQueue.empty());
}

bool MapHandler::CheckSubGlobalMaps()
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    return(!mlpAtlasSubQueue.empty());
}


void MapHandler::InsertNewPubLocalMap(ORB_SLAM3::Map* pMap)
{
    unique_lock<mutex> lock(mMutexNewMaps);
    mlpMapPubQueue.push_back(pMap);
}

void MapHandler::InsertNewSubLocalMap(orbslam3_interfaces::msg::Map::SharedPtr pRosMap)
{
    unique_lock<mutex> lock(mMutexNewMaps);
    mlpMapSubQueue.push_back(pRosMap);
}

void MapHandler::InsertNewPubGlobalMap(std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate)
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    mlpAtlasPubQueue.push_back(mtAtlasUpdate);
}

void MapHandler::InsertNewSubGlobalMap(orbslam3_interfaces::msg::Atlas::SharedPtr pRosAtlas)
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    mlpAtlasSubQueue.push_back(pRosAtlas);
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
  
//}
