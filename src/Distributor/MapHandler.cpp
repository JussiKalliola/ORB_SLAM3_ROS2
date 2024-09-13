
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
  : mbFinished(false), mbFinishRequested(false), mnMapFreq_ms(0), mnGlobalMapFreq_ms(0), mnPubIters(0), mnAtlasBatchNumber(0), 
    maxUpdateN(5), maxUpdateGlobalN(20), mnLastGlobalAction(-1)
{
  mpNewRosMap = std::shared_ptr<orbslam3_interfaces::msg::Map>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);
  mpNewRosAtlas = std::shared_ptr<orbslam3_interfaces::msg::Atlas>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);

  msUpdatedLocalKFs = std::set<unsigned long int>();
  msUpdatedLocalMPs = std::set<std::string>();

  msErasedKFs = std::set<unsigned long int>();
  msUpdatedGlobalKFs = std::set<unsigned long int>();
  msUpdatedGlobalMPs = std::set<std::string>();

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
        //if(mpObserver->GetTaskModule() == 2 && CheckNewUpdates())
        //{
        //    InsertNewPubLocalMap();
        //}
        
        // Local map publish/subscription 
        if(CheckPubLocalMaps() && !mpLoopCloser->CheckIfRunning() && !mpLoopCloser->isRunningGBA()  && !mpLocalMapper->IsLMRunning() && mpObserver->GetTaskModule()==2)
            ProcessNewPubLocalMap();
        else if(CheckSubLocalMaps() && !mpLoopCloser->CheckIfRunning() && !mpLoopCloser->isRunningGBA()  && !mpLocalMapper->IsLMRunning())
            ProcessNewSubLocalMap2();
        
        if(!mpLocalMapper->mbGBARunning && !mpLocalMapper->IsLMRunning() && mpKeyFrameSubscriber->KeyFramesInQueue() == 0 && mpLocalMapper->KeyframesInQueue() == 0 && !msToBeErasedKFs.empty() && mpObserver->GetTaskModule() == 2)
        {
            std::cout << "LocalMapper is empty and there is KFs to be optimized=" << msToBeErasedKFs.size() << std::endl;
            unsigned long int mnId = *msToBeErasedKFs.begin();
            msToBeErasedKFs.erase(mnId);
            ORB_SLAM3::KeyFrame* pUpdateKF = mpObserver->GetKeyFrame(mnId);
            if(pUpdateKF)
            {
                pUpdateKF->SetLastModule(3);
                //mpKeyFrameDB->erase(pUpdateKF);
                //mpAtlas->GetCurrentMap()->EraseKeyFrame(pUpdateKF);
                //mpKeyFrameDB->erase(pUpdateKF);
                pUpdateKF->mbLCDone = true;
                pUpdateKF->mnNextTarget=3;
                mpLocalMapper->InsertKeyframeFromRos(pUpdateKF);
                //break;
            } 
            //else
            //{
            //    msErasedKFs.insert(mnId);
            //}
        }

        // Global map (Atlas - map merging, full map update, loop closing) publish/subscription 
        if(CheckPubGlobalMaps())
            ProcessNewPubGlobalMap();
        else if(CheckSubGlobalMaps())
            ProcessNewSubGlobalMap2();
       



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

    std::map<unsigned long int, ORB_SLAM3::KeyFrame*>& mFusedKFs = mpObserver->GetAllKeyFrames();
    std::map<unsigned long int, ORB_SLAM3::Map*> mMaps = mpObserver->GetAllMaps();

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartProcAtlas = std::chrono::steady_clock::now();
    std::chrono::system_clock::time_point time_Start = std::chrono::system_clock::now();
    vtTimesPubAtlas.push_back(time_Start);

    
    ORB_SLAM3::Map* mpCurrentMap = mpAtlas->GetCurrentMap();//mMaps[std::get<2>(mtAtlasUpdate)[0]];
    //std::cout << " ------------- ProcessNewPubGlobalMap::mpCurrentMap=" << std::get<2>(mtAtlasUpdate)[0] << std::endl; 
    //if(!mpCurrentMap)
    //    return;
    
    // Clear all updated data since we want to publish all KFs
    //if(std::get<0>(mtAtlasUpdate) || std::get<1>(mtAtlasUpdate))
    //{
    //    mpCurrentMap->ClearUpdatedKFIds();
    //    mpCurrentMap->ClearUpdatedMPIds();
    //}
    
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
    
    mRosAtlas->system_id = std::getenv("SLAM_SYSTEM_ID");
    mRosAtlas->from_module_id = mpObserver->GetTaskModule();
    mRosAtlas->mb_map_merge = std::get<0>(mtAtlasUpdate);
    mRosAtlas->mb_loop_closer = std::get<1>(mtAtlasUpdate);
    mRosAtlas->mv_merged_map_ids = std::get<2>(mtAtlasUpdate);
    mRosAtlas->mn_batch_number = mnAtlasBatchNumber;

    {
        unique_lock<mutex> lock(mMutexUpdates);
        std::cout << "msUpdatedGlobalKFs.size()=" << msUpdatedGlobalKFs.size() << ", msUpdatedGlobalMPs.size()=" << msUpdatedGlobalMPs.size() << ", msErasedMPs.size()=" << msErasedMPs.size() << ", msErasedKFs.size()=" << msErasedKFs.size() << ", maxUpdateGlobalN=" << maxUpdateGlobalN << ", mnAtlasBatchNumber=" << mnAtlasBatchNumber<< ", mnGlobalMapFreq_ms=" << mnGlobalMapFreq_ms << std::endl;

        if(msUpdatedGlobalKFs.size() > maxUpdateGlobalN)
        {
            std::set<unsigned long int> s;
            
            // For the first 5 maps, traverse spanning tree and publish the covisible KFs
            //if(mnAtlasBatchNumber<2)
            //{
            //    ORB_SLAM3::KeyFrame* mpLastKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL);
            //    while(!mpLastKF)
            //    {
            //        if(mnAtlasBatchNumber==0)
            //            mpLastKF=mpLoopCloser->GetCurrentKeyFrame();
            //        else
            //        {   

            //            const std::vector<ORB_SLAM3::KeyFrame*>& mvKfs = mpLastGlobalKF->GetBestCovisibilityKeyFrames(10);
            //            for(const auto& mpKF : mvKfs)
            //            {
            //                if(mpKF && msUpdatedGlobalKFs.find(mpKF->mnId) != msUpdatedGlobalKFs.end())
            //                {
            //                  mpLastKF=mpKF;
            //                  break;
            //                }

            //            }
            //        }

            //    }

            //    const std::vector<ORB_SLAM3::KeyFrame*>& vpConnected = mpLastKF->GetBestCovisibilityKeyFrames(maxUpdateGlobalN);
            //    int iters = maxUpdateGlobalN;
            //    if(vpConnected.size() < maxUpdateGlobalN)
            //        iters = vpConnected.size();
            //        
            //    for(int i=0;i<iters;i++)
            //    {
            //        ORB_SLAM3::KeyFrame* pConnKF = vpConnected[i];
            //        if(!pConnKF)
            //            continue;
            //        if(msUpdatedGlobalKFs.find(pConnKF->mnId) != msUpdatedGlobalKFs.end())
            //        {
            //            s.insert(pConnKF->mnId);
            //            msUpdatedGlobalKFs.erase(pConnKF->mnId);
            //        }

            //        mpLastGlobalKF = pConnKF;
            //    }

            //}
            //else 
            //{
                ORB_SLAM3::KeyFrame* mpLastKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL); 
                while(!mpLastKF)
                {
                    if(msUpdatedGlobalKFs.empty())
                        break;

                    unsigned long int mnId;
                    mnId = *msUpdatedGlobalKFs.rbegin();

                    mpLastKF=mpObserver->GetKeyFrame(mnId);
                    if(mpLastKF)
                    {
                        s.insert(mnId);
                        msUpdatedGlobalKFs.erase(mnId);
                        break;
                    } else
                        msUpdatedGlobalKFs.erase(mnId);
                }

                while(s.size()<maxUpdateGlobalN)
                {
                    if(msUpdatedGlobalKFs.empty())
                        break;
                    if(!mpLastKF)
                        return;

                    const std::vector<ORB_SLAM3::KeyFrame*>& mvpCovisibleKFs = mpLastKF->GetVectorCovisibleKeyFrames();
                    for(const auto& tempKF : mvpCovisibleKFs)
                    {
                        if(msUpdatedGlobalKFs.find(tempKF->mnId) != msUpdatedGlobalKFs.end())
                        {
                            msUpdatedGlobalKFs.erase(tempKF->mnId);
                            s.insert(tempKF->mnId);
                            if(s.size()>=maxUpdateGlobalN)
                                break;
                        }
                    }

                    for(std::set<unsigned long int>::reverse_iterator it = msUpdatedGlobalKFs.rbegin(); it != msUpdatedGlobalKFs.rend(); ++it)
                    {
                        mpLastKF=mpObserver->GetKeyFrame(*it);

                        if(mpLastKF)
                        {
                            s.insert(*it);
                            msUpdatedGlobalKFs.erase(*it);
                            break;
                        } else
                            msUpdatedGlobalKFs.erase(*it);
                      }


                }

                //for(int i=0;i<maxUpdateGlobalN;i++)
                //{
                //    s.insert(*msUpdatedGlobalKFs.rbegin());
                //    msUpdatedGlobalKFs.erase(*msUpdatedGlobalKFs.rbegin());
                //    //msErasedKFs.erase(*msUpdatedLocalKFs.rbegin());
                //}

            //}


            std::set<unsigned long int>& msAllErasedKFs = mpObserver->msAllErasedKFIds;
            std::set<std::string>& msAllErasedMPs = mpObserver->msAllErasedMPIds;
            std::cout << "altas publish. msAllErasedMPs.size()=" << msAllErasedMPs.size() << ", msAllErasedKFs.size()=" << msAllErasedKFs.size() << std::endl;
            mRosMap = Converter::MapConverter::OrbMapToRosMap(mpCurrentMap, s, msUpdatedGlobalMPs, msAllErasedKFs, msAllErasedMPs, mpObserver->GetAllKeyFrames(), mpObserver->GetAllMapPoints()); 


            
            if(mnAtlasBatchNumber > 0)
            {
                mRosMap->mvp_erased_mappoint_ids.clear();
                mRosMap->mvp_backup_map_points_ids.clear();
                mRosMap->mvp_backup_keyframes_ids.clear();
                mRosMap->mv_backup_keyframe_origins_id.clear();
            }
            // Decrease the rate of publishing so that the network does not congest
            mnGlobalMapFreq_ms=100;
            mnAtlasBatchNumber+=1;

        }
        else {
            std::cout << "altas publish. msAllErasedMPs.size()=" << msErasedMPs.size() << ", msAllErasedKFs.size()=" << msErasedKFs.size() << std::endl;
            mRosMap = Converter::MapConverter::OrbMapToRosMap(mpCurrentMap, msUpdatedGlobalKFs, msUpdatedGlobalMPs, msErasedKFs, msErasedMPs, mpObserver->GetAllKeyFrames(), mpObserver->GetAllMapPoints()); 

            msUpdatedGlobalKFs.clear();
            msUpdatedGlobalMPs.clear();
            msErasedKFs.clear();
            msErasedMPs.clear();
            
            // Make next update instant
            mnGlobalMapFreq_ms=0;
            maxUpdateGlobalN=20;
            mRosAtlas->mb_last_batch = true;
            mnAtlasBatchNumber=0;
            mlpAtlasPubQueue.pop_front();
        }

        if(mRosMap->msp_keyframes.size()==0 && !mlpAtlasPubQueue.empty())
        {
            msUpdatedGlobalKFs.clear();
            msUpdatedGlobalMPs.clear();
            msErasedKFs.clear();
            msErasedMPs.clear();
            
            // Make next update instant
            mnGlobalMapFreq_ms=0;
            maxUpdateGlobalN=20;
            mRosAtlas->mb_last_batch = true;
            mnAtlasBatchNumber=0;
            mlpAtlasPubQueue.pop_front();

        }
    }
    //mRosMap.from_module_id = mnTaskModule; 

    mRosAtlas->mp_current_map = *mRosMap;//Converter::MapConverter::OrbMapToRosMap(mpCurrentMap);
    

    // End of timer
    //std::chrono::steady_clock::time_point time_EndConvMap = std::chrono::steady_clock::now();
    //double timeConvMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvMap - time_StartConvMap).count();
    //vdOrb2RosConvMap_ms.push_back(timeConvMap);





    

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
    std::chrono::system_clock::time_point time_Start = std::chrono::system_clock::now();
    vtTimesPubMap.push_back(time_Start);

    const std::vector<ORB_SLAM3::GeometricCamera*>& mvpCameras = mpAtlas->GetAllCameras();
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
        std::cout << "msUpdatedLocalKFs.size()=" << msUpdatedLocalKFs.size() << ", msUpdatedLocalMPs.size()=" << msUpdatedLocalMPs.size() << ", msErasedMPs.size()=" << msErasedMPs.size() << ", msErasedKFs.size()=" << msErasedKFs.size() << ", maxUpdateN=" << maxUpdateN << ", mnPubIters=" << mnPubIters << ", mnMapFreq_ms=" << mnMapFreq_ms << std::endl;
        if(msUpdatedLocalKFs.size() > maxUpdateN)
        {
            std::set<unsigned long int> s;
            // First 20 kfs taken from covisible keyframes (KFs which needs the fastest updates)
            if(mnPubIters==0) //&& mpObserver->mpLastKeyFrame)
            {
                mnMapFreq_ms=30;
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
                while(s.size()<maxUpdateN) 
                {
                    if(msUpdatedLocalKFs.empty())
                        break;
                    ORB_SLAM3::KeyFrame* tempKF=mpObserver->GetKeyFrame(*msUpdatedLocalKFs.rbegin());
                    if(!tempKF) //|| (tempKF && tempKF->GetLastModule()==3))
                        continue;


                    s.insert(*msUpdatedLocalKFs.rbegin());
                    msUpdatedLocalKFs.erase(*msUpdatedLocalKFs.rbegin());
                    //msErasedKFs.erase(*msUpdatedLocalKFs.rbegin());
                }
                //std::set<std::string> msMPs = mpLocalMapper->msNewMapPointIds;
                //for(const std::string& mpId : msMPs)
                //{
                //    msUpdatedLocalMPs.erase(mpId);
                //}
                mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap, s, msUpdatedLocalMPs, msErasedKFs, msErasedMPs, mpObserver->GetAllKeyFrames(), mpObserver->GetAllMapPoints()); 
                //mpLocalMapper->msNewMapPointIds.clear();
            }
            // Others can be updated in order
            else { 
                ORB_SLAM3::KeyFrame* mpLastKF = static_cast<ORB_SLAM3::KeyFrame*>(NULL); 
                while(!mpLastKF)
                {
                    if(msUpdatedLocalKFs.empty())
                        break;
                    mpLastKF=mpObserver->GetKeyFrame(*msUpdatedLocalKFs.rbegin());
                    if(mpLastKF)
                    {
                        s.insert(*msUpdatedLocalKFs.rbegin());
                        msUpdatedLocalKFs.erase(*msUpdatedLocalKFs.rbegin());
                        break;
                    } else
                        msUpdatedLocalKFs.erase(*msUpdatedLocalKFs.rbegin());
                }

                while(s.size()<maxUpdateN)
                {
                    if(msUpdatedLocalKFs.empty())
                        break;
                    const std::vector<ORB_SLAM3::KeyFrame*>& mvpCovisibleKFs = mpLastKF->GetVectorCovisibleKeyFrames();
                    for(const auto& tempKF : mvpCovisibleKFs)
                    {
                        if(msUpdatedLocalKFs.find(tempKF->mnId) != msUpdatedLocalKFs.end())
                        {
                            msUpdatedLocalKFs.erase(tempKF->mnId);
                            s.insert(tempKF->mnId);
                            if(s.size()>=maxUpdateN)
                                break;
                        }
                    }

                    for(std::set<unsigned long int>::reverse_iterator it = msUpdatedLocalKFs.rbegin(); it != msUpdatedLocalKFs.rend(); ++it)
                    {
                        mpLastKF=mpObserver->GetKeyFrame(*it);
                        if(mpLastKF)
                        {
                            s.insert(*msUpdatedLocalKFs.rbegin());
                            msUpdatedLocalKFs.erase(*it);
                            break;
                        } else
                            msUpdatedLocalKFs.erase(*it);

                    }

                }
                
                mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap, s, msUpdatedLocalMPs, msErasedKFs, msErasedMPs, mpObserver->GetAllKeyFrames(), mpObserver->GetAllMapPoints()); 
            }

            if(mnPubIters==0)
                mRosMap->mb_first_batch = true;
            // Decrease the rate of publishing so that the network does not congest
            //mnMapFreq_ms=50;
            //maxUpdateN=5;
            mnPubIters++;
            //msErasedMPs.clear();
            //msErasedKFs.clear();
        }
        else if(msUpdatedLocalKFs.size() <= maxUpdateN && !msUpdatedLocalKFs.empty()) {
            mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap, msUpdatedLocalKFs, msUpdatedLocalMPs, msErasedKFs, msErasedMPs, mpObserver->GetAllKeyFrames(), mpObserver->GetAllMapPoints()); 

            if(mnPubIters == 0)
                mRosMap->mb_first_batch = true;

            msUpdatedLocalKFs.clear();
            //msUpdatedLocalMPs.clear();
            //msErasedKFs.clear();
            //msErasedMPs.clear();
            
            // Make next update instant
            //if(mpLocalMapper->mbGBARunning)
            //    maxUpdateN=15;
            //else
        } else if(!msUpdatedLocalMPs.empty() && msUpdatedLocalKFs.empty()) 
        {
            mRosMap = Converter::MapConverter::OrbMapToRosMap(pMap, msUpdatedLocalKFs, msUpdatedLocalMPs, msErasedKFs, msErasedMPs, mpObserver->GetAllKeyFrames(), mpObserver->GetAllMapPoints()); 
            
            if(mnPubIters == 0)
                mRosMap->mb_first_batch = true;
        

        }

    }
    //mRosMap.from_module_id = mnTaskModule; 

    
    // End of timer
    std::chrono::steady_clock::time_point time_EndConvMap = std::chrono::steady_clock::now();
    double timeConvMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndConvMap - time_StartConvMap).count();
    vdOrb2RosConvMap_ms.push_back(timeConvMap);

    mRosMap->from_module_id = mpObserver->GetTaskModule();

    if(maxUpdateN<10)
    {
        maxUpdateN+=5;
        mnMapFreq_ms+=100;
    }

    
    if(mpAtlas->GetBadMaps().size()>0)
    {
        std::set<ORB_SLAM3::Map*> msBadMaps=mpAtlas->GetBadMaps();
        std::vector<unsigned long int> mvpBadMapIds;
        for(std::set<ORB_SLAM3::Map*>::iterator it = msBadMaps.begin(); it != msBadMaps.end(); ++it)
        {
          ORB_SLAM3::Map* pBadMap= *it;
          if(pBadMap)
          {
              mvpBadMapIds.push_back(pBadMap->GetId());
              std::cout << "----------------------------- BAD MAPS=" << pBadMap->GetId() << " ---------------- " << std::endl;

          }
        }
        mRosMap->ms_bad_maps=mvpBadMapIds;
        mpAtlas->RemoveBadMaps();

    }


    
    //pMap->ClearErasedData();
    //pMap->ClearUpdatedKFIds();
    //pMap->ClearUpdatedMPIds();

    pSLAMNode->publishMap(mRosMap);

    {
        unique_lock<mutex> lock2(mMutexUpdates);
        msErasedMPs.clear();
        msErasedKFs.clear();
    }


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
    std::chrono::system_clock::time_point time_Start = std::chrono::system_clock::now();
    vtTimesSubAtlas.push_back(time_Start);

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

        std::cout << "Got Atlas Update. 0. Merge map current map=" << mpRosAtlas->mv_merged_map_ids[0] << ", merge map=" << mpRosAtlas->mv_merged_map_ids[1] << std::endl;

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
    //else if(pCurrentMap && pCurrentMap != mpAtlas->GetCurrentMap() && pCurrentMap->GetId() < mpAtlas->GetCurrentMap()->GetId()) 
    //{
    //  mpAtlas->ChangeMap(pCurrentMap);
    //  pCurrentMap->SetCurrentMap();
    //  mpRosMap->mn_id = pCurrentMap->GetId(); //mpAtlas->GetCurrentMap()->GetId();
    //  pCurrentMap = mpAtlas->GetCurrentMap();
    //}
    
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*>& mFusedKFs = mpObserver->GetAllKeyFrames(); 
    std::map<std::string, ORB_SLAM3::MapPoint*>& mFusedMPs = mpObserver->GetAllMapPoints(); 

    //for(const auto& pair : mFusedKFs)
    //{
    //    if(pair.second)
    //        pair.second->SetLastModule(3);
    //}

    //for(const auto& pair : mFusedMPs)
    //{
    //    if(pair.second)
    //      pair.second->SetLastModule(3);
    //}
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
    std::cout << "mvp_erased_keyframe_ids.size()=" << mpRosMap->mvp_erased_keyframe_ids.size() << ", mvp_erased_mappoint_ids.size()=" << mpRosMap->mvp_erased_mappoint_ids.size() << std::endl;
    for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
    {
        unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
        ORB_SLAM3::KeyFrame* pEraseKF = mpObserver->GetKeyFrame(mnId);
        if(pEraseKF) // && pEraseKF->GetToBeErased())
        {
            std::cout << "Global. Delete KF=" << mnId << std::endl;
            mpKeyFrameDB->erase(pEraseKF);
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

    //if(mpRosAtlas->mb_map_merge && mpRosAtlas->mv_merged_map_ids.size() == 2)
    //{
    //    //std::cout << "# of Backup KFs=" << mpRosMap->mvp_backup_keyframes_ids.size() << ", # of Backup MPs=" << mpRosMap->mvp_backup_map_points_ids.size() << std::endl;
    //    for(const auto& pKF : pMergeMap->GetAllKeyFrames())
    //    {
    //        if(pKF)
    //        {
    //            pKF->UpdateMap(pCurrentMap);
    //            pCurrentMap->AddKeyFrame(pKF);

    //            for(const auto& pMP : pKF->GetMapPoints())
    //            {
    //              pMP->UpdateMap(pCurrentMap);
    //              pCurrentMap->AddMapPoint(pMP);
    //            }
    //        }
    //    }

    //    //for(size_t i=0;i<mpRosMap->mvp_backup_map_points_ids.size();++i)
    //    //{
    //    //    std::string mnId = mpRosMap->mvp_backup_map_points_ids[i];
    //    //    if(mpObserver->CheckIfMapPointExists(mnId))
    //    //    {
    //    //        ORB_SLAM3::MapPoint* pMP=mFusedMPs[mnId];
    //    //        pMP->UpdateMap(pCurrentMap);

    //    //    }
    //    //}

    //}



    std::cout << "Got Atlas Update. 7. KFs and MPs removed" << std::endl;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartUpdateAtlas = std::chrono::steady_clock::now();

    // Finally update Map object 
    // First create temporary Map from ROS data
    // Then Inject the data into real map (do not update KFs and MPs since those are already added)
    ORB_SLAM3::Map* tempMap = static_cast<ORB_SLAM3::Map*>(NULL);
    tempMap = Converter::MapConverter::RosMapToOrbMap(mpRosMap, tempMap);

    bool bKFUnprocessed = false;
    tempMap->PostLoad(mpKeyFrameDB, pSLAM->GetORBVocabulary(), mFusedKFs, mFusedMPs, mCameras, &bKFUnprocessed);
    mpObserver->InjectMap(tempMap, pCurrentMap, mpRosAtlas->from_module_id);
    
    mpObserver->mbMapIsUpToDate = true;
    // End of timer
    std::chrono::steady_clock::time_point time_EndUpdateAtlas = std::chrono::steady_clock::now();
    double timeUpdateAtlas = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateAtlas - time_StartUpdateAtlas).count();
    vdUpdateAtlas_ms.push_back(timeUpdateAtlas);

    //std::cout << "Got Atlas Update. 8. Map Injection completed" << std::endl;

    if(mpRosAtlas->mb_map_merge && pCurrentMap)
    {
        std::cout << "Got Atlas Update. 8. Remove merged map." << std::endl;
        mpAtlas->ChangeMap(pCurrentMap);
        pCurrentMap->SetCurrentMap();
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
        //mpLocalMapper->mbGBARunning = false;
        //if(mpObserver->GetTaskModule() == 1)
        //mpLocalMapper->Release();

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

    if(!mpRosMap)
        return;

    // Start of a timer -------------
    std::chrono::steady_clock::time_point time_StartRos2OrbProcMap = std::chrono::steady_clock::now();
    std::chrono::system_clock::time_point time_Start = std::chrono::system_clock::now();
    vtTimesSubMap.push_back(time_Start);
  

    // Get maps and the map where KF is
    // Get maps and the map where KF is
    std::map<unsigned long int, ORB_SLAM3::Map*>& mMaps = mpObserver->GetAllMaps();
    ORB_SLAM3::Map* pCurrentMap = mMaps[mpRosMap->mn_id];

    if(!pCurrentMap)
    {
        mpAtlas->CreateNewMap();
        pCurrentMap = mpAtlas->GetCurrentMap(); 
        pCurrentMap->attachDistributor(mpObserver);
    } 
    //else if(pCurrentMap && pCurrentMap != mpAtlas->GetCurrentMap() && pCurrentMap->GetId() < mpAtlas->GetCurrentMap()->GetId()) 
    //{
    //  mpAtlas->ChangeMap(pCurrentMap);
    //  pCurrentMap->SetCurrentMap();
    //  mpRosMap->mn_id = pCurrentMap->GetId(); //mpAtlas->GetCurrentMap()->GetId();
    //  pCurrentMap = mpAtlas->GetCurrentMap();
    //}

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

    mpObserver->mbMapIsUpToDate = true;

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
        for(size_t i=0; i<mpRosMap->mvp_erased_keyframe_ids.size(); ++i)
        {
            unsigned long int mnId = mpRosMap->mvp_erased_keyframe_ids[i];
            ORB_SLAM3::KeyFrame* pEraseKF = mpObserver->GetKeyFrame(mnId);
            if(pEraseKF) // && pEraseKF->GetToBeErased())
            {
                std::cout << "Local. Delete KF=" << mnId << std::endl;
                mpKeyFrameDB->erase(pEraseKF);
                pEraseKF->SetBadFlag();
                mpObserver->EraseKeyFrame(mnId);
            }
        }


        for(size_t i=0; i<mpRosMap->mvp_erased_mappoint_ids.size(); ++i)
        {
            std::string mnId = mpRosMap->mvp_erased_mappoint_ids[i];
            ORB_SLAM3::MapPoint* pEraseMP=mpObserver->GetMapPoint(mnId);
            if(pEraseMP)
            {
                pEraseMP->SetBadFlag();
                mpObserver->EraseMapPoint(mnId);
            }
        }

    }


    std::vector<ORB_SLAM3::MapPoint*> mvpTempMPs;
    std::vector<bool> mvbNewMPs;
    mvpTempMPs.reserve(2000);
    mvbNewMPs.reserve(2000);


    int mnTaskModule = mpObserver->GetTaskModule();
    for(int i=0; i<mpRosMap->msp_map_points.size(); ++i)
    {
        orbslam3_interfaces::msg::MapPoint::SharedPtr mpRosMP = std::make_shared<orbslam3_interfaces::msg::MapPoint>(mpRosMap->msp_map_points[i]);
        if(mnTaskModule==3 && mpRosMP->mn_last_module == 3)
            continue;

        mpRosMP->mp_map_id = mpAtlas->GetCurrentMap()->GetId();
        ORB_SLAM3::MapPoint* tempMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
        ORB_SLAM3::MapPoint* mpExistingMP = mpObserver->GetMapPoint(mpRosMP->m_str_hex_id);
        if(mpExistingMP)//mFusedMPs.find(mpRosMP->m_str_hex_id) != mFusedMPs.end())
        {
          ORB_SLAM3::MapPoint* mpCopyMP = new ORB_SLAM3::MapPoint(*mpExistingMP);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpCopyMP);
          mvbNewMPs.push_back(false);
        } else {

          mpExistingMP = static_cast<ORB_SLAM3::MapPoint*>(NULL);
          tempMP = mpObserver->ConvertMapPoint(mpRosMP, mpExistingMP);
          mvbNewMPs.push_back(true);
          mpObserver->AddMapPoint(tempMP);
        }

        mvpTempMPs.push_back(tempMP);
    }
    
    for(ORB_SLAM3::MapPoint* tempMP : mvpTempMPs)
    {
        
        tempMP->PostLoad(mFusedKFs, mFusedMPs, &bKFUnprocessed);
    }


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

      //tempMP->SetReferenceKeyFrame(pKF);

      mpObserver->InjectMapPoint(tempMP, mpExistingMP, mbNew);
    }

    //if(mpRosMap->ms_bad_maps.size() > 0)
    //{
    //    for(const auto& mnId : mpRosMap->ms_bad_maps)
    //    {
    //        ORB_SLAM3::Map* pTempMap = mMaps[mnId];
    //        if(pTempMap && !pTempMap->IsBad())
    //        {
    //            mpAtlas->SetMapBad(pTempMap);
    //            //mpObserver->EraseMap(pTempMap);
    //        }

    //    }
    //}


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

bool MapHandler::CheckNewUpdates()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    return(mpAtlas->GetCurrentMap()->GetUpdatedKFIds().size()>0);
}

bool MapHandler::CheckPubLocalMaps()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    std::chrono::high_resolution_clock::time_point msLastMUStop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(msLastMUStop - msLastMUStart);
    auto dCount = duration.count();

    if(dCount > mnMapFreq_ms && msUpdatedLocalKFs.size() >= 2)
    {
        return true;
    }
    else if(dCount>mnMapFreq_ms && msUpdatedLocalMPs.size()>0) {
        return true;
    }

    return false;

    //if(mpAtlas->GetCurrentMap()->KeyFramesInMap() <= 2)
    //return((dCount > mnMapFreq_ms && msUpdatedLocalKFs.size() >= 2)); //|| (dCount>mnMapFreq_ms && msUpdatedLocalMPs.size()>0));
    //else
    //  return(msUpdatedLocalKFs.size() >= 1 && dCount > mnMapFreq_ms);
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

    return(dCount > mnGlobalMapFreq_ms && (!mlpAtlasPubQueue.empty() || !msUpdatedGlobalKFs.empty()));
}

bool MapHandler::CheckSubGlobalMaps()
{
    unique_lock<mutex> lock(mMutexNewAtlas);
    return mpNewRosAtlas != NULL; //&& mpKeyFrameSubscriber->KeyFrameUpdatesInQueue() < 10);
}

void MapHandler::InsertNewUpdatedLocalKF(ORB_SLAM3::KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexUpdates);
    msUpdatedLocalKFs.insert(pKF->mnId);
}


void MapHandler::InsertNewUpdatedLocalMP(ORB_SLAM3::MapPoint* pMP)
{
    unique_lock<mutex> lock(mMutexUpdates);
    msUpdatedLocalMPs.insert(pMP->mstrHexId);
}


void MapHandler::InsertNewUpdatedLocalMP(std::string mnId)
{
    unique_lock<mutex> lock(mMutexUpdates);
    msUpdatedLocalMPs.insert(mnId);
}



void MapHandler::InsertNewPubLocalMap()
{
    double timeSinceReset = mpObserver->TimeSinceReset();
    if(timeSinceReset < 100)
        return;

    unique_lock<mutex> lock(mMutexNewMaps);

    //if(!mlpAtlasSubQueue.empty())
    //    return;
    
    //if(mpNewRosAtlas != NULL)
    //    return;

    std::cout << "***** SEND LOCAL MAP ******" << std::endl;

    ORB_SLAM3::Map* pMap = mpAtlas->GetCurrentMap();
    const std::set<unsigned long int>& tempUpdatedKFs=pMap->GetUpdatedKFIds();
    std::set<std::string> tempUpdatedMPs=pMap->GetUpdatedMPIds();
    
    const std::set<std::string>& tempErasedMPs=pMap->GetErasedMPIds();
    const std::set<unsigned long int>& tempErasedKFs=pMap->GetErasedKFIds();





    {
        unique_lock<mutex> lock2(mMutexUpdates);
        for(auto const& mnId : tempErasedKFs)
        {
          msErasedKFs.insert(mnId);
          //mpObserver->EraseKeyFrame(mnId);
        }

        for(auto const& mnId : tempErasedMPs)
        {
          msErasedMPs.insert(mnId);
          mpObserver->EraseMapPoint(mnId);
        }
    }

    //msErasedKFs.insert(tempErasedKFs.begin(), tempErasedKFs.end());
    //msErasedMPs.insert(tempErasedMPs.begin(), tempErasedMPs.end());
        
    {
        unique_lock<mutex> lock2(mMutexUpdates);
        msUpdatedLocalKFs.insert(tempUpdatedKFs.begin(), tempUpdatedKFs.end());
    }

    for(std::set<std::string>::iterator it = tempUpdatedMPs.begin(); it != tempUpdatedMPs.end(); ++it)
    {
        InsertNewUpdatedLocalMP(*it);
    }
        
    if(mpLocalMapper->mbGBARunning)
    {
        //for(const auto& mnId : tempUpdatedKFs)
        //{
        //  msToBeErasedKFs.insert(mnId);
        //}

        for(const auto& mnId : mpLocalMapper->msNewMapPointIds)
        {
          msToBeErasedMPs.insert(mnId);
        }
    }



    pMap->ClearErasedData();
    pMap->ClearUpdatedMPIds();
    pMap->ClearUpdatedKFIds();
    
    // Make next update instant
    mnMapFreq_ms=0;
    //if(mpLocalMapper->mbGBARunning)
    //    maxUpdateN=15;
    //else
    maxUpdateN=5;
    mnPubIters=0;

    // Make next update instant
    //mnMapFreq_ms=100;

    std::cout << "Insert new pub local map, updated KFs=" << msUpdatedLocalKFs.size() << ", MPs=" << msUpdatedLocalMPs.size() << ", in map=" << pMap->GetUpdatedKFIds().size() << std::endl;
    //mlpMapPubQueue.push_back(pMap);
}

void MapHandler::InsertNewPubLocalMap(ORB_SLAM3::Map* pMap)
{
    unique_lock<mutex> lock(mMutexNewMaps);

    //if(!mlpAtlasSubQueue.empty())
    //    return;
    
    //if(mpNewRosAtlas != NULL)
    //    return;

    std::cout << "***** SEND LOCAL MAP ******" << std::endl;

    const std::set<unsigned long int>& tempUpdatedKFs=pMap->GetUpdatedKFIds();
    std::set<std::string> tempUpdatedMPs=pMap->GetUpdatedMPIds();
    
    const std::set<std::string>& tempErasedMPs=pMap->GetErasedMPIds();
    const std::set<unsigned long int>& tempErasedKFs=pMap->GetErasedKFIds();





    {
        unique_lock<mutex> lock2(mMutexUpdates);
        for(auto const& mnId : tempErasedKFs)
        {
          msErasedKFs.insert(mnId);
          //mpObserver->EraseKeyFrame(mnId);
        }

        for(auto const& mnId : tempErasedMPs)
        {
          msErasedMPs.insert(mnId);
          mpObserver->EraseMapPoint(mnId);
        }
    }

    //msErasedKFs.insert(tempErasedKFs.begin(), tempErasedKFs.end());
    //msErasedMPs.insert(tempErasedMPs.begin(), tempErasedMPs.end());
        
    {
        unique_lock<mutex> lock2(mMutexUpdates);
        msUpdatedLocalKFs.insert(tempUpdatedKFs.begin(), tempUpdatedKFs.end());
    }

    for(std::set<std::string>::iterator it = tempUpdatedMPs.begin(); it != tempUpdatedMPs.end(); ++it)
    {
        InsertNewUpdatedLocalMP(*it);
    }
        
    if(mpLocalMapper->mbGBARunning)
    {
        //for(const auto& mnId : tempUpdatedKFs)
        //{
        //  msToBeErasedKFs.insert(mnId);
        //}

        for(const auto& mnId : mpLocalMapper->msNewMapPointIds)
        {
          msToBeErasedMPs.insert(mnId);
        }
    }



    pMap->ClearErasedData();
    pMap->ClearUpdatedMPIds();
    pMap->ClearUpdatedKFIds();
    
    // Make next update instant
    mnMapFreq_ms=0;
    //if(mpLocalMapper->mbGBARunning)
    //    maxUpdateN=15;
    //else
    maxUpdateN=5;
    mnPubIters=0;

    // Make next update instant
    //mnMapFreq_ms=100;

    std::cout << "Insert new pub local map, updated KFs=" << msUpdatedLocalKFs.size() << ", MPs=" << msUpdatedLocalMPs.size() << ", in map=" << pMap->GetUpdatedKFIds().size() << std::endl;
    //mlpMapPubQueue.push_back(pMap);
}

void MapHandler::InsertNewSubLocalMap(orbslam3_interfaces::msg::Map::SharedPtr pRosMap)
{
    if(!mlpAtlasPubQueue.empty() || mpLoopCloser->isRunningGBA() || !mpLoopCloser->CheckIfRunning())
      return;
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
      for(size_t i=0;i<pRosMap->msp_map_points.size();++i)
      {
          unsigned long int mnId = pRosMap->msp_map_points[i].mn_id;
          mpObserver->UpdateMaxMPId(mnId);
          
      }

      mpObserver->UpdateMaxMPId(ORB_SLAM3::MapPoint::nNextId);
      ORB_SLAM3::MapPoint::nNextId=mpObserver->GetMaxMPId()+1;



    std::cout << "***** mvp_erased_mappoint_ids.size()=" << pRosMap->mvp_erased_mappoint_ids.size() << "******" << std::endl;
      //for(auto const& mnId : pRosMap->mvp_erased_mappoint_ids)
      //{
      //    mpObserver->EraseMapPoint(mnId);
      //}

      mpObserver->mbMapIsUpToDate = false;
      for(orbslam3_interfaces::msg::KeyFrameUpdate& pRosKF : pRosMap->msp_keyframes)
      {
        //if(!mpKeyFrameQueue[pRosKF.mn_id]) //|| (mpObserver && mpKeyFrameQueue[pRosKF.mn_id] && mpKeyFrameQueue[pRosKF.mn_id]->mn_last_module < mpObserver->GetTaskModule()))
            orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(pRosKF);
            mpRosKF->from_module_id = pRosMap->from_module_id;
            //if(mpAtlas->GetCurrentMap()->KeyFramesInMap() > 10)
            //{
            //    if(mpObserver->HasKeyFrameBeenErased(mpRosKF->mn_id))
            //        mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKF);

            //} else 
            //{
            //if(mpObserver->GetTaskModule()==3&&!mpRosKF->mb_lc_done && mpAtlas->GetCurrentMap()->GetLastBigChangeIdx()>0)
            //    continue;
            mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKF);
            //}
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
    double timeSinceReset = mpObserver->TimeSinceReset();
    if(timeSinceReset < 10000)
        return;
    
    ResetLocalQueue();
    mpKeyFrameSubscriber->ResetQueue(false);

    {
        unique_lock<mutex> lock(mMutexNewAtlas);
        mlpAtlasPubQueue.push_back(mtAtlasUpdate);
    }

    std::cout << "***** SEND GLOBAL MAP ******" << std::endl;
    const std::set<unsigned long int>& tempUpdatedKFs = mpAtlas->GetCurrentMap()->GetUpdatedKFIds();
    const std::set<std::string>&  tempUpdatedMPs = mpAtlas->GetCurrentMap()->GetUpdatedMPIds();

    if(std::get<0>(mtAtlasUpdate))
    {
        unsigned long int pCurId = std::get<2>(mtAtlasUpdate)[1]; 
        unsigned long int pMergeId = std::get<2>(mtAtlasUpdate)[0]; 
        mpObserver->EraseMap(pCurId);
        mpObserver->EraseMap(pMergeId);
        mpObserver->AddMap(mpAtlas->GetCurrentMap());
    }

    //for(const auto& pKF : mpAtlas->GetCurrentMap()->GetAllKeyFrames())
    //{
    //    tempUpdatedKFs.insert(pKF->mnId);

    //    for(const auto& pMP : pKF->GetMapPoints())
    //    {
    //        tempUpdatedMPs.insert(pMP->mstrHexId);
    //    }
    //}

    const std::set<std::string>& tempErasedMPs=mpAtlas->GetCurrentMap()->GetErasedMPIds();
    const std::set<unsigned long int>& tempErasedKFs=mpAtlas->GetCurrentMap()->GetErasedKFIds();

    {
        unique_lock<mutex> lock(mMutexUpdates);
        msErasedKFs.insert(tempErasedKFs.begin(), tempErasedKFs.end());
        msErasedMPs.insert(tempErasedMPs.begin(), tempErasedMPs.end());
        
        msUpdatedGlobalKFs.insert(tempUpdatedKFs.begin(), tempUpdatedKFs.end());
        msUpdatedGlobalMPs.insert(tempUpdatedMPs.begin(), tempUpdatedMPs.end());
    }

    mpAtlas->GetCurrentMap()->ClearUpdatedKFIds();
    mpAtlas->GetCurrentMap()->ClearUpdatedMPIds();
      
    //if(std::get<0>(mtAtlasUpdate)){
    unique_lock<mutex> lock(mMutexUpdates);
    std::cout << "Current map ID=" << mpAtlas->GetCurrentMap()->GetId() << ", KFs in map=" << mpAtlas->GetCurrentMap()->GetAllKeyFrames().size() << ", MPs=" << mpAtlas->GetCurrentMap()->GetAllMapPoints().size() << ", number of maps in Atlas=" << mpAtlas->GetAllMaps().size() << std::endl;
    for(const auto& pKFi : mpAtlas->GetCurrentMap()->GetAllKeyFrames())
    {
        if(pKFi)
        {
            pKFi->mbLCDone=true;
            pKFi->SetLastModule(3);
            msUpdatedGlobalKFs.insert(pKFi->mnId);
        }
    }

    for(const auto& pMPi : mpAtlas->GetCurrentMap()->GetAllMapPoints())
    {
        if(pMPi)
            msUpdatedGlobalMPs.insert(pMPi->mstrHexId);
    }

    //}
    //    mpObserver->EraseMap(std::get<2>(mtAtlasUpdate)[0]);
    
    std::cout << "msUpdatedGlobalMPs=" << msUpdatedGlobalMPs.size() << ", msUpdatedGlobalKFs=" << msUpdatedGlobalKFs.size() << ", msErasedMPs.size()=" << msErasedMPs.size() << ", msErasedKFs.size()=" << msErasedKFs.size() << std::endl;

    // Make next update instant
    mnGlobalMapFreq_ms=0;
    maxUpdateGlobalN=20;
    
}

void MapHandler::InsertNewSubGlobalMap(orbslam3_interfaces::msg::Atlas::SharedPtr pRosAtlas)
{
    double timeSinceReset = mpObserver->TimeSinceReset();
    if(timeSinceReset < 200)
        return;

    std::cout << "***** GOT GLOBAL MAP, BATCH=" << pRosAtlas->mn_batch_number << " ******" << std::endl;
    // If loop closer is received, empty all map updates and just perform atlas update
    //if(pRosAtlas->mn_batch_number == 0)//if(pRosAtlas->mb_loop_closer)
    //{
    //  unique_lock<mutex> lock(mMutexNewMaps);
    //  unique_lock<mutex> lock2(mMutexNewAtlas);

    //  //mpLocalMapper->RequestStop();
    //  //while(!mpLocalMapper->isStopped())
    //  //    usleep(3000);

    //  {
    //      unique_lock<mutex> lock3(mMutexUpdates);
    //      msErasedMPs.clear();
    //      msErasedKFs.clear();
    //      msUpdatedLocalKFs.clear();
    //      msUpdatedLocalMPs.clear();
    //      //mpKeyFrameSubscriber->ResetQueue();

    //  }
    //  



    //  //mlpMapSubQueue.clear();

    //  //mlpAtlasPubQueue.clear();
    //  //mlpAtlasSubQueue.clear();

    //  //mlpAtlasSubQueue.push_back(pRosAtlas);
    //}
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
    
    if(pRosAtlas->mb_map_merge)
        mnLastGlobalAction=0;
    else if(pRosAtlas->mb_loop_closer)
        mnLastGlobalAction=1;

    {
      //unique_lock<mutex> lock(mMutexNewMaps);
      //unique_lock<mutex> lock2(mMutexNewAtlas);
      //if(mpLocalMapper->isStopped())
      //mpLocalMapper->Release();
      if(pRosAtlas->mn_batch_number == 0)//if(pRosAtlas->mb_loop_closer)
      {
          mpObserver->mbMapIsUpToDate = false;
          mpObserver->mbReferenceUpdated=false;
          //mpLocalMapper->RequestStop();
          //ResetLocalQueue();
          //mpLocalMapper->EmptyQueue();
          std::cout << "msToBeErasedMPs.size()=" << msToBeErasedMPs.size() << ", msToBeErasedKFs.size()="<<msToBeErasedKFs.size() << std::endl;
          
          for(size_t i=0;i<pRosAtlas->mp_current_map.mvp_backup_keyframes_ids.size();++i)
          {
              long int mnId = static_cast<long int>(pRosAtlas->mp_current_map.mvp_backup_keyframes_ids[i]);
              ORB_SLAM3::KeyFrame* pKF=mpObserver->GetKeyFrame(mnId);
              std::cout << mnId << ", found=" << (pKF!=nullptr) << ". ";
              if(pKF)
              {
                  if(mnId >= mpObserver->mnUpdateReferenceId)
                  {
                    mpObserver->mnUpdateReferenceId=mnId;
                  }
                  pKF->SetLastModule(3);
                  //InsertNewUpdatedLocalKF(pKF);
              }
          }
          std::cout << "." << std::endl;

          std::cout << " =!=!=!==!=!=!=!=!==! mnUpdateReferenceId=" << mpObserver->mnUpdateReferenceId << ", mbReferenceUpdated=" << mpObserver->mbReferenceUpdated << std::endl;

          for(size_t i=0;i<pRosAtlas->mp_current_map.mvp_backup_map_points_ids.size();++i)
          {
              std::string mnId = pRosAtlas->mp_current_map.mvp_backup_map_points_ids[i];
              ORB_SLAM3::MapPoint* pMP=mpObserver->GetMapPoint(mnId);
              if(pMP)
              {
                  pMP->SetLastModule(3);
                  //InsertNewUpdatedLocalMP(pMP);
              }
          }


          if(pRosAtlas->mb_map_merge)
          {
              //std::cout << "# of Backup KFs=" << mpRosMap->mvp_backup_keyframes_ids.size() << ", # of Backup MPs=" << mpRosMap->mvp_backup_map_points_ids.size() << std::endl;
                
                ORB_SLAM3::Map* pCurrentMap;
                ORB_SLAM3::Map* pMergeMap;
                for(const auto& pMap : mpAtlas->GetAllMaps())
                {
                  if(pMap && pMap->GetId() == pRosAtlas->mv_merged_map_ids[1])
                      pCurrentMap = pMap; 
                  else if(pMap && pMap->GetId() == pRosAtlas->mv_merged_map_ids[0])
                      pMergeMap= pMap; 
                }
                
                //unsigned long int pCurId = pCurrentMap->GetId();
                //unsigned long int pMergeId = pMergeMap->GetId();
                //pCurrentMap->ChangeId(pMergeId);
                //pMergeMap->ChangeId(pCurId);

                //mpObserver->EraseMap(pCurId);
                //mpObserver->EraseMap(pMergeId);

                //mpObserver->AddMap(pCurrentMap);
                //mpObserver->AddMap(pMergeMap);
                
                mpAtlas->ChangeMap(pCurrentMap);
                pCurrentMap->SetCurrentMap();

                std::cout << "Current map id=" << pCurrentMap->GetId() << ", merged map id=" << pMergeMap->GetId() << std::endl;

                std::vector<ORB_SLAM3::KeyFrame*> vKFs = pMergeMap->GetAllKeyFrames();
                std::vector<ORB_SLAM3::MapPoint*> vMPs = pMergeMap->GetAllMapPoints();
                for(const auto& pKFi : vKFs)
                {
                    if(pKFi)
                    {
                      pKFi->UpdateMap(pCurrentMap);
                      pCurrentMap->AddKeyFrame(pKFi);
                      pMergeMap->EraseKeyFrame(pKFi);

                    }

                }

                for(const auto& pMPi : vMPs)
                {
                    if(pMPi)
                    {
                      pMPi->UpdateMap(pCurrentMap);
                      pCurrentMap->AddMapPoint(pMPi);
                      pMergeMap->EraseMapPoint(pMPi);
                    }

                }
                
                mpAtlas->SetMapBad(pMergeMap);

                //for(size_t i=0;i<pRosAtlas->mp_current_map.msp_keyframes.size();++i)
                //{
                //    unsigned long int mnId = pRosAtlas->mp_current_map.msp_keyframes[i].mn_id;
                //    ORB_SLAM3::KeyFrame* pKF=mpObserver->GetKeyFrame(mnId);
                //    if(pKF)
                //    {
                //      pKF->UpdateMap(pCurrentMap);
                //      pCurrentMap->AddKeyFrame(pKF);
                //      pMergeMap->EraseKeyFrame(pKF);
                //    }

                //    for(const auto& pRosMP : pRosAtlas->mp_current_map.msp_keyframes[i].mvp_map_points)
                //    {
                //        ORB_SLAM3::MapPoint* pMP=mpObserver->GetMapPoint(pRosMP.m_str_hex_id);
                //        if(pMP)
                //        {
                //          pMP->UpdateMap(pCurrentMap);
                //          pCurrentMap->AddMapPoint(pMP);
                //          pMergeMap->EraseMapPoint(pMP);
                //        }
                //    }
                //}

                //for(size_t i=0;i<pRosAtlas->mp_current_map.msp_map_points.size();++i)
                //{
                //    std::string mnId = pRosAtlas->mp_current_map.msp_map_points[i].m_str_hex_id;
                //    ORB_SLAM3::MapPoint* pMP=mpObserver->GetMapPoint(mnId);
                //    if(pMP)
                //    {
                //      pMP->UpdateMap(pCurrentMap);
                //      pCurrentMap->AddMapPoint(pMP);
                //      pMergeMap->EraseMapPoint(pMP);
                //      //for(const auto& pMP : pKF->GetMapPoints())
                //      //{
                //      //  if(pMP)
                //      //  {
                //      //      pMP->UpdateMap(mpAtlas->GetCurrentMap());
                //      //      mpAtlas->GetCurrentMap()->AddMapPoint(pMP);

                //      //  }
                //      //}
                //    }
                //}
          }



          //for(const auto& mnId : msToBeErasedMPs)
          //{
          //  ORB_SLAM3::MapPoint* pEraseMP = mpObserver->GetMapPoint(mnId);
          //  if(pEraseMP)
          //  {
          //      pEraseMP->SetLastModule(3);
          //      //pEraseMP->SetBadFlag();
          //      //mpObserver->EraseMapPoint(mnId);
          //      //mpAtlas->GetCurrentMap()->EraseMapPoint(pEraseMP);
          //  }
          //  //mpObserver->msAllErasedMPIds.insert(mnId);

          //}

          for(const auto& mnId : msToBeErasedKFs)
          {
            ORB_SLAM3::KeyFrame* pUpdateKF = mpObserver->GetKeyFrame(mnId);
            if(pUpdateKF)
            {
                pUpdateKF->SetLastModule(3);
                //pUpdateKF->GetMap()->EraseKeyFrame(pUpdateKF);
                //mpKeyFrameDB->erase(pUpdateKF);
                //mpLocalMapper->InsertKeyframeFromRos(pUpdateKF);
            }
          }
          

          //msToBeErasedMPs.clear();
          //msToBeErasedKFs.clear();

          mpNewRosAtlas = pRosAtlas;
      } 

      orbslam3_interfaces::msg::Map::SharedPtr pRosMap = std::make_shared<orbslam3_interfaces::msg::Map>(pRosAtlas->mp_current_map);
      for(orbslam3_interfaces::msg::KeyFrameUpdate& pRosKF : pRosMap->msp_keyframes)
      {
        //if(!mpKeyFrameQueue[pRosKF.mn_id]) //|| (mpObserver && mpKeyFrameQueue[pRosKF.mn_id] && mpKeyFrameQueue[pRosKF.mn_id]->mn_last_module < mpObserver->GetTaskModule()))
            orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr mpRosKF = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(pRosKF);
            mpRosKF->from_module_id = pRosAtlas->from_module_id;

            //if(mpObserver->HasKeyFrameBeenErased(mpRosKF->mn_id))
            mpKeyFrameSubscriber->InsertNewKeyFrame(mpRosKF);
            //mpKeyFrameQueue[pRosKF.mn_id] = std::make_shared<orbslam3_interfaces::msg::KeyFrameUpdate>(pRosKF);
      }
      

      if(pRosAtlas->mb_last_batch)
      {
          if(pRosAtlas->mb_map_merge)
          {
              std::map<unsigned long int, ORB_SLAM3::Map*>& mMaps = mpObserver->GetAllMaps();
              ORB_SLAM3::Map* pMergeMap = mMaps[pRosAtlas->mv_merged_map_ids[0]];
              std::cout << "------------ Removing map=" << pRosAtlas->mv_merged_map_ids[0] << std::endl;
              mpAtlas->RemoveBadMaps();
              mpObserver->EraseMap(pMergeMap);
              //mpTracker->UpdateReference();

          }
          mpLocalMapper->mbGBARunning = false;
          mpLocalMapper->Release();
          mnLastGlobalAction=-1;
      } 

      //for(const orbslam3_interfaces::msg::MapPoint& pRosMP : pRosMap->msp_map_points)
      //{
      //  if(!mpMapPointQueue[pRosMP.m_str_hex_id]) //|| (mpObserver && mpMapPointQueue[pRosMP.m_str_hex_id] && mpMapPointQueue[pRosMP.m_str_hex_id]->mn_last_module < mpObserver->GetTaskModule()))
      //      mpMapPointQueue[pRosMP.m_str_hex_id] = std::make_shared<orbslam3_interfaces::msg::MapPoint>(pRosMP);
      //}

      //pRosMap->msp_keyframes.clear();
      //pRosMap->msp_map_points.clear();

      //if(pRosAtlas->mb_last_batch)//&& mpObserver->GetTaskModule() != 1)


    }
}


void MapHandler::ResetLocalQueue()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    unique_lock<mutex> lock2(mMutexUpdates);

    msUpdatedLocalMPs.clear();
    msUpdatedLocalKFs.clear();

    mpNewRosMap = std::shared_ptr<orbslam3_interfaces::msg::Map>(NULL); //static_cast<orbslam3_interfaces::msg::Map>(NULL);
}

void MapHandler::ResetQueue()
{
    unique_lock<mutex> lock(mMutexNewMaps);
    unique_lock<mutex> lock2(mMutexNewAtlas);
    
    mlpAtlasPubQueue.clear();
    mlpMapPubQueue.clear();


    {
        unique_lock<mutex> lock3(mMutexUpdates);
        msUpdatedLocalMPs.clear();
        msUpdatedLocalKFs.clear();

        msErasedKFs.clear();
        msErasedMPs.clear();
    }

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
