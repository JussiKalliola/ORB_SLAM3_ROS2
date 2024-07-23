
#ifndef DISTRIBUTOR_MAP_HANDLER_H
#define DISTRIBUTOR_MAP_HANDLER_H

#include "System.h"
//#include "Frame.h"
//#include "KeyFrame.h"
//#include "Map.h"
//#include "MapPoint.h"
//#include "Tracking.h"
//#include "Converter.h"
//#include "KeyFrameDatabase.h"
//#include "GeometricCamera.h"

#include "../Converter/Converter.hpp"
//#include "orbslam3_interfaces/Converter.hpp"
//#include "./Observer.hpp"
#include <stdlib.h>
#include <mutex>

//namespace TempDistributor
//{

class SlamWrapperNode;
class Observer;
class KeyFrameSubscriber;
class MapConverter;

class MapHandler 
{
  public:  
    MapHandler();
    ~MapHandler();

    // main function
    void Run();

    int GlobalMapsInQueue();
    int LocalMapsInQueue();

    void InsertNewPubLocalMap(ORB_SLAM3::Map* pM);
    void InsertNewSubLocalMap(orbslam3_interfaces::msg::Map::SharedPtr pRosMap);

    void InsertNewPubGlobalMap(std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate);
    void InsertNewSubGlobalMap(orbslam3_interfaces::msg::Atlas::SharedPtr mRosAtlas);
    
    void AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM);
    void AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node);
    void AttachObserver(std::shared_ptr<Observer> pObserver);
    void AttachKFSubscriber(KeyFrameSubscriber* pKFSubscriber);

    void RequestFinish();
    bool isFinished();


    void ResetQueue();

    //vector<long int> vnNumberOfNewMapPoints;
    //vector<long int> vnNumberOfUpdatedMapPoints;
    vector<double> vdRos2OrbConvMap_ms;
    vector<double> vdRos2OrbKFConvMap_ms;
    vector<double> vdRos2OrbMPConvMap_ms;
    vector<double> vdRos2OrbDataConvMap_ms;
    vector<double> vdPostLoadKFMap_ms;
    vector<double> vdPostLoadMPMap_ms;
    vector<double> vdInjectKFMap_ms;
    vector<double> vdInjectMPMap_ms;
    vector<double> vdUpdateMap_ms; // Convert (without KFs and MPs), postload, inject
    vector<double> vdRos2OrbProcMap_ms;
    vector<int> vnNumberOfMPs;
    vector<int> vnNumberOfKFs;
    vector<int> vnNumberOfMPsTotal;
    vector<int> vnNumberOfKFsTotal;

    vector<std::chrono::steady_clock::time_point> vtTimesSubMap;

    vector<double> vdRos2OrbDataConvAtlas_ms;
    vector<double> vdRos2OrbKFConvAtlas_ms;
    vector<double> vdPostLoadKFAtlas_ms;
    vector<double> vdPostLoadMPAtlas_ms;
    vector<double> vdInjectKFAtlas_ms;
    vector<double> vdInjectMPAtlas_ms;
    vector<double> vdUpdateAtlas_ms; // Convert (without KFs and MPs), postload, inject
    vector<double> vdRos2OrbProcAtlas_ms;
    vector<std::chrono::steady_clock::time_point> vtTimesSubAtlas;

    vector<double> vdPreSaveMap_ms;
    vector<double> vdOrb2RosConvMap_ms;
    vector<double> vdOrb2RosProcMap_ms;
    vector<std::chrono::steady_clock::time_point> vtTimesPubMap;

    vector<double> vdPreSaveAtlas_ms;
    vector<double> vdOrb2RosConvAtlas_ms;
    vector<double> vdOrb2RosProcAtlas_ms;
    vector<std::chrono::steady_clock::time_point> vtTimesPubAtlas;


  protected:  
    bool CheckPubLocalMaps();
    bool CheckSubLocalMaps();

    bool CheckPubGlobalMaps();
    bool CheckSubGlobalMaps();

    void ProcessNewPubLocalMap();
    void ProcessNewSubLocalMap();
    void ProcessNewSubLocalMap2();

    void ProcessNewPubGlobalMap();
    void ProcessNewSubGlobalMap();
    void ProcessNewSubGlobalMap2();

    std::set<std::string> msErasedMPs;
    std::set<unsigned long int> msUpdatedLocalKFs;
    std::set<std::string> msUpdatedLocalMPs;

    std::set<unsigned long int> msErasedKFs;
    std::set<unsigned long int> msUpdatedGlobalKFs;
    std::set<std::string> msUpdatedGlobalMPs;
    std::mutex mMutexUpdates;

    std::map<unsigned long int, orbslam3_interfaces::msg::KeyFrameUpdate::SharedPtr> mpKeyFrameQueue;
    std::map<std::string, orbslam3_interfaces::msg::MapPoint::SharedPtr> mpMapPointQueue;
    orbslam3_interfaces::msg::Map::SharedPtr mpNewRosMap;
    orbslam3_interfaces::msg::Atlas::SharedPtr mpNewRosAtlas;

    std::list<ORB_SLAM3::Map*> mlpMapPubQueue;
    std::list<orbslam3_interfaces::msg::Map::SharedPtr> mlpMapSubQueue;
    std::mutex mMutexNewMaps;
    
    std::list<std::tuple<bool, bool, std::vector<unsigned long int>>> mlpAtlasPubQueue;
    std::list<orbslam3_interfaces::msg::Atlas::SharedPtr> mlpAtlasSubQueue;
    std::mutex mMutexNewAtlas;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    std::chrono::high_resolution_clock::time_point msLastMUStart;
    std::chrono::high_resolution_clock::time_point msLastGlobalMUStart;

    int mnMapFreq_ms;
    int maxUpdateN;
    int mnGlobalMapFreq_ms;
    int maxUpdateGlobalN;

  private:  
    int mnPubIters;
    int mnAtlasBatchNumber;

    // ORB_SLAM3
    ORB_SLAM3::System* pSLAM;
    ORB_SLAM3::Tracking* mpTracker;
    ORB_SLAM3::LocalMapping* mpLocalMapper;
    ORB_SLAM3::LoopClosing* mpLoopCloser;
    ORB_SLAM3::Atlas* mpAtlas;    
    ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB; 

    std::shared_ptr<SlamWrapperNode> pSLAMNode;

    std::shared_ptr<Observer> mpObserver;
    KeyFrameSubscriber* mpKeyFrameSubscriber;
};
//}

#endif // DISTRIBUTOR_MAP_HANDLER_H
