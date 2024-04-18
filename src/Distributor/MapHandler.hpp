
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

#include "orbslam3_interfaces/Converter.hpp"
//#include "./Observer.hpp"
#include <stdlib.h>
#include <mutex>

//namespace TempDistributor
//{

class SlamWrapperNode;
class Observer;
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

    void RequestFinish();
    bool isFinished();

  protected:  
    bool CheckPubLocalMaps();
    bool CheckSubLocalMaps();

    bool CheckPubGlobalMaps();
    bool CheckSubGlobalMaps();

    void ProcessNewPubLocalMap();
    void ProcessNewSubLocalMap();

    void ProcessNewPubGlobalMap();
    void ProcessNewSubGlobalMap();

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


  private:  
    // ORB_SLAM3
    ORB_SLAM3::System* pSLAM;
    ORB_SLAM3::Tracking* mpTracker;
    ORB_SLAM3::LocalMapping* mpLocalMapper;
    ORB_SLAM3::LoopClosing* mpLoopCloser;
    ORB_SLAM3::Atlas* mpAtlas;    
    ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB; 

    std::shared_ptr<SlamWrapperNode> pSLAMNode;

    std::shared_ptr<Observer> mpObserver;
};
//}

#endif // DISTRIBUTOR_MAP_HANDLER_H
