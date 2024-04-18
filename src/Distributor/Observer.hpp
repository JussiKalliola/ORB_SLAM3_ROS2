#ifndef DISTRIBUTOR_OBSERVER_H
#define DISTRIBUTOR_OBSERVER_H

#include "Distributor.h"
//#include "orbslam3_interfaces/Converter.hpp"
//#include "orbslam3_interfaces/KeyFrameConverter.hpp"
//#include "orbslam3_interfaces/MapConverter.hpp"
//#include "orbslam3_interfaces/AtlasConverter.hpp"
//#include "../slam/slam-wrapper-node.hpp"

#include "./KeyFramePublisher.hpp"
#include "./MapHandler.hpp"
#include "./KeyFrameSubscriber.hpp"
//#include "../slam/slam-wrapper-node.hpp"
//#include "./System.hpp"

#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"
#include "Converter.h"
#include "KeyFrameDatabase.h"
#include "GeometricCamera.h"

class SlamWrapperNode;

//namespace TempDistributor
//{

class KeyFramePublisher;
class KeyFrameSubscriber;
class MapHandler;
class System;

class Observer : public ORB_SLAM3::Distributor
{
  public:  
    Observer(MapHandler* pMapHandler, KeyFramePublisher* pKeyFramePublisher, KeyFrameSubscriber* pKeyFrameSubscriber);
    ~Observer();

    // Routing data
    void ForwardKeyFrameToTarget(ORB_SLAM3::KeyFrame* pKF, const unsigned int mnTargetModule, const unsigned int nFromModule);
    void InjectMap(ORB_SLAM3::Map* tempMap, ORB_SLAM3::Map* pCurrentMap);
    void InjectMapPoint(ORB_SLAM3::MapPoint* tempMP, std::map<std::string, ORB_SLAM3::MapPoint*> mMapMPs);
    ORB_SLAM3::KeyFrame* InjectKeyFrame(ORB_SLAM3::KeyFrame* tempKF, std::map<unsigned long int, ORB_SLAM3::KeyFrame*> mMapKFs);
    ORB_SLAM3::MapPoint* ConvertMapPoint(std::shared_ptr<orbslam3_interfaces::msg::MapPoint> mpRosMP, std::map<unsigned long int, ORB_SLAM3::Map*> mMaps);
    ORB_SLAM3::KeyFrame* ConvertKeyFrame(std::shared_ptr<orbslam3_interfaces::msg::KeyFrame> mpRosKF, std::map<unsigned long int, ORB_SLAM3::Map*> mMaps);

    
    void AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM);
    void AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node);
    void AttachKeyFramePublisher(KeyFramePublisher* pKeyFramePublisher);
    
    // Update worker set
    void AddNewWorker(unsigned int mnWorkerModule);

    // Task module related
    int GetTaskModule();
    
    // Override inherited public functions from ORBSLAM3
    void onActiveMapReset(unsigned long int mnMapId) override;
    void onLMResetRequested() override;
    void onChangeLMActive(bool bActive) override;
    void onKeyframeAdded(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule) override;
    void onLocalMapUpdated(ORB_SLAM3::Map* pM) override;
    void onGlobalMapUpdated(bool mbMerged, bool mbLoopClosure, std::vector<unsigned long int> mvMeergedIds) override;
    int KeyFramesInQueue() override;
    int MapsInQueue() override;

    
  protected:  
    int GetWorkerNumber();
    
    std::set<unsigned int> mspWorkers; 
    std::mutex mMutexWorker;

    std::mutex mMutexTask;
    int mnTaskModule;

  private:  
    std::shared_ptr<SlamWrapperNode> pSLAMNode;

    System* mpSystem;
    KeyFramePublisher* mpKeyFramePublisher;
    KeyFrameSubscriber* mpKeyFrameSubscriber;
    MapHandler* mpMapHandler;
    
    // ORB SLAM3 Data
    ORB_SLAM3::System* pSLAM;
    ORB_SLAM3::Tracking* mpTracker;
    ORB_SLAM3::LocalMapping* mpLocalMapper;
    ORB_SLAM3::LoopClosing* mpLoopCloser;
    ORB_SLAM3::Atlas* mpAtlas;    
    ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB; 
};
//}

#endif
