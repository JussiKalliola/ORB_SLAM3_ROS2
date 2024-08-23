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
    void ForwardKeyFrameToTarget(ORB_SLAM3::KeyFrame* pKF, const unsigned int nFromModule, const bool mbNew);

    // Injetion functions. Inject received data back into SLAM
    void InjectMap(ORB_SLAM3::Map* tempMap, ORB_SLAM3::Map* pCurrentMap, const int nFromModule);
    void InjectMapPoint(ORB_SLAM3::MapPoint* tempMP, ORB_SLAM3::MapPoint* mpExistingMP, const bool mbNew);
    ORB_SLAM3::KeyFrame* InjectKeyFrame(ORB_SLAM3::KeyFrame* tempKF, ORB_SLAM3::KeyFrame* mpExistingKF, const int nFromModule);

    // Conversion between ORB and ROS2 interfaces
    ORB_SLAM3::MapPoint* ConvertMapPoint(const std::shared_ptr<orbslam3_interfaces::msg::MapPointUpdate>& mpRosMP, ORB_SLAM3::MapPoint* mpExistingMP);
    ORB_SLAM3::MapPoint* ConvertMapPoint(const std::shared_ptr<orbslam3_interfaces::msg::MapPoint>& mpRosMP, ORB_SLAM3::MapPoint* mpExistingMP);
    ORB_SLAM3::KeyFrame* ConvertKeyFrame(const std::shared_ptr<orbslam3_interfaces::msg::KeyFrame>& mpRosKF, ORB_SLAM3::KeyFrame* mpExistingKF);
    void ConvertKeyFrame(const std::shared_ptr<orbslam3_interfaces::msg::KeyFrameUpdate>& mpRosKF, ORB_SLAM3::KeyFrame* mpExistingKF);

    
    // ORB SLAM3 functions
    void AddMapPoint(ORB_SLAM3::MapPoint* pMP);
    void EraseMapPoint(ORB_SLAM3::MapPoint* pMP);
    void EraseMapPoint(std::string mnId);
    bool CheckIfMapPointExists(ORB_SLAM3::MapPoint* pMP);
    bool CheckIfMapPointExists(std::string mstrId);
    bool HasMapPointBeenErased(std::string mnId);
    ORB_SLAM3::MapPoint* GetMapPoint(std::string mnId);
    void ClearMapPointsFromMap(ORB_SLAM3::Map* pM);
    std::map<std::string, ORB_SLAM3::MapPoint*>& GetAllMapPoints();
    std::set<ORB_SLAM3::MapPoint*>& GetAllMapPointsSet();

    void AddKeyFrame(ORB_SLAM3::KeyFrame* pKF);
    void EraseKeyFrame(ORB_SLAM3::KeyFrame* pKF);
    void EraseKeyFrame(unsigned long int mnId);
    ORB_SLAM3::KeyFrame* GetKeyFrame(unsigned long int mnId);
    bool CheckIfKeyFrameExists(ORB_SLAM3::KeyFrame* pKF);
    bool CheckIfKeyFrameExists(unsigned long int mnId);
    bool HasKeyFrameBeenErased(unsigned long int mnId);
    void ClearKeyFramesFromMap(ORB_SLAM3::Map* pM);
    std::map<unsigned long int, ORB_SLAM3::KeyFrame*>& GetAllKeyFrames();
    std::set<ORB_SLAM3::KeyFrame*>& GetAllKeyFramesSet();

    void AddMap(ORB_SLAM3::Map* pM);
    void EraseMap(ORB_SLAM3::Map* pM);
    void EraseMap(unsigned long int mnId);
    std::map<unsigned long int, ORB_SLAM3::Map*>& GetAllMaps();

    
    // Update worker set
    void AddNewWorker(unsigned int mnWorkerModule);

    // Task module related
    int GetTaskModule();

    double TimeSinceReset();
    std::chrono::system_clock::time_point GetLastResetTime();
    void UpdateLastResetTime();


    void AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM);
    void AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node);
    void AttachKeyFramePublisher(KeyFramePublisher* pKeyFramePublisher);

    ORB_SLAM3::KeyFrame* mpLastKeyFrame;
    unsigned long int mnLastKFId;

    std::set<unsigned long int> msAllErasedKFIds;
    std::set<std::string> msAllErasedMPIds;

    unsigned long int GetMaxMPId();
    void UpdateMaxMPId(unsigned long int mnId);
    
    // Override inherited public functions from ORBSLAM3
    void onNewMap(ORB_SLAM3::Map* pM) override;
    void onNewKeyFrame(ORB_SLAM3::KeyFrame* pKF) override;
    void onNewMapPoint(ORB_SLAM3::MapPoint* pMP) override;
    void onActiveMapReset(unsigned long int mnMapId) override;
    void onLMResetRequested() override;
    void onLMStopRequest(const bool bStopLM) override;
    //void onChangeLMActive(bool bActive) override;
    void onKeyframeAdded(ORB_SLAM3::KeyFrame* pKF) override;
    void onKeyframeAdded(ORB_SLAM3::KeyFrame* pKF, std::set<std::string> msNewMapPointIds) override;
    void onLocalMapUpdated(ORB_SLAM3::Map* pM) override;
    void onGlobalMapUpdated(bool mbMerged, bool mbLoopClosure, std::vector<unsigned long int> mvMeergedIds) override;
    int KeyFramesInQueue() override;
    int MapsInQueue() override;



    
  protected:  
    int GetWorkerNumber();
    bool CheckIfWorkerExists(const unsigned int mnTargetID);
    
    std::set<unsigned int> mspWorkers; 
    std::mutex mMutexWorker;

    std::mutex mMutexTask;
    int mnTaskModule;


    // ORB SLAM3 Data
    std::map<unsigned long int, ORB_SLAM3::Map*> mOrbMaps;
    std::mutex mMutexMap;

    unsigned long int mnMaxMPId;
    std::map<std::string, ORB_SLAM3::MapPoint*> mOrbMapPoints;
    std::set<ORB_SLAM3::MapPoint*> msOrbMapPoints;
    std::mutex mMutexMapPoint;

    std::map<unsigned long int, ORB_SLAM3::KeyFrame*> mOrbKeyFrames;
    std::set<ORB_SLAM3::KeyFrame*> msOrbKeyFrames;
    std::mutex mMutexKeyFrame;


    double mdSinceReset;
    std::chrono::system_clock::time_point mtLastResetTime;
    std::mutex mMutexTimes;

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
