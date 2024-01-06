#ifndef __ACTION_CHECKER_HPP_
#define __ACTION_CHECKER_HPP_

#include "orbslam3_interfaces/Converter.hpp"
#include "orbslam3_interfaces/KeyFrameConverter.hpp"
#include "orbslam3_interfaces/MapConverter.hpp"
#include "orbslam3_interfaces/AtlasConverter.hpp"
#include "orbslam3_interfaces/ActionParser.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "utility.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"
#include "Converter.h"
#include "KeyFrameDatabase.h"

class ActionChecker 
{
  public:
    ActionChecker(ORB_SLAM3::Atlas* mpAtlas, ORB_SLAM3::KeyFrameDatabase* mpKfDb);

    ~ActionChecker();
   
    void Run();
 
    void InsertKeyFrame(ORB_SLAM3::KeyFrame* pKF);  
    void InsertUnprocessedKeyFrame(ORB_SLAM3::KeyFrame* opKF, orbslam3_interfaces::msg::KeyFrame::SharedPtr rpKF); 
    void InsertMapPoint(ORB_SLAM3::MapPoint* pMP);  
    void InsertUnprocessedMapPoint(ORB_SLAM3::MapPoint* opMP, orbslam3_interfaces::msg::MapPoint::SharedPtr rpMP); 
    void InsertMap(ORB_SLAM3::Map* pM);  

    void InsertKeyFrameAction(orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rpKFA); 
    void InsertAtlasAction(orbslam3_interfaces::msg::AtlasActions::SharedPtr rpAA);
    void InsertMapPointAction(orbslam3_interfaces::msg::MapPointActions::SharedPtr rpMPA);
    void InsertKFDBAction(orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr rpKFDBA);


  private: 
    bool mbFinish;
    bool mbStop;

    bool CheckNewKeyFrames();
    bool CheckNewMapPoints();
    bool CheckNewROSActions();
  
    std::mutex mMutexNewKFs;
    std::mutex mMutexNewMPs;
    std::mutex mMutexNewMs;
    std::mutex mMutexNewKFAs;
    std::mutex mMutexNewMPAs;
    std::mutex mMutexNewAAs;
    std::mutex mMutexNewKFDBAs;
  

  private:
    ORB_SLAM3::Atlas* mpAtlas_;    
    ORB_SLAM3::KeyFrameDatabase* mpKfDb_; 
    void checkForNewActions();
    // Action Executors
    bool PerformKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rKF);
    bool PerformAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rAA);
    bool PerformKFDBAction(const orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr rKfdb);
    bool PerformMapPointAction(const orbslam3_interfaces::msg::MapPointActions::SharedPtr rMpA);

    // Store SLAM data
    std::map<long unsigned int, std::tuple<ORB_SLAM3::Map*, orbslam3_interfaces::msg::Map::SharedPtr>> mpUnprocOrbMaps;
    std::map<long unsigned int, std::tuple<ORB_SLAM3::KeyFrame*, orbslam3_interfaces::msg::KeyFrame::SharedPtr>> mpUnprocOrbKeyFrames;
    std::map<long unsigned int, std::tuple<ORB_SLAM3::MapPoint*, orbslam3_interfaces::msg::MapPoint::SharedPtr>> mpUnprocOrbMapPoints;
    
    std::map<long unsigned int, ORB_SLAM3::Map*> mpOrbMaps;
    std::map<long unsigned int, ORB_SLAM3::KeyFrame*> mpOrbKeyFrames;
    std::map<long unsigned int, ORB_SLAM3::MapPoint*> mpOrbMapPoints;

    std::vector<unsigned long int> mvNewKFIds;
    
    // Store ROS Action messages for later prosessing
    std::vector<int> mRosActions; // First is the action idx 0=Atlas, 1=KeyFrame, ..., and second is the idx in the corresponding vector
    std::vector<orbslam3_interfaces::msg::KeyFrameActions::SharedPtr> mvpKfRosActions; 
    std::vector<orbslam3_interfaces::msg::AtlasActions::SharedPtr> mvpAtlasRosActions; 
    std::vector<orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr> mvpKFDBRosActions; 
    std::vector<orbslam3_interfaces::msg::MapPointActions::SharedPtr> mvpMpRosActions; 

    //std::map<long unsigned int, orbslam3_interfaces::msg::KeyFrame*> mpRosKeyFrames;
};

#endif
