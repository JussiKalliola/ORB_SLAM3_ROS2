#include "Distributor.h"

#include "orbslam3_interfaces/Converter.hpp"
//#include "orbslam3_interfaces/KeyFrameConverter.hpp"
//#include "orbslam3_interfaces/MapConverter.hpp"
//#include "orbslam3_interfaces/AtlasConverter.hpp"

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

class Distributor : public ORB_SLAM3::Distributor {

    public:
        Distributor();
        ~Distributor();
        
        // Main function
        void Run();
        
        // Update worker set
        void AddNewWorker(unsigned int mnWorkerModule);

        // Override inherited public functions in ORBSLAM3
        void onActiveMapReset(unsigned long int mnMapId) override;
        void onLMResetRequested() override;
        void onChangeLMActive(bool bActive) override;
        void onKeyframeAdded(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule) override;
        void onLocalMapUpdated(ORB_SLAM3::Map* pM) override;
        void onGlobalMapUpdated(bool mbMerged, bool mbLoopClosure, std::vector<unsigned long int> mvMeergedIds) override;
        
        int KeyFramesInQueue() override;
        int MapsInQueue() override;
        
        void RequestFinish();
        bool isFinished();
        
        void AttachSLAMSystem(ORB_SLAM3::System* mSLAM);
        void AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node);
        void deleteSlamNode(std::shared_ptr<SlamWrapperNode> slam_node);

        int mnTaskModule;

    protected:
        bool CheckNewKeyFrames();
        bool CheckNewLocalMaps();
        bool CheckNewGlobalMaps();

        int GetWorkerNumber();
        bool CheckIfWorkerExists(unsigned int mnTargetModule);
        
        void InsertNewKeyFrame(ORB_SLAM3::KeyFrame* pKF, unsigned int mnTargetModule);
        void InsertNewLocalMap(ORB_SLAM3::Map* pM);
        void InsertNewGlobalMap(std::tuple<bool, bool, std::vector<unsigned long int>> mtAtlasUpdate);
        
        orbslam3_interfaces::msg::KeyFrame ProcessNewKeyFrame();
        void ProcessNewLocalMap();
        void ProcessNewGlobalMap();
        
        std::list<ORB_SLAM3::KeyFrame*> mlpKeyFrameQueue;
        std::list<unsigned int> mlKeyFrameTargetModule;
        std::mutex mMutexNewKFs;

        std::list<ORB_SLAM3::Map*> mlpMapQueue;
        std::mutex mMutexNewMaps;
        
        std::list<std::tuple<bool, bool, std::vector<unsigned long int>>> mlpAtlasQueue;
        std::mutex mMutexNewAtlas;

        std::set<unsigned int> mspWorkers; 
        std::mutex mMutexWorker;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;
        
        bool IsMapUpdateRunning()
        {
          std::mutex mMutexMapUpdate;
          return mbMapUpdateRunning;
        }

        void StopMapThread()
        {
            std::mutex mMutexMapUpdate;
            mpThreadMapUpdate->detach();
        }

        bool mbMapUpdateRunning;
        std::mutex mMutexMapUpdate;
        std::thread* mpThreadMapUpdate;

    private:
        std::shared_ptr<SlamWrapperNode> slam_node_;

        // ORB SLAM3 Data
        ORB_SLAM3::System* pSLAM;
        ORB_SLAM3::Tracking* mpTracker;
        ORB_SLAM3::LocalMapping* mpLocalMapper;
        ORB_SLAM3::LoopClosing* mpLoopCloser;
        ORB_SLAM3::Atlas* mpAtlas;    
        ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB; 
};

