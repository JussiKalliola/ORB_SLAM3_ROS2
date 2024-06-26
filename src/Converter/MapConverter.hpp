#ifndef MAP_CONVERTER_HPP_
#define MAP_CONVERTER_HPP

#include "Converter.hpp"


namespace Converter {

  class MapConverter {
    using map_point = orbslam3_interfaces::msg::MapPoint;
    using map = orbslam3_interfaces::msg::Map;

    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;
    using orb_keyframe = ORB_SLAM3::KeyFrame;

    public:
      static map OrbMapToRosMap(orb_map* opM) {
        std::mutex mMutexNewMP;
        std::lock_guard<std::mutex> lock(mMutexNewMP);
        
        map rM;
        rM.system_id = std::getenv("SLAM_SYSTEM_ID");
        //KeyFrame[] mvp_keyframe_origins           // vector<KeyFrame*> mvpKeyFrameOrigins;
        rM.mv_backup_keyframe_origins_id = opM->mvBackupKeyFrameOriginsId;    // vector<unsigned long int> mvBackupKeyFrameOriginsId;
        if(opM->mpFirstRegionKF != nullptr) rM.mp_first_region_kf_id = opM->mpFirstRegionKF->mnId;               // KeyFrame* mpFirstRegionKF;
                                                  // std::mutex mMutexMapUpdate;
        rM.mb_fail = opM->mbFail;                              // bool mbFail;

        // Size of the thumbnail (always in power of 2)
        rM.thumb_width = opM->THUMB_WIDTH;                         // static const int THUMB_WIDTH = 512;
        rM.thumb_height = opM->THUMB_HEIGHT;                        // static const int THUMB_HEIGHT = 512;

        rM.n_next_id = opM->nNextId;                          // static long unsigned int nNextId;

        // DEBUG: show KFs which are used in LBA
        rM.ms_opt_kfs = opM->GetOptKFs();                       // std::set<long unsigned int> msOptKFs;
        rM.ms_fixed_kfs = opM->GetFixedKFs();                     // std::set<long unsigned int> msFixedKFs;

        // protected:

        rM.mn_id = opM->GetId();                              // long unsigned int mnId;
        

        //std::cout << " #################### num of map points "<< opM->GetAllMapPoints().size() << std::endl;
        std::set<std::string> mspUpdatedMapPointIds = opM->GetUpdatedMPIds();
        std::vector<std::string> mvpUpdatedMPIds(mspUpdatedMapPointIds.begin(), mspUpdatedMapPointIds.end()); 
        rM.mvp_updated_map_points_ids = mvpUpdatedMPIds;
        
        std::set<unsigned long int> mspUpdatedKFIds = opM->GetUpdatedKFIds();
        std::vector<unsigned long int> mvpUpdatedKFIds(mspUpdatedKFIds.begin(), mspUpdatedKFIds.end()); 
        rM.mvp_updated_keyframes_ids = mvpUpdatedKFIds;
        
        std::set<unsigned long int> mspErasedKFIds = opM->GetErasedKFIds();
        std::vector<unsigned long int> mvpErasedKFIds(mspErasedKFIds.begin(), mspErasedKFIds.end()); 

        std::set<std::string> mspErasedMPIds = opM->GetErasedMPIds();
        std::vector<std::string> mvpErasedMPIds(mspErasedMPIds.begin(), mspErasedMPIds.end()); 
        
        std::cout << " ************ (MAP) mspUpdateMapPointIds.size()=" << mspUpdatedMapPointIds.size() << std::endl;
        //std::cout << "before all keyframes" << std::endl;
        if(opM->GetAllKeyFrames().size() > 0) {
          std::vector<ORB_SLAM3::KeyFrame*> mvpAllKeyFrames = opM->GetAllKeyFrames();
          for(ORB_SLAM3::KeyFrame* kf : mvpAllKeyFrames)
          {
            if(kf && !mspErasedKFIds.count(kf->mnId))
            {
              if(!mspUpdatedKFIds.empty())
              {
                if(mspUpdatedKFIds.find(kf->mnId) != mspUpdatedKFIds.end())
                {
                  rM.msp_keyframes.push_back(Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROSKeyFrameUpdate(kf, mspUpdatedMapPointIds, false));
                }

              } else {
                rM.msp_keyframes.push_back(Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROSKeyFrameUpdate(kf, mspUpdatedMapPointIds, false));
              }
            }
            kf->mnNextTarget=0;
          }
        }

        //for(ORB_SLAM3::MapPoint* mp : opM->GetAllMapPoints())
        //{
        //  if(mp && !mspErasedMPIds.count(mp->mstrHexId))
        //  {
        //    if(!mspUpdatedMapPointIds.empty())
        //    {
        //      if(mspUpdatedMapPointIds.find(mp->mstrHexId) != mspUpdatedMapPointIds.end())
        //      {
        //        rM.msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        //        mspUpdatedMapPointIds.erase(mp->mstrHexId);
        //      }
        //    } else {
        //      rM.msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        //      //mspUpdatedMapPointIds.erase(mp->mstrHexId);
        //    }
        //  }
        //}
        

        //opM->ClearUpdatedKFIds();
        //MapPoint[] msp_map_points                 // std::set<MapPoint*> mspMapPoints;
        //KeyFrame[] msp_keyframes                  // std::set<KeyFrame*> mspKeyFrames;

        //std::cout << "before backups" << std::endl;
        // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        rM.mvp_backup_map_points_ids = opM->GetBackupMapPointsId();          // std::vector<MapPoint*> mvpBackupMapPoints;
        rM.mvp_backup_keyframes_ids = opM->GetBackupKeyFrames();           // std::vector<KeyFrame*> mvpBackupKeyFrames;
        
        rM.mvp_erased_keyframe_ids = mvpErasedKFIds;


        rM.mvp_erased_mappoint_ids = mvpErasedMPIds;

        


        //KeyFrame mp_kf_initial                    // KeyFrame* mpKFinitial;
        //KeyFrame mp_kf_lower_id                   // KeyFrame* mpKFlowerID;
        rM.mn_backup_kf_initial_id = opM->GetInitKFid();           // unsigned long int mnBackupKFinitialID;
        rM.mn_backup_kf_lower_id = opM->GetBackupKFLowerID();               // unsigned long int mnBackupKFlowerID;

        rM.mvp_reference_map_points_id = opM->GetBackupReferenceMapPointsId();      // std::vector<MapPoint*> mvpReferenceMapPoints;

        rM.mb_imu_initialized = opM->GetImuInitialized();                   // bool mbImuInitialized;

        rM.mn_map_change  = opM->GetMapChange();                      // int mnMapChange;
        rM.mn_map_change_notified = opM->GetMapChangeNotified();               // int mnMapChangeNotified;
         
        rM.mn_init_kf_id = opM->GetInitKFid();                    // long unsigned int mnInitKFid;
        rM.mn_max_kf_id = opM->GetMaxKFid();                      // long unsigned int mnMaxKFid;
        // long unsigned int mnLastLoopKFid;

        // Index related to a big change in the map (loop closure, global BA)
        rM.mn_big_change_idx = opM->GetBigChangeIdx();                   // int mnBigChangeIdx;



        //std::cout << "before bools" << std::endl;
        rM.m_is_in_use = opM->IsInUse();                         // bool mIsInUse;
        rM.m_has_thumbnail = opM->HasThumbnail();                      // bool mHasTumbnail;
        rM.m_bad = opM->GetIsBad();                                // bool mbBad = false;

        rM.mb_is_inertial = opM->IsInertial();                       // bool mbIsInertial;
        rM.mb_imu_ba1 = opM->GetIniertialBA1();                           // bool mbIMU_BA1;
        rM.mb_imu_ba2 = opM->GetIniertialBA2();                           // bool mbIMU_BA2;


        //std::cout << "Map." << opM->GetId() << " Before ROS Broadcast - #MP=" << opM->GetAllMapPoints().size() <<", #KF=" << opM->GetAllKeyFrames().size() << ", #RefMPs" << opM->GetReferenceMapPoints().size() << ", initKF=" << opM->GetInitKFid() << ", MaxKF=" << opM->GetMaxKFid() << ", OriginKFid" << opM->GetOriginKF()->mnId << std::endl;


        return rM;
      }


      static map OrbMapToRosMap(orb_map* opM, std::set<unsigned long int>& msUpdatedKFs, std::set<std::string>& msUpdatedMPs, std::set<unsigned long int>& msErasedKFs, std::set<std::string>& msErasedMPs) {
        std::mutex mMutexNewMP;
        std::lock_guard<std::mutex> lock(mMutexNewMP);
        
        map rM;
        rM.system_id = std::getenv("SLAM_SYSTEM_ID");
        //KeyFrame[] mvp_keyframe_origins           // vector<KeyFrame*> mvpKeyFrameOrigins;
        rM.mv_backup_keyframe_origins_id = opM->mvBackupKeyFrameOriginsId;    // vector<unsigned long int> mvBackupKeyFrameOriginsId;
        if(opM->mpFirstRegionKF != nullptr) rM.mp_first_region_kf_id = opM->mpFirstRegionKF->mnId;               // KeyFrame* mpFirstRegionKF;
                                                  // std::mutex mMutexMapUpdate;
        rM.mb_fail = opM->mbFail;                              // bool mbFail;

        // Size of the thumbnail (always in power of 2)
        rM.thumb_width = opM->THUMB_WIDTH;                         // static const int THUMB_WIDTH = 512;
        rM.thumb_height = opM->THUMB_HEIGHT;                        // static const int THUMB_HEIGHT = 512;

        rM.n_next_id = opM->nNextId;                          // static long unsigned int nNextId;

        // DEBUG: show KFs which are used in LBA
        rM.ms_opt_kfs = opM->GetOptKFs();                       // std::set<long unsigned int> msOptKFs;
        rM.ms_fixed_kfs = opM->GetFixedKFs();                     // std::set<long unsigned int> msFixedKFs;

        // protected:

        rM.mn_id = opM->GetId();                              // long unsigned int mnId;
        

        //std::cout << " #################### num of map points "<< opM->GetAllMapPoints().size() << std::endl;
        std::set<std::string> mspUpdatedMapPointIds = msUpdatedMPs;
        std::vector<std::string> mvpUpdatedMPIds(mspUpdatedMapPointIds.begin(), mspUpdatedMapPointIds.end()); 
        rM.mvp_updated_map_points_ids = mvpUpdatedMPIds;
        
        std::set<unsigned long int> mspUpdatedKFIds = msUpdatedKFs;
        std::vector<unsigned long int> mvpUpdatedKFIds(mspUpdatedKFIds.begin(), mspUpdatedKFIds.end()); 
        rM.mvp_updated_keyframes_ids = mvpUpdatedKFIds;
        

        std::cout << " ************ (MAP) mspUpdateMapPointIds.size()=" << mspUpdatedMapPointIds.size() << std::endl;
        std::set<std::string> mspErasedMPIds = opM->GetErasedMPIds();
        std::vector<std::string> mvpErasedMPIds(msErasedMPs.begin(), msErasedMPs.end()); 

        std::set<unsigned long int> mspErasedKFIds = opM->GetErasedKFIds();
        std::vector<unsigned long int> mvpErasedKFIds(msErasedKFs.begin(), msErasedKFs.end()); 

        //std::cout << "before all keyframes" << std::endl;
        if(opM->GetAllKeyFrames().size() > 0 && !mspUpdatedKFIds.empty()) {
          std::vector<ORB_SLAM3::KeyFrame*> mvpAllKeyFrames = opM->GetAllKeyFrames();
          for(ORB_SLAM3::KeyFrame* kf : mvpAllKeyFrames)
          {
            if(kf && !mspErasedKFIds.count(kf->mnId))
            {
              if(mspUpdatedKFIds.find(kf->mnId) != mspUpdatedKFIds.end())
              {
                rM.msp_keyframes.push_back(Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROSKeyFrameUpdate(kf, mspUpdatedMapPointIds, false));
              }

              //else {
              //  rM.msp_keyframes.push_back(Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROSKeyFrameUpdate(kf, mspUpdatedMapPointIds, false));
              //}
            }
            kf->mnNextTarget=0;
          }
        }

        //if(!mspUpdatedMapPointIds.empty())
        //{
        //  for(ORB_SLAM3::MapPoint* mp : opM->GetAllMapPoints())
        //  {
        //    if(mp && !mspErasedMPIds.count(mp->mstrHexId))
        //    {
        //        if(mspUpdatedMapPointIds.find(mp->mstrHexId) != mspUpdatedMapPointIds.end())
        //        {
        //          rM.msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        //          mspUpdatedMapPointIds.erase(mp->mstrHexId);
        //        }
        //        //} else {
        //        //  rM.msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        //        //  //mspUpdatedMapPointIds.erase(mp->mstrHexId);
        //        //}
        //    }
        //  }
        //}
        

        //opM->ClearUpdatedKFIds();
        //MapPoint[] msp_map_points                 // std::set<MapPoint*> mspMapPoints;
        //KeyFrame[] msp_keyframes                  // std::set<KeyFrame*> mspKeyFrames;

        //std::cout << "before backups" << std::endl;
        // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        rM.mvp_backup_map_points_ids = opM->GetBackupMapPointsId();          // std::vector<MapPoint*> mvpBackupMapPoints;
        rM.mvp_backup_keyframes_ids = opM->GetBackupKeyFrames();           // std::vector<KeyFrame*> mvpBackupKeyFrames;
        
        rM.mvp_erased_keyframe_ids = mvpErasedKFIds;


        //std::set<std::string> mspErasedMPIds = opM->GetErasedMPIds();
        rM.mvp_erased_mappoint_ids = mvpErasedMPIds;

        


        //KeyFrame mp_kf_initial                    // KeyFrame* mpKFinitial;
        //KeyFrame mp_kf_lower_id                   // KeyFrame* mpKFlowerID;
        rM.mn_backup_kf_initial_id = opM->GetInitKFid();           // unsigned long int mnBackupKFinitialID;
        rM.mn_backup_kf_lower_id = opM->GetBackupKFLowerID();               // unsigned long int mnBackupKFlowerID;

        rM.mvp_reference_map_points_id = opM->GetBackupReferenceMapPointsId();      // std::vector<MapPoint*> mvpReferenceMapPoints;

        rM.mb_imu_initialized = opM->GetImuInitialized();                   // bool mbImuInitialized;

        rM.mn_map_change  = opM->GetMapChange();                      // int mnMapChange;
        rM.mn_map_change_notified = opM->GetMapChangeNotified();               // int mnMapChangeNotified;
         
        rM.mn_init_kf_id = opM->GetInitKFid();                    // long unsigned int mnInitKFid;
        rM.mn_max_kf_id = opM->GetMaxKFid();                      // long unsigned int mnMaxKFid;
        // long unsigned int mnLastLoopKFid;

        // Index related to a big change in the map (loop closure, global BA)
        rM.mn_big_change_idx = opM->GetBigChangeIdx();                   // int mnBigChangeIdx;



        //std::cout << "before bools" << std::endl;
        rM.m_is_in_use = opM->IsInUse();                         // bool mIsInUse;
        rM.m_has_thumbnail = opM->HasThumbnail();                      // bool mHasTumbnail;
        rM.m_bad = opM->GetIsBad();                                // bool mbBad = false;

        rM.mb_is_inertial = opM->IsInertial();                       // bool mbIsInertial;
        rM.mb_imu_ba1 = opM->GetIniertialBA1();                           // bool mbIMU_BA1;
        rM.mb_imu_ba2 = opM->GetIniertialBA2();                           // bool mbIMU_BA2;


        //std::cout << "Map." << opM->GetId() << " Before ROS Broadcast - #MP=" << opM->GetAllMapPoints().size() <<", #KF=" << opM->GetAllKeyFrames().size() << ", #RefMPs" << opM->GetReferenceMapPoints().size() << ", initKF=" << opM->GetInitKFid() << ", MaxKF=" << opM->GetMaxKFid() << ", OriginKFid" << opM->GetOriginKF()->mnId << std::endl;


        return rM;
      }
      
      static orb_map* RosMapToOrbMap(const map::SharedPtr& rM, orb_map* mpPrevMap) {        
        

        bool mbFail = rM->mb_fail;
        std::vector<unsigned long int> mvOptKFs = rM->ms_opt_kfs;
        std::set<unsigned long int> msOptKFs(mvOptKFs.begin(), mvOptKFs.end());
        std::vector<unsigned long int> mvFixedKFs = rM->ms_fixed_kfs;
        std::set<unsigned long int> msFixedKFs(mvFixedKFs.begin(), mvFixedKFs.end());
        long unsigned int mnId = rM->mn_id;
        std::vector<std::string> mvpBackupMapPointsId = rM->mvp_backup_map_points_ids;
        std::vector<unsigned long int> mvpBackupKeyFramesId = rM->mvp_backup_keyframes_ids;
        
        std::vector<std::string> mvUpdatedMPIds = rM->mvp_updated_map_points_ids;
        std::set<std::string> msUpdatedMPIds(mvUpdatedMPIds.begin(), mvUpdatedMPIds.end());

        std::vector<unsigned long int> mvUpdatedKFIds = rM->mvp_updated_keyframes_ids;
        std::set<unsigned long int> msUpdatedKFIds(mvUpdatedKFIds.begin(), mvUpdatedKFIds.end());

        vector<unsigned long int> mvBackupKeyFrameOriginsId = rM->mv_backup_keyframe_origins_id;     // vector<unsigned long int> mvBackupKeyFrameOriginsId;
        unsigned long int mnBackupKFinitialID = rM->mn_backup_kf_initial_id;
        unsigned long int mnBackupKFlowerID = rM->mn_backup_kf_lower_id;
        std::vector<std::string> mvpBackupReferenceMapPointsId = rM->mvp_reference_map_points_id;
        bool mbImuInitialized = rM->mb_imu_initialized;
        int mnMapChange = rM->mn_map_change;
        int mnMapChangeNotified = rM->mn_map_change_notified;
        long unsigned int mnInitKFid = rM->mn_init_kf_id;
        long unsigned int mnMaxKFid = rM->mn_max_kf_id;
        int mnBigChangeIdx = rM->mn_big_change_idx;
        bool mIsInUse = rM->m_is_in_use;
        bool mHasTumbnail = rM->m_has_thumbnail;
        bool mbBad = rM->m_bad;
        bool mbIsInertial = rM->mb_is_inertial;
        bool mbIMU_BA1 = rM->mb_imu_ba1;
        bool mbIMU_BA2 = rM->mb_imu_ba2;

        std::vector<unsigned long int> mvErasedKFIds = rM->mvp_erased_keyframe_ids;
        std::set<unsigned long int> msErasedKFIds(mvErasedKFIds.begin(), mvErasedKFIds.end());

        std::vector<std::string> mvErasedMPIds = rM->mvp_erased_mappoint_ids;
        std::set<std::string> msErasedMPIds(mvErasedMPIds.begin(), mvErasedMPIds.end());

        if(mpPrevMap)
        {
          std::cout << "updating map..." << std::endl;
          mpPrevMap->UpdateMap(mbFail, msOptKFs, msFixedKFs, mnId, mvpBackupMapPointsId, mvpBackupKeyFramesId, msUpdatedKFIds, msUpdatedMPIds, mvBackupKeyFrameOriginsId, mnBackupKFinitialID, mnBackupKFlowerID, mvpBackupReferenceMapPointsId, mbImuInitialized, mnMapChange, mnMapChangeNotified, mnInitKFid, mnMaxKFid, mnBigChangeIdx, mIsInUse, /*mHasTumbnail,*/ mbBad, msErasedKFIds, msErasedMPIds/*, mbIsInertial, mbIMU_BA1, mbIMU_BA2*/);
          std::cout << "done with map update." << std::endl;
          return mpPrevMap;
        } else {
          std::cout << "Creating a new map..." << std::endl;
          return new orb_map(mbFail, msOptKFs, msFixedKFs, mnId, mvpBackupMapPointsId, mvpBackupKeyFramesId, mvBackupKeyFrameOriginsId, mnBackupKFinitialID, mnBackupKFlowerID, mvpBackupReferenceMapPointsId, mbImuInitialized, mnMapChange, mnMapChangeNotified, mnInitKFid, mnMaxKFid, mnBigChangeIdx, mIsInUse, mHasTumbnail, mbBad, mbIsInertial, mbIMU_BA1, mbIMU_BA2, msErasedKFIds, msErasedMPIds);
        }

    } 

  };

};


#endif
