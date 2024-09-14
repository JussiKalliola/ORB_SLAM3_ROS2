#ifndef MAP_CONVERTER_HPP_
#define MAP_CONVERTER_HPP

#include "Converter.hpp"


namespace Converter {

  class MapConverter {
    using map_point = orbslam3_interfaces::msg::MapPoint;

    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;
    using orb_keyframe = ORB_SLAM3::KeyFrame;

    public:
      static orbslam3_interfaces::msg::Map::SharedPtr OrbMapToRosMap(orb_map* opM) {
        std::mutex mMutexNewMP;
        std::lock_guard<std::mutex> lock(mMutexNewMP);
        
        orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = std::make_shared<orbslam3_interfaces::msg::Map>();
        mpRosMap->system_id = std::getenv("SLAM_SYSTEM_ID");
        //KeyFrame[] mvp_keyframe_origins           // vector<KeyFrame*> mvpKeyFrameOrigins;
        mpRosMap->mv_backup_keyframe_origins_id = opM->mvBackupKeyFrameOriginsId;    // vector<unsigned long int> mvBackupKeyFrameOriginsId;
        if(opM->mpFirstRegionKF != nullptr) 
            mpRosMap->mp_first_region_kf_id = opM->mpFirstRegionKF->mnId;               // KeyFrame* mpFirstRegionKF;
                                                  // std::mutex mMutexMapUpdate;
        mpRosMap->mb_fail = opM->mbFail;                              // bool mbFail;

        // Size of the thumbnail (always in power of 2)
        mpRosMap->thumb_width = opM->THUMB_WIDTH;                         // static const int THUMB_WIDTH = 512;
        mpRosMap->thumb_height = opM->THUMB_HEIGHT;                        // static const int THUMB_HEIGHT = 512;

        mpRosMap->n_next_id = opM->nNextId;                          // static long unsigned int nNextId;

        // DEBUG: show KFs which are used in LBA
        //mpRosMap->ms_opt_kfs = opM->GetOptKFs();                       // std::set<long unsigned int> msOptKFs;
        //mpRosMap->ms_fixed_kfs = opM->GetFixedKFs();                     // std::set<long unsigned int> msFixedKFs;

        // protected:

        mpRosMap->mn_id = opM->GetId();                              // long unsigned int mnId;
        

        //std::cout << " #################### num of map points "<< opM->GetAllMapPoints().size() << std::endl;
        std::set<std::string> mspUpdatedMapPointIds = opM->GetUpdatedMPIds();
        std::vector<std::string> mvpUpdatedMPIds(mspUpdatedMapPointIds.size()); 
        for(std::set<std::string>::iterator it = mspUpdatedMapPointIds.begin(); it != mspUpdatedMapPointIds.end(); ++it)
        {
          mvpUpdatedMPIds[std::distance(mspUpdatedMapPointIds.begin(),it)] = *it;
        }
        mpRosMap->mvp_updated_map_points_ids = mvpUpdatedMPIds;
        
        std::set<unsigned long int> mspUpdatedKFIds = opM->GetUpdatedKFIds();
        std::vector<unsigned long int> mvpUpdatedKFIds(mspUpdatedKFIds.size()); 
        for(std::set<unsigned long int>::iterator it = mspUpdatedKFIds.begin(); it != mspUpdatedKFIds.end(); ++it)
        {
          mvpUpdatedKFIds[std::distance(mspUpdatedKFIds.begin(),it)] = *it;
        }
        mpRosMap->mvp_updated_keyframes_ids = mvpUpdatedKFIds;
        
        std::set<unsigned long int> mspErasedKFIds = opM->GetErasedKFIds();
        std::vector<unsigned long int> mvpErasedKFIds(mspErasedKFIds.size()); 
        for(std::set<unsigned long int>::iterator it = mspErasedKFIds.begin(); it != mspErasedKFIds.end(); ++it)
        {
          mvpErasedKFIds[std::distance(mspErasedKFIds.begin(),it)] = *it;
        }

        std::set<std::string> mspErasedMPIds = opM->GetErasedMPIds();
        std::vector<std::string> mvpErasedMPIds(mspErasedMPIds.size()); 
        for(std::set<std::string>::iterator it = mspErasedMPIds.begin(); it != mspErasedMPIds.end(); ++it)
        {
          mvpErasedMPIds[std::distance(mspErasedMPIds.begin(),it)] = *it;
        }
        
        std::cout << " ************ (MAP) mspUpdateMapPointIds.size()=" << mspUpdatedMapPointIds.size() << std::endl;
        std::vector<orbslam3_interfaces::msg::KeyFrameUpdate> mvRosKFUpdates;
        mvRosKFUpdates.reserve(opM->GetAllKeyFrames().size());
        //std::cout << "before all keyframes" << std::endl;
        if(opM->GetAllKeyFrames().size() > 0) {
          const std::vector<ORB_SLAM3::KeyFrame*>& mvpAllKeyFrames = opM->GetAllKeyFrames();
          for(ORB_SLAM3::KeyFrame* kf : mvpAllKeyFrames)
          {
            if(kf)
            {
              //if(kf->GetLastModule() == 4)
              //  continue;

              if(!mspUpdatedKFIds.empty())
              {
                if(mspUpdatedKFIds.find(kf->mnId) != mspUpdatedKFIds.end())
                {
                  mvRosKFUpdates.push_back(Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROSKeyFrameUpdate(kf, mspUpdatedMapPointIds, mspErasedMPIds, false));
                }

              } else {
                mvRosKFUpdates.push_back(Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROSKeyFrameUpdate(kf, mspUpdatedMapPointIds, mspErasedMPIds, false));
              }
            }
            kf->mnNextTarget=0;
          }
        }
        mpRosMap->msp_keyframes = mvRosKFUpdates;

        //for(ORB_SLAM3::MapPoint* mp : opM->GetAllMapPoints())
        //{
        //  if(mp && !mspErasedMPIds.count(mp->mstrHexId))
        //  {
        //    if(!mspUpdatedMapPointIds.empty())
        //    {
        //      if(mspUpdatedMapPointIds.find(mp->mstrHexId) != mspUpdatedMapPointIds.end())
        //      {
        //        mpRosMap->msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        //        mspUpdatedMapPointIds.erase(mp->mstrHexId);
        //      }
        //    } else {
        //      mpRosMap->msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        //      //mspUpdatedMapPointIds.erase(mp->mstrHexId);
        //    }
        //  }
        //}
        

        //opM->ClearUpdatedKFIds();
        //MapPoint[] msp_map_points                 // std::set<MapPoint*> mspMapPoints;
        //KeyFrame[] msp_keyframes                  // std::set<KeyFrame*> mspKeyFrames;

        //std::cout << "before backups" << std::endl;
        // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        mpRosMap->mvp_backup_map_points_ids = opM->GetBackupMapPointsId();          // std::vector<MapPoint*> mvpBackupMapPoints;
        mpRosMap->mvp_backup_keyframes_ids = opM->GetBackupKeyFrames();           // std::vector<KeyFrame*> mvpBackupKeyFrames;
        
        mpRosMap->mvp_erased_keyframe_ids = mvpErasedKFIds;


        mpRosMap->mvp_erased_mappoint_ids = mvpErasedMPIds;

        


        //KeyFrame mp_kf_initial                    // KeyFrame* mpKFinitial;
        //KeyFrame mp_kf_lower_id                   // KeyFrame* mpKFlowerID;
        mpRosMap->mn_backup_kf_initial_id = opM->GetInitKFid();           // unsigned long int mnBackupKFinitialID;
        mpRosMap->mn_backup_kf_lower_id = opM->GetBackupKFLowerID();               // unsigned long int mnBackupKFlowerID;

        mpRosMap->mvp_reference_map_points_id = opM->GetBackupReferenceMapPointsId();      // std::vector<MapPoint*> mvpReferenceMapPoints;

        mpRosMap->mb_imu_initialized = opM->GetImuInitialized();                   // bool mbImuInitialized;

        mpRosMap->mn_map_change  = opM->GetMapChange();                      // int mnMapChange;
        mpRosMap->mn_map_change_notified = opM->GetMapChangeNotified();               // int mnMapChangeNotified;
         
        mpRosMap->mn_init_kf_id = opM->GetInitKFid();                    // long unsigned int mnInitKFid;
        mpRosMap->mn_max_kf_id = opM->GetMaxKFid();                      // long unsigned int mnMaxKFid;
        // long unsigned int mnLastLoopKFid;

        // Index related to a big change in the map (loop closure, global BA)
        mpRosMap->mn_big_change_idx = opM->GetLastBigChangeIdx();                   // int mnBigChangeIdx;



        //std::cout << "before bools" << std::endl;
        mpRosMap->m_is_in_use = opM->IsInUse();                         // bool mIsInUse;
        mpRosMap->m_has_thumbnail = opM->HasThumbnail();                      // bool mHasTumbnail;
        mpRosMap->m_bad = opM->GetIsBad();                                // bool mbBad = false;

        mpRosMap->mb_is_inertial = opM->IsInertial();                       // bool mbIsInertial;
        mpRosMap->mb_imu_ba1 = opM->GetIniertialBA1();                           // bool mbIMU_BA1;
        mpRosMap->mb_imu_ba2 = opM->GetIniertialBA2();                           // bool mbIMU_BA2;


        //std::cout << "Map." << opM->GetId() << " Before ROS Broadcast - #MP=" << opM->GetAllMapPoints().size() <<", #KF=" << opM->GetAllKeyFrames().size() << ", #RefMPs" << opM->GetReferenceMapPoints().size() << ", initKF=" << opM->GetInitKFid() << ", MaxKF=" << opM->GetMaxKFid() << ", OriginKFid" << opM->GetOriginKF()->mnId << std::endl;


        return mpRosMap;
      }


      static orbslam3_interfaces::msg::Map::SharedPtr OrbMapToRosMap(orb_map* opM, std::set<unsigned long int>& msUpdatedKFs, std::set<std::string>& msUpdatedMPs, std::set<unsigned long int>& msErasedKFs, std::set<std::string>& msErasedMPs, std::map<unsigned long int, ORB_SLAM3::KeyFrame*>& mOrbKeyFrames, std::map<std::string, ORB_SLAM3::MapPoint*>& mOrbMapPoints) {
        std::mutex mMutexNewMP;
        std::lock_guard<std::mutex> lock(mMutexNewMP);
        
        orbslam3_interfaces::msg::Map::SharedPtr mpRosMap = std::make_shared<orbslam3_interfaces::msg::Map>();
        mpRosMap->system_id = std::getenv("SLAM_SYSTEM_ID");
        //KeyFrame[] mvp_keyframe_origins           // vector<KeyFrame*> mvpKeyFrameOrigins;
        mpRosMap->mv_backup_keyframe_origins_id = opM->mvBackupKeyFrameOriginsId;    // vector<unsigned long int> mvBackupKeyFrameOriginsId;
        if(opM->mpFirstRegionKF != nullptr) 
            mpRosMap->mp_first_region_kf_id = opM->mpFirstRegionKF->mnId;               // KeyFrame* mpFirstRegionKF;
                                                  // std::mutex mMutexMapUpdate;
        mpRosMap->mb_fail = opM->mbFail;                              // bool mbFail;

        // Size of the thumbnail (always in power of 2)
        mpRosMap->thumb_width = opM->THUMB_WIDTH;                         // static const int THUMB_WIDTH = 512;
        mpRosMap->thumb_height = opM->THUMB_HEIGHT;                        // static const int THUMB_HEIGHT = 512;

        mpRosMap->n_next_id = opM->nNextId;                          // static long unsigned int nNextId;

        // DEBUG: show KFs which are used in LBA
        //mpRosMap->ms_opt_kfs = opM->GetOptKFs();                       // std::set<long unsigned int> msOptKFs;
        //mpRosMap->ms_fixed_kfs = opM->GetFixedKFs();                     // std::set<long unsigned int> msFixedKFs;

        // protected:

        mpRosMap->mn_id = opM->GetId();                              // long unsigned int mnId;
        

        //std::cout << " #################### num of map points "<< opM->GetAllMapPoints().size() << std::endl;
        std::set<std::string> mspUpdatedMapPointIds;
        
        std::set<unsigned long int>& mspUpdatedKFIds = msUpdatedKFs;
        std::vector<unsigned long int> mvpUpdatedKFIds; 
        mvpUpdatedKFIds.reserve(mspUpdatedKFIds.size());
        for(std::set<unsigned long int>::iterator it = mspUpdatedKFIds.begin(); it != mspUpdatedKFIds.end(); ++it)
        {
          mvpUpdatedKFIds.push_back(*it);
        }
        mpRosMap->mvp_updated_keyframes_ids = mvpUpdatedKFIds;
        

        //std::set<std::string>& mspErasedMPIds = opM->GetErasedMPIds();
        std::vector<std::string> mvpErasedMPIds; 
        mvpErasedMPIds.reserve(msErasedMPs.size());
        for(std::set<std::string>::iterator it = msErasedMPs.begin(); it != msErasedMPs.end(); ++it)
        {
          mvpErasedMPIds.push_back(*it);
        }

        //std::set<unsigned long int> mspErasedKFIds = opM->GetErasedKFIds();
        std::vector<unsigned long int> mvpErasedKFIds(msErasedKFs.size()); 

        for(std::set<unsigned long int>::iterator it = msErasedKFs.begin(); it != msErasedKFs.end(); ++it)
        {
          mvpErasedKFIds[std::distance(msErasedKFs.begin(),it)] = *it;
        }

        std::vector<orbslam3_interfaces::msg::KeyFrameUpdate> mvRosKFUpdates;
        mvRosKFUpdates.reserve(opM->GetAllKeyFrames().size());

        //std::cout << "before all keyframes" << std::endl;
        if(opM->GetAllKeyFrames().size() > 0 && !msUpdatedKFs.empty()) {

            for(std::set<unsigned long int>::iterator it = mspUpdatedKFIds.begin(); it != mspUpdatedKFIds.end(); ++it)
            {
                  ORB_SLAM3::KeyFrame* pKFi = mOrbKeyFrames[*it];
                  if(pKFi)
                  {
                      const orbslam3_interfaces::msg::KeyFrameUpdate& msgKf = Converter::KeyFrameConverter::ORBSLAM3KeyFrameToROSKeyFrameUpdate(pKFi, msUpdatedMPs, msErasedMPs, false); // = FormDefaultKeyFrameMessage();
                                                                                                                                      mvRosKFUpdates.push_back(msgKf);
                      for(const auto& mpMsg : msgKf.mvp_map_points)
                      {
                          mspUpdatedMapPointIds.insert(mpMsg.m_str_hex_id); 
                          msUpdatedMPs.erase(mpMsg.m_str_hex_id);
                      }
                      std::cout << " ************ (MAP) mspUpdateMapPointIds.size()=" << msUpdatedMPs.size() << std::endl;
                      pKFi->mnNextTarget=0;
                  }

            }
        }

        if(msUpdatedKFs.empty() && !msUpdatedMPs.empty())
        {
            std::vector<orbslam3_interfaces::msg::MapPoint> mvRosMPs;
            mvRosMPs.reserve(500);
            long int mpIter = 0;
            while(mpIter < 500 )
            {
                if(msUpdatedMPs.empty())
                  break;
                std::string mnId = *msUpdatedMPs.begin();
                ORB_SLAM3::MapPoint* pMPi = mOrbMapPoints[mnId];
                if(pMPi)
                {
                    mvRosMPs.emplace_back(MapPointConverter::ORBSLAM3MapPointToROS(pMPi, 0));
                }
                msUpdatedMPs.erase(mnId);
                mpIter++;
            }

            mpRosMap->msp_map_points = mvRosMPs;

        }
        mpRosMap->msp_keyframes = mvRosKFUpdates;

        std::vector<std::string> mvpUpdatedMPIds(mspUpdatedMapPointIds.size()); 
        for(std::set<std::string>::iterator it = mspUpdatedMapPointIds.begin(); it != mspUpdatedMapPointIds.end(); ++it)
        {
          mvpUpdatedMPIds[std::distance(mspUpdatedMapPointIds.begin(),it)] = *it;
        }
        mpRosMap->mvp_updated_map_points_ids = mvpUpdatedMPIds;

        //if(!mspUpdatedMapPointIds.empty())
        //{
        //  for(ORB_SLAM3::MapPoint* mp : opM->GetAllMapPoints())
        //  {
        //    if(mp && !mspErasedMPIds.count(mp->mstrHexId))
        //    {
        //        if(mspUpdatedMapPointIds.find(mp->mstrHexId) != mspUpdatedMapPointIds.end())
        //        {
        //          mpRosMap->msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
        //          mspUpdatedMapPointIds.erase(mp->mstrHexId);
        //        }
        //        //} else {
        //        //  mpRosMap->msp_map_points.push_back(Converter::MapPointConverter::ORBSLAM3MapPointToROS(mp));
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
        mpRosMap->mvp_backup_map_points_ids = opM->GetBackupMapPointsId();          // std::vector<MapPoint*> mvpBackupMapPoints;
        mpRosMap->mvp_backup_keyframes_ids = opM->GetBackupKeyFrames();           // std::vector<KeyFrame*> mvpBackupKeyFrames;
        
        mpRosMap->mvp_erased_keyframe_ids = mvpErasedKFIds;


        //std::set<std::string> mspErasedMPIds = opM->GetErasedMPIds();
        mpRosMap->mvp_erased_mappoint_ids = mvpErasedMPIds;

        


        //KeyFrame mp_kf_initial                    // KeyFrame* mpKFinitial;
        //KeyFrame mp_kf_lower_id                   // KeyFrame* mpKFlowerID;
        mpRosMap->mn_backup_kf_initial_id = opM->GetInitKFid();           // unsigned long int mnBackupKFinitialID;
        mpRosMap->mn_backup_kf_lower_id = opM->GetBackupKFLowerID();               // unsigned long int mnBackupKFlowerID;

        mpRosMap->mvp_reference_map_points_id = opM->GetBackupReferenceMapPointsId();      // std::vector<MapPoint*> mvpReferenceMapPoints;

        mpRosMap->mb_imu_initialized = opM->GetImuInitialized();                   // bool mbImuInitialized;

        mpRosMap->mn_map_change  = opM->GetMapChange();                      // int mnMapChange;
        mpRosMap->mn_map_change_notified = opM->GetMapChangeNotified();               // int mnMapChangeNotified;
         
        mpRosMap->mn_init_kf_id = opM->GetInitKFid();                    // long unsigned int mnInitKFid;
        mpRosMap->mn_max_kf_id = opM->GetMaxKFid();                      // long unsigned int mnMaxKFid;
        // long unsigned int mnLastLoopKFid;

        // Index related to a big change in the map (loop closure, global BA)
        mpRosMap->mn_big_change_idx = opM->GetLastBigChangeIdx();                   // int mnBigChangeIdx;



        //std::cout << "before bools" << std::endl;
        mpRosMap->m_is_in_use = opM->IsInUse();                         // bool mIsInUse;
        mpRosMap->m_has_thumbnail = opM->HasThumbnail();                      // bool mHasTumbnail;
        mpRosMap->m_bad = opM->GetIsBad();                                // bool mbBad = false;

        mpRosMap->mb_is_inertial = opM->IsInertial();                       // bool mbIsInertial;
        mpRosMap->mb_imu_ba1 = opM->GetIniertialBA1();                           // bool mbIMU_BA1;
        mpRosMap->mb_imu_ba2 = opM->GetIniertialBA2();                           // bool mbIMU_BA2;


        //std::cout << "Map." << opM->GetId() << " Before ROS Broadcast - #MP=" << opM->GetAllMapPoints().size() <<", #KF=" << opM->GetAllKeyFrames().size() << ", #RefMPs" << opM->GetReferenceMapPoints().size() << ", initKF=" << opM->GetInitKFid() << ", MaxKF=" << opM->GetMaxKFid() << ", OriginKFid" << opM->GetOriginKF()->mnId << std::endl;


        return mpRosMap;
      }
      
      static orb_map* RosMapToOrbMap(const map::SharedPtr& rM, orb_map* mpPrevMap) {        
        

        const bool mbFail = rM->mb_fail;
        std::vector<unsigned long int>& mvOptKFs = rM->ms_opt_kfs;
        const std::set<unsigned long int> msOptKFs(mvOptKFs.begin(), mvOptKFs.end());
        std::vector<unsigned long int>& mvFixedKFs = rM->ms_fixed_kfs;
        const std::set<unsigned long int> msFixedKFs(mvFixedKFs.begin(), mvFixedKFs.end());
        const long unsigned int mnId = rM->mn_id;
        const std::vector<std::string>& mvpBackupMapPointsId = rM->mvp_backup_map_points_ids;
        const std::vector<unsigned long int>& mvpBackupKeyFramesId = rM->mvp_backup_keyframes_ids;
        
        std::vector<std::string>& mvUpdatedMPIds = rM->mvp_updated_map_points_ids;
        const std::set<std::string> msUpdatedMPIds(mvUpdatedMPIds.begin(), mvUpdatedMPIds.end());

        std::vector<unsigned long int>& mvUpdatedKFIds = rM->mvp_updated_keyframes_ids;
        const std::set<unsigned long int> msUpdatedKFIds(mvUpdatedKFIds.begin(), mvUpdatedKFIds.end());

        const vector<unsigned long int>& mvBackupKeyFrameOriginsId = rM->mv_backup_keyframe_origins_id;     // vector<unsigned long int> mvBackupKeyFrameOriginsId;
        const unsigned long int mnBackupKFinitialID = rM->mn_backup_kf_initial_id;
        const unsigned long int mnBackupKFlowerID = rM->mn_backup_kf_lower_id;
        const std::vector<std::string>& mvpBackupReferenceMapPointsId = rM->mvp_reference_map_points_id;
        const bool mbImuInitialized = rM->mb_imu_initialized;
        const int mnMapChange = rM->mn_map_change;
        const int mnMapChangeNotified = rM->mn_map_change_notified;
        const long unsigned int mnInitKFid = rM->mn_init_kf_id;
        const long unsigned int mnMaxKFid = rM->mn_max_kf_id;
        const int mnBigChangeIdx = rM->mn_big_change_idx;
        const bool mIsInUse = rM->m_is_in_use;
        const bool mHasTumbnail = rM->m_has_thumbnail;
        const bool mbBad = rM->m_bad;
        const bool mbIsInertial = rM->mb_is_inertial;
        const bool mbIMU_BA1 = rM->mb_imu_ba1;
        const bool mbIMU_BA2 = rM->mb_imu_ba2;

        std::vector<unsigned long int>& mvErasedKFIds = rM->mvp_erased_keyframe_ids;
        const std::set<unsigned long int> msErasedKFIds(mvErasedKFIds.begin(), mvErasedKFIds.end());

        std::vector<std::string>& mvErasedMPIds = rM->mvp_erased_mappoint_ids;
        const std::set<std::string> msErasedMPIds(mvErasedMPIds.begin(), mvErasedMPIds.end());

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
