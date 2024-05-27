#ifndef MAPPOINT_CONVERTER_HPP_
#define MAPPOINT_CONVERTER_HPP


#include "Converter.hpp"


namespace Converter {

  class MapPointConverter {
    using map_point = orbslam3_interfaces::msg::MapPoint;
    using map_point_update = orbslam3_interfaces::msg::MapPointUpdate;

    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;
    using orb_keyframe = ORB_SLAM3::KeyFrame;

    public:

      
      static map_point_update ORBSLAM3MapPointToROSMapPointUpdate(orb_map_point* pMp, long unsigned int hostKfId=-1) {
          map_point_update msgMp; // = FormDefaultMapPointMessage();
          
          // public
          msgMp.mn_id = pMp->mnId;
          msgMp.m_str_hex_id = pMp->mstrHexId;

          msgMp.n_obs = pMp-> nObs;
          
          orb_keyframe* mpHostKF = pMp->mpHostKF; 
          msgMp.mp_host_kf_id = (mpHostKF) ? mpHostKF->mnId : -1; 
          msgMp.mn_origin_map_id = pMp->mnOriginMapId;
          
          // Position in absolute coordinates
          msgMp.m_world_pos = CppToRos::EigenVector3fToVector3(pMp->GetWorldPos());
          
          // Keyframes observing the point and associated index in keyframe
          std::map<ORB_SLAM3::KeyFrame*,std::tuple<int,int> > obs = pMp->GetObservations(); 

          std::map<long unsigned int, int> obsBackup1;
          std::map<long unsigned int, int> obsBackup2;
          for(std::map<ORB_SLAM3::KeyFrame*,std::tuple<int,int> >::const_iterator it = obs.begin(), end = obs.end(); it != end; ++it)
          {
            ORB_SLAM3::KeyFrame* pKFi = it->first;
              if(!pKFi)
                  continue;
              obsBackup1[it->first->mnId] = get<0>(it->second);
              obsBackup2[it->first->mnId] = get<1>(it->second);
          }
          
          // For save relation without pointer, this is necessary for save/load function
          msgMp.m_backup_observations_id1 = CppToRos::MapToRosIntTupleVector(obsBackup1); 
          msgMp.m_backup_observations_id2 = CppToRos::MapToRosIntTupleVector(obsBackup2); 
          // Mean viewing direction
          msgMp.m_normal_vector = CppToRos::EigenVector3fToVector3(pMp->GetNormal());
          
          // Best Descriptor to fast matching
          msgMp.m_descriptor = CppToRos::CVMatToImage(pMp->GetDescriptor());
          
          // Refernce KeyFrame
          orb_keyframe* mpRefKF = pMp->GetReferenceKeyFrame();
          msgMp.m_backup_ref_kf_id = (mpRefKF) ? mpRefKF->mnId : pMp->GetRefBackup();
          
          // Tracking counters
          msgMp.mn_visible = pMp->GetVisible();
          msgMp.mn_found = pMp->GetFound();
         

          // Bad flag (we do not currently erase MapPoint from memory)
          msgMp.mb_bad = pMp->isBad();

          // For save relation without pointer, this is necessary for save/load function
          orb_map_point* mpReplaced = pMp->GetReplaced();
          msgMp.m_backup_replaced_id = (mpReplaced) ? mpReplaced->mstrHexId : ""; 

          // Scale invariance distances
          msgMp.mf_min_distance = pMp->GetMinDistance();
          msgMp.mf_max_distance = pMp->GetMaxDistance();
          
          orb_map* pM = pMp->GetMap();
          msgMp.mp_map_id = (pM != nullptr) ? pM->GetId() : -1; 
          
          msgMp.mn_last_module = pMp->GetLastModule();

          return msgMp;
      } 


      static map_point ORBSLAM3MapPointToROS(orb_map_point* pMp, long unsigned int hostKfId=-1) {
          map_point msgMp = FormDefaultMapPointMessage();
          
          // public
          msgMp.mn_id = pMp->mnId;
          msgMp.m_str_hex_id = pMp->mstrHexId;

          msgMp.n_next_id = pMp->nNextId;
          msgMp.mn_first_kf_id = pMp->mnFirstKFid;
          msgMp.mn_first_frame = pMp->mnFirstFrame;
          msgMp.n_obs = pMp-> nObs;
          
          // Variables used by the tracking
          msgMp.m_track_proj_x = pMp->mTrackProjX;
          msgMp.m_track_proj_y = pMp->mTrackProjY;
          msgMp.m_track_depth = pMp->mTrackDepth;
          msgMp.m_track_depth_r = pMp->mTrackDepthR;
          msgMp.m_track_proj_xr = pMp->mTrackProjXR;
          msgMp.m_track_proj_yr = pMp->mTrackProjYR;
          msgMp.mb_track_in_view = pMp->mbTrackInView;
          msgMp.mb_track_in_view_r = pMp->mbTrackInViewR;
          msgMp.mn_track_scale_level = pMp->mnTrackScaleLevel;
          msgMp.mn_track_scale_level_r = pMp->mnTrackScaleLevelR;
          msgMp.m_track_view_cos = pMp->mTrackViewCos;
          msgMp.m_track_view_cos_r = pMp->mTrackViewCosR;
          msgMp.mn_track_reference_for_frame = pMp->mnTrackReferenceForFrame;
          msgMp.mn_last_frame_seen = pMp->mnLastFrameSeen;
          
          // Variables used by local mapping
          msgMp.mn_ba_local_for_kf = pMp->mnBALocalForKF;
          msgMp.mn_fuse_candidate_for_kf = pMp->mnFuseCandidateForKF;
          
          // Variables used by loop closing
          msgMp.mn_loop_point_for_kf = pMp->mnLoopPointForKF;
          msgMp.mn_corrected_by_kf = pMp->mnCorrectedByKF;
          msgMp.mn_corrected_referece = pMp->mnCorrectedReference;
          msgMp.m_pos_gba = CppToRos::EigenVector3fToVector3(pMp->mPosGBA);
          msgMp.mn_ba_global_for_kf = pMp->mnBAGlobalForKF;
          msgMp.mn_ba_local_for_merge = pMp->mnBALocalForMerge;
          
          // Variables used by merging
          msgMp.m_pos_merge = CppToRos::EigenVector3fToVector3(pMp->mPosMerge);
          msgMp.m_normal_vector_merge = CppToRos::EigenVector3fToVector3(pMp->mNormalVectorMerge);
          // For inverse depth optimization
          msgMp.m_inv_depth = pMp->mInvDepth;
          msgMp.m_init_u = pMp->mInitU;
          msgMp.m_init_v = pMp->mInitV;
          orb_keyframe* mpHostKF = pMp->mpHostKF; 
          msgMp.mp_host_kf_id = (mpHostKF) ? mpHostKF->mnId : -1; 
          msgMp.mn_origin_map_id = pMp->mnOriginMapId;
          
          // Protected
          // Position in absolute coordinates
          msgMp.m_world_pos = CppToRos::EigenVector3fToVector3(pMp->GetWorldPos());
          
          // Keyframes observing the point and associated index in keyframe
          std::map<ORB_SLAM3::KeyFrame*,std::tuple<int,int> > obs = pMp->GetObservations(); 
          //msgMp.m_observations = CppToRos::MapToRosKeyValuePairVector(obs);

          std::map<long unsigned int, int> obsBackup1;
          std::map<long unsigned int, int> obsBackup2;
          for(std::map<ORB_SLAM3::KeyFrame*,std::tuple<int,int> >::const_iterator it = obs.begin(), end = obs.end(); it != end; ++it)
          {
            ORB_SLAM3::KeyFrame* pKFi = it->first;
              if(!pKFi)
                  continue;
              obsBackup1[it->first->mnId] = get<0>(it->second);
              obsBackup2[it->first->mnId] = get<1>(it->second);
          }
          
          // For save relation without pointer, this is necessary for save/load function
          msgMp.m_backup_observations_id1 = CppToRos::MapToRosIntTupleVector(obsBackup1); 
          msgMp.m_backup_observations_id2 = CppToRos::MapToRosIntTupleVector(obsBackup2); 
          // Mean viewing direction
          msgMp.m_normal_vector = CppToRos::EigenVector3fToVector3(pMp->GetNormal());
          
          // Best Descriptor to fast matching
          msgMp.m_descriptor = CppToRos::CVMatToImage(pMp->GetDescriptor());
          
          // Refernce KeyFrame
          orb_keyframe* mpRefKF = pMp->GetReferenceKeyFrame();
          msgMp.m_backup_ref_kf_id = (mpRefKF) ? mpRefKF->mnId : pMp->GetRefBackup();
          
          // Tracking counters
          msgMp.mn_visible = pMp->GetVisible();
          msgMp.mn_found = pMp->GetFound();
         

          // Bad flag (we do not currently erase MapPoint from memory)
          msgMp.mb_bad = pMp->isBad();

          // For save relation without pointer, this is necessary for save/load function
          orb_map_point* mpReplaced = pMp->GetReplaced();
          msgMp.m_backup_replaced_id = (mpReplaced) ? mpReplaced->mstrHexId : ""; 

          // Scale invariance distances
          msgMp.mf_min_distance = pMp->GetMinDistance();
          msgMp.mf_max_distance = pMp->GetMaxDistance();
          
          orb_map* pM = pMp->GetMap();
          msgMp.mp_map_id = (pM != nullptr) ? pM->GetId() : -1; 
          
          msgMp.mn_last_module = pMp->GetLastModule();

          return msgMp;
      } 

      static void RosMapPointUpdateToOrb(const map_point_update::SharedPtr& rMp, orb_map_point* mpExistingMP) {
        
          long long int mnId = rMp->mn_id;
          std::string mstrHexId = rMp->m_str_hex_id;
          int nObs = rMp->n_obs;
          
          long long int mBackupHostKFId = rMp->mp_host_kf_id;
          
          unsigned int mnOriginMapId = rMp->mn_origin_map_id;
          Eigen::Vector3f mWorldPos = RosToCpp::Vector3ToEigenVector3f(rMp->m_world_pos);

          std::map<long unsigned int, int> mBackupObservationsId1 = RosToCpp::IntTupleVectorToMap(rMp->m_backup_observations_id1);
          std::map<long unsigned int, int> mBackupObservationsId2 = RosToCpp::IntTupleVectorToMap(rMp->m_backup_observations_id2);

          Eigen::Vector3f mNormalVector = RosToCpp::Vector3ToEigenVector3f(rMp->m_normal_vector);
         

          long long int mBackupRefKFId = rMp->m_backup_ref_kf_id;
          int mnVisible = rMp->mn_visible;
          int mnFound = rMp->mn_found;
          bool mbBad = rMp->mb_bad;
          
          std::string mBackupReplacedId = rMp->m_backup_replaced_id;
          float mfMinDistance = rMp->mf_min_distance;
          float mfMaxDistance = rMp->mf_max_distance;
          
          cv::Mat mDescriptor = RosToCpp::ImageToCVMat(rMp->m_descriptor);
          
          unsigned int mnLastModule = rMp->mn_last_module;

          mpExistingMP->UpdateMapPoint(nObs, /*mpHostKF,*/ mBackupHostKFId, mnOriginMapId, mWorldPos, 
              /*mObservations,*/ mBackupObservationsId1, mBackupObservationsId2, mNormalVector, mDescriptor, 
              /*mpRefKF,*/ mBackupRefKFId, mnVisible, mnFound, mbBad, /*mpReplaced,*/ mBackupReplacedId, 
              mfMinDistance, mfMaxDistance /*mpMap*/, mnLastModule);
      }

      static orb_map_point* RosMapPointToOrb(const map_point::SharedPtr& rMp, orb_map_point* mpExistingMP = static_cast<orb_map_point*>(NULL)) {
        
          long long int mnId = rMp->mn_id;
          std::string mstrHexId = rMp->m_str_hex_id;
          long int mnFirstKFid = rMp->mn_first_kf_id;
          long int mnFirstFrame = rMp->mn_first_frame;
          int nObs = rMp->n_obs;
          long unsigned int mnBALocalForKF = rMp->mn_ba_local_for_kf;
          long unsigned int mnFuseCandidateForKF = rMp->mn_fuse_candidate_for_kf;
          long unsigned int mnLoopPointForKF = rMp->mn_loop_point_for_kf;
          long unsigned int mnCorrectedByKF = rMp->mn_corrected_by_kf;
          long unsigned int mnCorrectedReference = rMp->mn_corrected_referece;
          Eigen::Vector3f mPosGBA = RosToCpp::Vector3ToEigenVector3f(rMp->m_pos_gba);
          long unsigned int mnBAGlobalForKF = rMp->mn_ba_global_for_kf;
          long unsigned int mnBALocalForMerge = rMp->mn_ba_local_for_merge;
          Eigen::Vector3f mPosMerge = RosToCpp::Vector3ToEigenVector3f(rMp->m_pos_merge);
          Eigen::Vector3f mNormalVectorMerge = RosToCpp::Vector3ToEigenVector3f(rMp->m_normal_vector_merge);
          double mInvDepth = rMp->m_inv_depth;
          double mInitU = rMp->m_init_u;
          double mInitV = rMp->m_init_v;
          
          long long int mBackupHostKFId = rMp->mp_host_kf_id;
          
          unsigned int mnOriginMapId = rMp->mn_origin_map_id;
          Eigen::Vector3f mWorldPos = RosToCpp::Vector3ToEigenVector3f(rMp->m_world_pos);

          std::map<long unsigned int, int> mBackupObservationsId1 = RosToCpp::IntTupleVectorToMap(rMp->m_backup_observations_id1);
          std::map<long unsigned int, int> mBackupObservationsId2 = RosToCpp::IntTupleVectorToMap(rMp->m_backup_observations_id2);

          Eigen::Vector3f mNormalVector = RosToCpp::Vector3ToEigenVector3f(rMp->m_normal_vector);
          
          long long int mBackupRefKFId = rMp->m_backup_ref_kf_id;
          int mnVisible = rMp->mn_visible;
          int mnFound = rMp->mn_found;
          bool mbBad = rMp->mb_bad;
          
          std::string mBackupReplacedId = rMp->m_backup_replaced_id;
          float mfMinDistance = rMp->mf_min_distance;
          float mfMaxDistance = rMp->mf_max_distance;
          
          cv::Mat mDescriptor = RosToCpp::ImageToCVMat(rMp->m_descriptor);
          
          unsigned int mnLastModule = rMp->mn_last_module;

          if(mpExistingMP)
          {
              mpExistingMP->UpdateMapPoint(mnFirstKFid, mnFirstFrame, nObs, /*mTrackProjX, mTrackProjY, mTrackDepth,*/
                  /* mTrackDepthR, mTrackProjXR, mTrackProjYR, mbTrackInView, mbTrackInViewR, mnTrackScaleLevel,*/
                  /* mnTrackScaleLevelR, mTrackViewCos, mTrackViewCosR, mnTrackReferenceForFrame, mnLastFrameSeen,*/ 
                  mnBALocalForKF, mnFuseCandidateForKF, mnLoopPointForKF, mnCorrectedByKF, mnCorrectedReference, 
                  mPosGBA, mnBAGlobalForKF, mnBALocalForMerge, mPosMerge, mNormalVectorMerge, mInvDepth, mInitU, 
                  mInitV, /*mpHostKF,*/ mBackupHostKFId, mnOriginMapId, mWorldPos, /*mObservations,*/ mBackupObservationsId1, 
                  mBackupObservationsId2, mNormalVector, mDescriptor, /*mpRefKF,*/ mBackupRefKFId, mnVisible, 
                  mnFound, mbBad, /*mpReplaced,*/ mBackupReplacedId, mfMinDistance, mfMaxDistance /*mpMap*/, mnLastModule);
              return mpExistingMP;
          } else
          {
              float mTrackProjX = rMp->m_track_proj_x;
              float mTrackProjY = rMp->m_track_proj_y;
              float mTrackDepth = rMp->m_track_depth;
              float mTrackDepthR = rMp->m_track_depth_r;
              float mTrackProjXR = rMp->m_track_proj_xr;
              float mTrackProjYR = rMp->m_track_proj_yr;
              bool mbTrackInView = rMp->mb_track_in_view;
              bool mbTrackInViewR = rMp->mb_track_in_view_r;
              int mnTrackScaleLevel = rMp->mn_track_scale_level;
              int mnTrackScaleLevelR = rMp->mn_track_scale_level_r;
              float mTrackViewCos = rMp->m_track_view_cos;
              float mTrackViewCosR = rMp->m_track_view_cos_r;
              long unsigned int mnTrackReferenceForFrame = rMp->mn_track_reference_for_frame;
              long unsigned int mnLastFrameSeen = rMp->mn_last_frame_seen;

              return new orb_map_point(mnId, mstrHexId, mnFirstKFid, mnFirstFrame, nObs, mTrackProjX, mTrackProjY, 
                  mTrackDepth, mTrackDepthR, mTrackProjXR, mTrackProjYR, mbTrackInView, mbTrackInViewR, mnTrackScaleLevel, 
                  mnTrackScaleLevelR, mTrackViewCos, mTrackViewCosR, mnTrackReferenceForFrame, mnLastFrameSeen, mnBALocalForKF, 
                  mnFuseCandidateForKF, mnLoopPointForKF, mnCorrectedByKF, mnCorrectedReference, mPosGBA, mnBAGlobalForKF, 
                  mnBALocalForMerge, mPosMerge, mNormalVectorMerge, mInvDepth, mInitU, mInitV, /*mpHostKF,*/ 
                  mBackupHostKFId, mnOriginMapId, mWorldPos, /*mObservations,*/ mBackupObservationsId1, mBackupObservationsId2, 
                  mNormalVector, mDescriptor, /*mpRefKF,*/ mBackupRefKFId, mnVisible, mnFound, mbBad, /*mpReplaced,*/ 
                  mBackupReplacedId, mfMinDistance, mfMaxDistance /*mpMap*/, mnLastModule);
          }
      }

      static map_point FormDefaultMapPointMessage()
      {
          map_point msgMp;

          msgMp.mp_host_kf_id = -1;
          msgMp.m_backup_ref_kf_id = -1; 
          msgMp.m_backup_replaced_id= "";       
          msgMp.mp_map_id = -1;

          return msgMp;

      }

    private:

  };

};


#endif
