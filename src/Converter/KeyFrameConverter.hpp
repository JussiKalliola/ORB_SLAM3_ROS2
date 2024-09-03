
#ifndef KEYFRAME_CONVERTER_HPP_
#define KEYFRAME_CONVERTER_HPP_

#include "MapPointConverter.hpp"
#include "Converter.hpp"


namespace Converter {
  class KeyFrameConverter {
    using keyframe = orbslam3_interfaces::msg::KeyFrame; 
    using keyframe_update = orbslam3_interfaces::msg::KeyFrameUpdate; 
    using map_point = orbslam3_interfaces::msg::MapPoint;

    using orb_keyframe = ORB_SLAM3::KeyFrame; 
    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;

    public: 
      static void UpdateORBKeyFrame(const keyframe_update::SharedPtr& rKf, orb_keyframe* mpExistingKF = static_cast<orb_keyframe*>(NULL)) {
                
        std::chrono::steady_clock::time_point time_Start = std::chrono::steady_clock::now();
        const long unsigned int mnId = rKf->mn_id;
        const long unsigned int mnRelocQuery = rKf->mn_reloc_query;
        const int mnRelocWords = rKf->mn_reloc_words;
        const float mRelocScore = rKf->mn_reloc_score;
        const long unsigned int mnPlaceRecognitionQuery = rKf->mn_place_recognition_query;
        const int mnPlaceRecognitionWords = rKf->mn_place_recognition_words;
        const float mPlaceRecognitionScore = rKf->m_place_recognition_score;

        const bool mbCurrentPlaceRecognition = rKf->mb_current_place_recognition;


        //float mfScale = rKf->mf_scale;;
        // Pose relative to parent (this is computed when bad flag is activated)
        const Sophus::SE3f mTcp = RosToCpp::PoseToSophusSE3f(rKf->m_tcp);
        // sophus poses
        const Sophus::SE3<float> mTcw = RosToCpp::PoseToSophusSE3f(rKf->m_tcw);
        std::vector<std::string> mvBackupMapPointsId(rKf->mv_backup_map_points_id.size());
        for(size_t i=0;i<rKf->mv_backup_map_points_id.size();i++)
        {
          mvBackupMapPointsId[i]=rKf->mv_backup_map_points_id[i];
        }
        //std::vector<orb_keyframe*> mvpOrderedConnectedKeyFrames = std::vector<orb_keyframe*>(); //std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames = rKf->;
        const std::vector<int>& mvOrderedWeights = rKf->mv_ordered_weights;
        // For save relation without pointer, this is necessary for save/load function
        const std::map<long unsigned int, int>& mBackupConnectedKeyFrameIdWeights = RosToCpp::IntTupleVectorToMap(rKf->m_backup_connected_keyframe_id_weights);
        // For save relation without pointer, this is necessary for save/load function
        const long long int mBackupParentId = rKf->m_backup_parent_id;
        const std::vector<long unsigned int>& mvBackupChildrensId = rKf->mv_backup_childrens_id;

        // Bad flags
        const bool mbNotErase = rKf->mb_not_erase;
        const bool mbToBeErased = rKf->mb_to_be_erased;
        const bool mbBad = rKf->mb_bad;    

        // Backup variables for inertial
        const long long int mBackupPrevKFId = rKf->m_backup_prev_kf_id;
        const long long int mBackupNextKFId = rKf->m_backup_next_kf_id;
        const unsigned int mnLastModule = rKf->mn_last_module;
        const unsigned int mnNextTarget = rKf->target;

        const bool mbLCDone = rKf->mb_lc_done;
        
        mpExistingKF->UpdateKeyFrame(mnRelocQuery, mnRelocWords, mRelocScore,  mnPlaceRecognitionQuery, mnPlaceRecognitionWords, mPlaceRecognitionScore, mbCurrentPlaceRecognition, /*mfScale,*/ mTcp, /*mPrevKF,*/ /*mNextKF,*/mTcw, /*mvpMapPoints,*/ mvBackupMapPointsId, /*mConnectedKeyFrameWeights,*/ /*mvpOrderedConnectedKeyFrames,*/ mvOrderedWeights, mBackupConnectedKeyFrameIdWeights,  /*mpParent,*/ /*mspChildrens,*/ mBackupParentId, mvBackupChildrensId, mbNotErase, mbToBeErased, mbBad,  /*mpMap,*/ mBackupPrevKFId, mBackupNextKFId, mnLastModule, mbLCDone, mnNextTarget);
          std::chrono::steady_clock::time_point time_End = std::chrono::steady_clock::now();
          double timeConv = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_End - time_Start).count();

          //std::cout << "KF CONVERSION (UPDATE) TIME=" << timeConv << std::endl;

      }   
      
      static void UpdateORBKeyFrame(const keyframe::SharedPtr& rKf, orb_keyframe* mpExistingKF = static_cast<orb_keyframe*>(NULL)) {
                
        std::chrono::steady_clock::time_point time_Start = std::chrono::steady_clock::now();
        //std::mutex mMutexNewKF;
        //std::lock_guard<std::mutex> lock(mMutexNewKF);
        const bool bImu = rKf->b_imu;
        const unsigned int mnNextTarget = rKf->target;
        
        const long unsigned int nNextId = rKf->n_next_id;
        const long unsigned int mnId = rKf->mn_id;
        const long unsigned int mnFrameId = rKf-> mn_frame_id;

        const double mTimeStamp = rKf->m_time_stamp;

        // Grid (to speed up feature matching)
        const int mnGridCols = rKf->mn_grid_cols;
        const int mnGridRows = rKf->mn_grid_rows;
        const float mfGridElementWidthInv = rKf->mf_grid_element_width_inv;
        const float mfGridElementHeightInv = rKf->mf_grid_element_height_inv;

        // Variables used by the tracking
        const long unsigned int mnTrackReferenceForFrame = rKf->mn_track_reference_for_frame;
        const long unsigned int mnFuseTargetForKF = rKf->mn_fuse_target_for_kf;

        // Variables used by the local mapping
        const long unsigned int mnBALocalForKF = rKf->mn_ba_local_for_kf;
        const long unsigned int mnBAFixedForKF = rKf->mn_ba_fixed_for_kf;

        //Number of optimizations by BA(amount of iterations in BA)
        const long unsigned int mnNumberOfOpt = rKf->mn_number_of_opt;

        // Variables used by the keyframe database
        const long unsigned int mnLoopQuery = rKf->mn_loop_query;
        const int mnLoopWords = rKf->mn_loop_words;
        const float mLoopScore = rKf->m_loop_score;
        const long unsigned int mnRelocQuery = rKf->mn_reloc_query;
        const int mnRelocWords = rKf->mn_reloc_words;
        const float mRelocScore = rKf->mn_reloc_score;
        const long unsigned int mnMergeQuery = rKf->mn_merge_query;
        const int mnMergeWords = rKf->mn_merge_words;
        const float mMergeScore = rKf->m_merge_score;
        const long unsigned int mnPlaceRecognitionQuery = rKf->mn_place_recognition_query;
        const int mnPlaceRecognitionWords = rKf->mn_place_recognition_words;
        const float mPlaceRecognitionScore = rKf->m_place_recognition_score;

        const bool mbCurrentPlaceRecognition = rKf->mb_current_place_recognition;


        // Variables used by loop closing
        const Sophus::SE3f mTcwGBA = RosToCpp::PoseToSophusSE3f(rKf->m_tcw_gba);
        const Sophus::SE3f mTcwBefGBA = RosToCpp::PoseToSophusSE3f(rKf->m_tcw_bef_gba);
        const Eigen::Vector3f mVwbGBA = RosToCpp::Vector3ToEigenVector3f(rKf->m_vwb_gba);
        const Eigen::Vector3f mVwbBefGBA = RosToCpp::Vector3ToEigenVector3f(rKf->m_vwb_bef_gba);
        //ORB_SLAM3::IMU::Bias mBiasGBA = RosToOrb::RosBiasToOrbImuBias(rKf->m_bias_gba);//IMU::Bias mBiasGBA = rKf->;
        const long unsigned int mnBAGlobalForKF = rKf->mn_ba_global_for_kf;

        // Variables used by merging
        const Sophus::SE3f mTcwMerge = RosToCpp::PoseToSophusSE3f(rKf->m_tcw_merge);
        const Sophus::SE3f mTcwBefMerge = RosToCpp::PoseToSophusSE3f(rKf->m_tcw_bef_merge);
        const Sophus::SE3f mTwcBefMerge = RosToCpp::PoseToSophusSE3f(rKf->m_twc_bef_merge);
        const Eigen::Vector3f mVwbMerge = RosToCpp::Vector3ToEigenVector3f(rKf->m_vwb_bef_merge);
        const Eigen::Vector3f mVwbBefMerge = RosToCpp::Vector3ToEigenVector3f(rKf->m_vwb_bef_merge);
        //ORB_SLAM3::IMU::Bias mBiasMerge = RosToOrb::RosBiasToOrbImuBias(rKf->m_bias_merge);  //IMU::Bias mBiasMerge = rKf->;
        const long unsigned int mnMergeCorrectedForKF = rKf->mn_merge_corrected_for_kf;
        const long unsigned int mnMergeForKF = rKf->mn_merge_for_kf;
        const float mfScaleMerge = rKf->mf_scale_merge;
        const long unsigned int mnBALocalForMerge = rKf->mn_ba_local_for_merge;

        const float mfScale = rKf->mf_scale;;


        // Pose relative to parent (this is computed when bad flag is activated)
        const Sophus::SE3f mTcp = RosToCpp::PoseToSophusSE3f(rKf->m_tcp);


        //ORB_SLAM3::IMU::Preintegrated* mpImuPreintegrated = nullptr; //IMU::Preintegrated* mpImuPreintegrated = rKf->;
        
        
        const std::vector<unsigned long int>& mvLoopCandKFIds = rKf->mvp_loop_cand_kfs_id;
        const std::vector<unsigned long int>& mvMergeCandKFIds = rKf->mvp_merge_cand_kfs_id;

        // The following variables need to be accessed trough a mutex to be thread safe.
        // sophus poses
        const Sophus::SE3<float> mTcw = RosToCpp::PoseToSophusSE3f(rKf->m_tcw);
        //Eigen::Matrix3f mRcw = mTcw.rotationMatrix();
        
        //Sophus::SE3<float> mTwc = RosToCpp::PoseToSophusSE3f(rKf->m_twc);
        //Eigen::Matrix3f mRwc = mTwc.rotationMatrix();

        // IMU position
        //Eigen::Vector3f mOwb = RosToCpp::Vector3ToEigenVector3f(rKf->m_owb);
        // Velocity (Only used for inertial SLAM)
        //Eigen::Vector3f mVw = RosToCpp::Vector3ToEigenVector3f(rKf->m_vw);
        const bool mbHasVelocity = rKf->mb_has_velocity;

        //Transformation matrix between cameras in stereo fisheye
        const Sophus::SE3<float> mTlr = RosToCpp::PoseToSophusSE3f(rKf->m_tlr);
        const Sophus::SE3<float> mTrl = RosToCpp::PoseToSophusSE3f(rKf->m_trl);


        // MapPoints associated to keypoints
        // For save relation without pointer, this is necessary for save/load function
        
        std::vector<std::string> mvBackupMapPointsId(rKf->mv_backup_map_points_id.size());
        for(size_t i=0;i<rKf->mv_backup_map_points_id.size();i++)
        {
          mvBackupMapPointsId[i]=rKf->mv_backup_map_points_id[i];
        }



        //std::vector<orb_keyframe*> mvpOrderedConnectedKeyFrames = std::vector<orb_keyframe*>(); //std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames = rKf->;
        const std::vector<int>& mvOrderedWeights = rKf->mv_ordered_weights;
        // For save relation without pointer, this is necessary for save/load function
        const std::map<long unsigned int, int>& mBackupConnectedKeyFrameIdWeights = RosToCpp::IntTupleVectorToMap(rKf->m_backup_connected_keyframe_id_weights);

        // Spanning Tree and Loop Edges
        const bool mbFirstConnection = rKf->mb_first_connection;//bool mbFirstConnection = rKf->mb_first_connection;
       
        
        // For save relation without pointer, this is necessary for save/load function
        const long long int mBackupParentId = rKf->m_backup_parent_id;
        const std::vector<long unsigned int>& mvBackupChildrensId = rKf->mv_backup_childrens_id;
        const std::vector<long unsigned int>& mvBackupLoopEdgesId = rKf->mv_backup_loop_edges_id;
        const std::vector<long unsigned int>& mvBackupMergeEdgesId = rKf->mv_backup_merge_edges_id;

        // Bad flags
        const bool mbNotErase = rKf->mb_not_erase;
        const bool mbToBeErased = rKf->mb_to_be_erased;
        const bool mbBad = rKf->mb_bad;    

        const float mHalfBaseline = 0.0;//float mHalfBaseline = rKf->; // Only for visualization

        // Backup variables for inertial
        const long long int mBackupPrevKFId = rKf->m_backup_prev_kf_id;
        const long long int mBackupNextKFId = rKf->m_backup_next_kf_id;
        //ORB_SLAM3::IMU::Preintegrated mBackupImuPreintegrated = ORB_SLAM3::IMU::Preintegrated();//IMU::Preintegrated mBackupImuPreintegrated = rKf->;

        // Backup for Cameras
        const unsigned int mnBackupIdCamera  = rKf->mn_backup_id_camera;
        const unsigned int mnBackupIdCamera2 = rKf->mn_backup_id_camera2;
        
        const unsigned int mnLastModule = rKf->mn_last_module;

        const bool mbLCDone = rKf->mb_lc_done;
        
        mpExistingKF->UpdateKeyFrame(bImu, mnNextTarget, /*mnFrameId, mTimeStamp,   mnGridCols, mnGridRows,  mfGridElementWidthInv,  mfGridElementHeightInv,*/ /*mnTrackReferenceForFrame, mnFuseTargetForKF,*/ mnBALocalForKF, mnBAFixedForKF, mnNumberOfOpt, mnLoopQuery, mnLoopWords, mLoopScore, mnRelocQuery, mnRelocWords, mRelocScore, mnMergeQuery, mnMergeWords, mMergeScore, mnPlaceRecognitionQuery, mnPlaceRecognitionWords, mPlaceRecognitionScore, mbCurrentPlaceRecognition, mTcwGBA, mTcwBefGBA, mVwbGBA, mVwbBefGBA, /*mBiasGBA,*/ mnBAGlobalForKF, mTcwMerge, mTcwBefMerge, mTwcBefMerge, mVwbMerge, mVwbBefMerge, /*mBiasMerge,*/ mnMergeCorrectedForKF, mnMergeForKF, mfScaleMerge, mnBALocalForMerge, mfScale, /*fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth,*/ /*mDistCoef,*/ /*N, mvKeys, mvKeysUn, mvuRight, mvDepth, mDescriptors,*/ /*mBowVec, mFeatVec,*/ mTcp, /*mnScaleLevels, mfScaleFactor, mfLogScaleFactor, mvScaleFactors, mvLevelSigma2, mvInvLevelSigma2, mnMinX, mnMinY, mnMaxX, mnMaxY,*/ /*mPrevKF,*/ /*mNextKF,*/ /*mpImuPreintegrated,*/ /*mImuCalib, mnOriginMapId, mNameFile, mnDataset, *//*mvpLoopCandKFs,*/ /*mvpMergeCandKFs,*/ mvLoopCandKFIds, mvMergeCandKFIds, mTcw, /*mRcw, mTwc, mRwc, mOwb, mVw,*/ mbHasVelocity, mTlr, mTrl, /*mImuBias,*/ /*mvpMapPoints,*/ mvBackupMapPointsId, /*mpKeyFrameDB,*/ /*mpORBvocabulary,*/ /*mGrid,*/ /*mConnectedKeyFrameWeights,*/ /*mvpOrderedConnectedKeyFrames,*/ mvOrderedWeights, mBackupConnectedKeyFrameIdWeights, mbFirstConnection, /*mpParent,*/ /*mspChildrens,*/ /*mspLoopEdges,*/ /*mspMergeEdges,*/ mBackupParentId, mvBackupChildrensId, mvBackupLoopEdgesId, mvBackupMergeEdgesId, mbNotErase, mbToBeErased, mbBad, mHalfBaseline, /*mpMap,*/ mBackupPrevKFId, mBackupNextKFId, /*mBackupImuPreintegrated, */  mnBackupIdCamera, mnBackupIdCamera2, /*mK_,*/ mnLastModule, mbLCDone /*mpCamera,*/ /*mpCamera2,*/ /*mvLeftToRightMatch, mvRightToLeftMatch,*/ /*mvKeysRight, NLeft, NRight,*/ /*mGridRight*/);
          std::chrono::steady_clock::time_point time_End = std::chrono::steady_clock::now();
          double timeConv = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_End - time_Start).count();

          //std::cout << "KF CONVERSION (UPDATE) TIME=" << timeConv << std::endl;

      }   
      
      static orb_keyframe* ROSKeyFrameToORBSLAM3(const keyframe::SharedPtr& rKf, orb_keyframe* mpExistingKF = static_cast<orb_keyframe*>(NULL)) {
                
        std::chrono::steady_clock::time_point time_Start = std::chrono::steady_clock::now();
        //std::mutex mMutexNewKF;
        //std::lock_guard<std::mutex> lock(mMutexNewKF);
        const bool bImu = rKf->b_imu;
        const unsigned int mnNextTarget = rKf->target;
        
        const long unsigned int nNextId = rKf->n_next_id;
        const long unsigned int mnId = rKf->mn_id;
        const long unsigned int mnFrameId = rKf-> mn_frame_id;

        const double mTimeStamp = rKf->m_time_stamp;

        // Grid (to speed up feature matching)
        const int mnGridCols = rKf->mn_grid_cols;
        const int mnGridRows = rKf->mn_grid_rows;
        const float mfGridElementWidthInv = rKf->mf_grid_element_width_inv;
        const float mfGridElementHeightInv = rKf->mf_grid_element_height_inv;

        // Variables used by the tracking
        const long unsigned int mnTrackReferenceForFrame = rKf->mn_track_reference_for_frame;
        const long unsigned int mnFuseTargetForKF = rKf->mn_fuse_target_for_kf;

        // Variables used by the local mapping
        const long unsigned int mnBALocalForKF = rKf->mn_ba_local_for_kf;
        const long unsigned int mnBAFixedForKF = rKf->mn_ba_fixed_for_kf;

        //Number of optimizations by BA(amount of iterations in BA)
        const long unsigned int mnNumberOfOpt = rKf->mn_number_of_opt;

        // Variables used by the keyframe database
        const long unsigned int mnLoopQuery = rKf->mn_loop_query;
        const int mnLoopWords = rKf->mn_loop_words;
        const float mLoopScore = rKf->m_loop_score;
        const long unsigned int mnRelocQuery = rKf->mn_reloc_query;
        const int mnRelocWords = rKf->mn_reloc_words;
        const float mRelocScore = rKf->mn_reloc_score;
        const long unsigned int mnMergeQuery = rKf->mn_merge_query;
        const int mnMergeWords = rKf->mn_merge_words;
        const float mMergeScore = rKf->m_merge_score;
        const long unsigned int mnPlaceRecognitionQuery = rKf->mn_place_recognition_query;
        const int mnPlaceRecognitionWords = rKf->mn_place_recognition_words;
        const float mPlaceRecognitionScore = rKf->m_place_recognition_score;

        const bool mbCurrentPlaceRecognition = rKf->mb_current_place_recognition;


        // Variables used by loop closing
        const Sophus::SE3f mTcwGBA = RosToCpp::PoseToSophusSE3f(rKf->m_tcw_gba);
        const Sophus::SE3f mTcwBefGBA = RosToCpp::PoseToSophusSE3f(rKf->m_tcw_bef_gba);
        const Eigen::Vector3f mVwbGBA = RosToCpp::Vector3ToEigenVector3f(rKf->m_vwb_gba);
        const Eigen::Vector3f mVwbBefGBA = RosToCpp::Vector3ToEigenVector3f(rKf->m_vwb_bef_gba);
        //ORB_SLAM3::IMU::Bias mBiasGBA = RosToOrb::RosBiasToOrbImuBias(rKf->m_bias_gba);//IMU::Bias mBiasGBA = rKf->;
        const long unsigned int mnBAGlobalForKF = rKf->mn_ba_global_for_kf;

        // Variables used by merging
        const Sophus::SE3f mTcwMerge = RosToCpp::PoseToSophusSE3f(rKf->m_tcw_merge);
        const Sophus::SE3f mTcwBefMerge = RosToCpp::PoseToSophusSE3f(rKf->m_tcw_bef_merge);
        const Sophus::SE3f mTwcBefMerge = RosToCpp::PoseToSophusSE3f(rKf->m_twc_bef_merge);
        const Eigen::Vector3f mVwbMerge = RosToCpp::Vector3ToEigenVector3f(rKf->m_vwb_bef_merge);
        const Eigen::Vector3f mVwbBefMerge = RosToCpp::Vector3ToEigenVector3f(rKf->m_vwb_bef_merge);
        //ORB_SLAM3::IMU::Bias mBiasMerge = RosToOrb::RosBiasToOrbImuBias(rKf->m_bias_merge);  //IMU::Bias mBiasMerge = rKf->;
        const long unsigned int mnMergeCorrectedForKF = rKf->mn_merge_corrected_for_kf;
        const long unsigned int mnMergeForKF = rKf->mn_merge_for_kf;
        const float mfScaleMerge = rKf->mf_scale_merge;
        const long unsigned int mnBALocalForMerge = rKf->mn_ba_local_for_merge;

        const float mfScale = rKf->mf_scale;;


        // Pose relative to parent (this is computed when bad flag is activated)
        const Sophus::SE3f mTcp = RosToCpp::PoseToSophusSE3f(rKf->m_tcp);


        //ORB_SLAM3::IMU::Preintegrated* mpImuPreintegrated = nullptr; //IMU::Preintegrated* mpImuPreintegrated = rKf->;
        
        
        const std::vector<unsigned long int>& mvLoopCandKFIds = rKf->mvp_loop_cand_kfs_id;
        const std::vector<unsigned long int>& mvMergeCandKFIds = rKf->mvp_merge_cand_kfs_id;

        // The following variables need to be accessed trough a mutex to be thread safe.
        // sophus poses
        const Sophus::SE3<float> mTcw = RosToCpp::PoseToSophusSE3f(rKf->m_tcw);
        //Eigen::Matrix3f mRcw = mTcw.rotationMatrix();
        
        //Sophus::SE3<float> mTwc = RosToCpp::PoseToSophusSE3f(rKf->m_twc);
        //Eigen::Matrix3f mRwc = mTwc.rotationMatrix();

        // IMU position
        //Eigen::Vector3f mOwb = RosToCpp::Vector3ToEigenVector3f(rKf->m_owb);
        // Velocity (Only used for inertial SLAM)
        //Eigen::Vector3f mVw = RosToCpp::Vector3ToEigenVector3f(rKf->m_vw);
        const bool mbHasVelocity = rKf->mb_has_velocity;

        //Transformation matrix between cameras in stereo fisheye
        const Sophus::SE3<float> mTlr = RosToCpp::PoseToSophusSE3f(rKf->m_tlr);
        const Sophus::SE3<float> mTrl = RosToCpp::PoseToSophusSE3f(rKf->m_trl);


        // MapPoints associated to keypoints
        // For save relation without pointer, this is necessary for save/load function
        std::vector<std::string> mvBackupMapPointsId;
        mvBackupMapPointsId.reserve(rKf->mv_backup_map_points_id.size()*3);
        for(size_t i=0;i<rKf->mv_backup_map_points_id.size();i++)
        {
          mvBackupMapPointsId.emplace_back(rKf->mv_backup_map_points_id[i]);
        }



        //std::vector<orb_keyframe*> mvpOrderedConnectedKeyFrames = std::vector<orb_keyframe*>(); //std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames = rKf->;
        const std::vector<int>& mvOrderedWeights = rKf->mv_ordered_weights;
        // For save relation without pointer, this is necessary for save/load function
        const std::map<long unsigned int, int>& mBackupConnectedKeyFrameIdWeights = RosToCpp::IntTupleVectorToMap(rKf->m_backup_connected_keyframe_id_weights);

        // Spanning Tree and Loop Edges
        const bool mbFirstConnection = rKf->mb_first_connection;//bool mbFirstConnection = rKf->mb_first_connection;
       
        
        // For save relation without pointer, this is necessary for save/load function
        const long long int mBackupParentId = rKf->m_backup_parent_id;
        const std::vector<long unsigned int>& mvBackupChildrensId = rKf->mv_backup_childrens_id;
        const std::vector<long unsigned int>& mvBackupLoopEdgesId = rKf->mv_backup_loop_edges_id;
        const std::vector<long unsigned int>& mvBackupMergeEdgesId = rKf->mv_backup_merge_edges_id;

        // Bad flags
        const bool mbNotErase = rKf->mb_not_erase;
        const bool mbToBeErased = rKf->mb_to_be_erased;
        const bool mbBad = rKf->mb_bad;    

        const float mHalfBaseline = 0.0;//float mHalfBaseline = rKf->; // Only for visualization

        // Backup variables for inertial
        const long long int mBackupPrevKFId = rKf->m_backup_prev_kf_id;
        const long long int mBackupNextKFId = rKf->m_backup_next_kf_id;
        //ORB_SLAM3::IMU::Preintegrated mBackupImuPreintegrated = ORB_SLAM3::IMU::Preintegrated();//IMU::Preintegrated mBackupImuPreintegrated = rKf->;

        // Backup for Cameras
        const unsigned int mnBackupIdCamera  = rKf->mn_backup_id_camera;
        const unsigned int mnBackupIdCamera2 = rKf->mn_backup_id_camera2;
        
        const unsigned int mnLastModule = rKf->mn_last_module;

        const bool mbLCDone = rKf->mb_lc_done;
        
        if(mpExistingKF)
        {
          mpExistingKF->UpdateKeyFrame(bImu, mnNextTarget, /*mnFrameId, mTimeStamp,   mnGridCols, mnGridRows,  mfGridElementWidthInv,  mfGridElementHeightInv,*/ /*mnTrackReferenceForFrame, mnFuseTargetForKF,*/ mnBALocalForKF, mnBAFixedForKF, mnNumberOfOpt, mnLoopQuery, mnLoopWords, mLoopScore, mnRelocQuery, mnRelocWords, mRelocScore, mnMergeQuery, mnMergeWords, mMergeScore, mnPlaceRecognitionQuery, mnPlaceRecognitionWords, mPlaceRecognitionScore, mbCurrentPlaceRecognition, mTcwGBA, mTcwBefGBA, mVwbGBA, mVwbBefGBA, /*mBiasGBA,*/ mnBAGlobalForKF, mTcwMerge, mTcwBefMerge, mTwcBefMerge, mVwbMerge, mVwbBefMerge, /*mBiasMerge,*/ mnMergeCorrectedForKF, mnMergeForKF, mfScaleMerge, mnBALocalForMerge, mfScale, /*fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth,*/ /*mDistCoef,*/ /*N, mvKeys, mvKeysUn, mvuRight, mvDepth, mDescriptors,*/ /*mBowVec, mFeatVec,*/ mTcp, /*mnScaleLevels, mfScaleFactor, mfLogScaleFactor, mvScaleFactors, mvLevelSigma2, mvInvLevelSigma2, mnMinX, mnMinY, mnMaxX, mnMaxY,*/ /*mPrevKF,*/ /*mNextKF,*/ /*mpImuPreintegrated,*/ /*mImuCalib, mnOriginMapId, mNameFile, mnDataset, *//*mvpLoopCandKFs,*/ /*mvpMergeCandKFs,*/ mvLoopCandKFIds, mvMergeCandKFIds, mTcw, /*mRcw, mTwc, mRwc, mOwb, mVw,*/ mbHasVelocity, mTlr, mTrl, /*mImuBias,*/ /*mvpMapPoints,*/ mvBackupMapPointsId, /*mpKeyFrameDB,*/ /*mpORBvocabulary,*/ /*mGrid,*/ /*mConnectedKeyFrameWeights,*/ /*mvpOrderedConnectedKeyFrames,*/ mvOrderedWeights, mBackupConnectedKeyFrameIdWeights, mbFirstConnection, /*mpParent,*/ /*mspChildrens,*/ /*mspLoopEdges,*/ /*mspMergeEdges,*/ mBackupParentId, mvBackupChildrensId, mvBackupLoopEdgesId, mvBackupMergeEdgesId, mbNotErase, mbToBeErased, mbBad, mHalfBaseline, /*mpMap,*/ mBackupPrevKFId, mBackupNextKFId, /*mBackupImuPreintegrated, */  mnBackupIdCamera, mnBackupIdCamera2, /*mK_,*/ mnLastModule, mbLCDone /*mpCamera,*/ /*mpCamera2,*/ /*mvLeftToRightMatch, mvRightToLeftMatch,*/ /*mvKeysRight, NLeft, NRight,*/ /*mGridRight*/);
          std::chrono::steady_clock::time_point time_End = std::chrono::steady_clock::now();
          double timeConv = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_End - time_Start).count();

          //std::cout << "KF CONVERSION (UPDATE) TIME=" << timeConv << std::endl;

          return mpExistingKF;
        } else {
          //BoW
          const DBoW2::BowVector mBowVec = DBoW2::BowVector(); //RosToOrb::RosBowVectorToDBoW2Vector(rKf->m_bow_vec); //DBoW2::BowVector mBowVec = rKf->;
          const  DBoW2::FeatureVector mFeatVec = DBoW2::FeatureVector(); //RosToOrb::RosBowFeatureVectorToDBoW2FeatureVector(rKf->m_feat_vec); //DBoW2::FeatureVector mFeatVec = rKf->;
          
          // Calibration parameters
          const float fx = rKf->fx;
          const float fy = rKf->fy;
          const float cx = rKf->cx;
          const float cy = rKf->cy;
          const float invfx = rKf->invfx;
          const float invfy = rKf->invfy;
          const float mbf = rKf->mbf;
          const float mb = rKf->mb;
          const float mThDepth = rKf->m_th_depth;
          const cv::Mat& mDistCoef = RosToCpp::ImageToCVMat(rKf->m_dist_coef);

          // Number of KeyPoints
          const int N = rKf->n;

          // KeyPoints, stereo coordinate and descriptors (all associated by an index)
          const std::vector<cv::KeyPoint>& mvKeys = RosToCpp::KeypointVectorToCVKeypointVector(rKf->mv_keys);
          const std::vector<cv::KeyPoint>& mvKeysUn = RosToCpp::KeypointVectorToCVKeypointVector(rKf->mv_keys_un);
          const std::vector<float> mvuRight = rKf->mvu_right; // negative value for monocular points
          const std::vector<float> mvDepth = rKf->mv_depth; // negative value for monocular points
          const cv::Mat& mDescriptors = RosToCpp::ImageToCVMat(rKf->m_descriptors);

          // Scale
          const int mnScaleLevels = rKf->mn_scale_levels;
          const float mfScaleFactor = rKf->mf_scale_factor;
          const float mfLogScaleFactor = rKf->mf_log_scale_factor;
          const std::vector<float> mvScaleFactors = rKf->mv_scale_factors;
          const std::vector<float> mvLevelSigma2 = rKf->mv_level_sigma2;
          const std::vector<float> mvInvLevelSigma2 = rKf->mv_inv_level_sigma2;

          // Image bounds and calibration
          const int mnMinX = rKf->mn_min_x;
          const int mnMinY = rKf->mn_min_y;
          const int mnMaxX = rKf->mn_max_x;
          const int mnMaxY = rKf->mn_max_y;

          const ORB_SLAM3::IMU::Calib mImuCalib = ORB_SLAM3::IMU::Calib();//IMU::Calib mImuCalib = rKf->;

          const unsigned int mnOriginMapId = rKf->mn_origin_map_id;

          const string mNameFile = rKf->m_name_file;

          const int mnDataset = rKf->mn_dataset;

          // Imu bias
          const ORB_SLAM3::IMU::Bias mImuBias = RosToOrb::RosBiasToOrbImuBias(rKf->m_imu_bias); //IMU::Bias mImuBias = rKf->;

          // Grid over the image to speed up feature matching
          const std::vector< std::vector <std::vector<size_t> > >& mGrid = Converter::RosToCpp::Grid3DToVector(rKf->m_grid);// std::vector< std::vector <std::vector<size_t> > > mGrid = rKf->;

          // Calibration
          const Eigen::Matrix3f mK_ = RosToCpp::MatrixToEigenMatrix3(rKf->m_k_calib);

          //ORB_SLAM3::GeometricCamera* mpCamera = nullptr;//GeometricCamera* mpCamera = rKf->
          //ORB_SLAM3::GeometricCamera* mpCamera2 = nullptr;//GeometricCamera* mpCamera2 = rKf->;

          //Indexes of stereo observations correspondences
          const std::vector<int>& mvLeftToRightMatch = rKf->mv_left_to_right_match;
          const std::vector<int>& mvRightToLeftMatch = rKf->mv_right_to_left_match;

          //KeyPoints in the right image (for stereo fisheye, coordinates are needed)
          const std::vector<cv::KeyPoint>& mvKeysRight = RosToCpp::KeypointVectorToCVKeypointVector(rKf->mv_keys_right);

          const int NLeft = rKf->n_left;
          const int NRight = rKf->n_right;

          
          //std::vector<orb_map_point*> mvpMapPoints = std::vector<orb_map_point*>(N);

          const std::vector< std::vector <std::vector<size_t> > >& mGridRight = std::vector< std::vector <std::vector<size_t> > >(); //std::vector< std::vector <std::vector<size_t> > > mGridRight = rKf->;
          

          orb_keyframe* newKF = new orb_keyframe(bImu, nNextId, mnNextTarget, mnId, mnFrameId, mTimeStamp,   mnGridCols, mnGridRows,  mfGridElementWidthInv,  mfGridElementHeightInv,  mnTrackReferenceForFrame, mnFuseTargetForKF, mnBALocalForKF, mnBAFixedForKF, mnNumberOfOpt, mnLoopQuery, mnLoopWords, mLoopScore, mnRelocQuery, mnRelocWords, mRelocScore, mnMergeQuery, mnMergeWords, mMergeScore, mnPlaceRecognitionQuery, mnPlaceRecognitionWords, mPlaceRecognitionScore, mbCurrentPlaceRecognition, mTcwGBA, mTcwBefGBA, mVwbGBA, mVwbBefGBA, /*mBiasGBA,*/ mnBAGlobalForKF, mTcwMerge, mTcwBefMerge, mTwcBefMerge, mVwbMerge, mVwbBefMerge, /*mBiasMerge,*/ mnMergeCorrectedForKF, mnMergeForKF, mfScaleMerge, mnBALocalForMerge, mfScale, fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth, mDistCoef, N, mvKeys, mvKeysUn, mvuRight, mvDepth, mDescriptors, mBowVec, mFeatVec, mTcp, mnScaleLevels, mfScaleFactor, mfLogScaleFactor, mvScaleFactors, mvLevelSigma2, mvInvLevelSigma2, mnMinX, mnMinY, mnMaxX, mnMaxY, /*mPrevKF,*/ /*mNextKF,*/ /*mpImuPreintegrated,*/ mImuCalib, mnOriginMapId, mNameFile, mnDataset, /*mvpLoopCandKFs,*/ /*mvpMergeCandKFs,*/ mvLoopCandKFIds, mvMergeCandKFIds, mTcw, /*mRcw, mTwc, mRwc, mOwb, mVw,*/ mbHasVelocity, mTlr, mTrl, /*mImuBias,*/ /*mvpMapPoints,*/ mvBackupMapPointsId, /*mpKeyFrameDB,*/ /*mpORBvocabulary,*/ mGrid, /*mConnectedKeyFrameWeights,*/ /*mvpOrderedConnectedKeyFrames,*/ mvOrderedWeights, mBackupConnectedKeyFrameIdWeights, mbFirstConnection, /*mpParent,*/ /*mspChildrens,*/ /*mspLoopEdges,*/ /*mspMergeEdges,*/ mBackupParentId, mvBackupChildrensId, mvBackupLoopEdgesId, mvBackupMergeEdgesId, mbNotErase, mbToBeErased, mbBad, mHalfBaseline, /*mpMap,*/ mBackupPrevKFId, mBackupNextKFId, /*mBackupImuPreintegrated, */  mnBackupIdCamera, mnBackupIdCamera2, mK_, mnLastModule, mbLCDone, /*mpCamera,*/ /*mpCamera2,*/ mvLeftToRightMatch, mvRightToLeftMatch, mvKeysRight, NLeft, NRight, mGridRight);

          std::chrono::steady_clock::time_point time_End = std::chrono::steady_clock::now();
          double timeConv = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_End - time_Start).count();

          //std::cout << "KF CONVERSION (NEW) TIME=" << timeConv << std::endl;

          return newKF;
        }
      }   

      static keyframe_update ORBSLAM3KeyFrameToROSKeyFrameUpdate(orb_keyframe* pKf, std::set<std::string>& mspUpdateMapPointIds, std::set<std::string>& mspErasedMPIds, bool mbProcessMPs) {
        keyframe_update msgKf; // = FormDefaultKeyFrameMessage();
        msgKf.system_id = std::getenv("SLAM_SYSTEM_ID");
        
        msgKf.mn_last_module = pKf->GetLastModule();
        msgKf.mb_lc_done = pKf->mbLCDone;
        msgKf.target = pKf->mnNextTarget;

        msgKf.mn_id = pKf->mnId;
        msgKf.mn_reloc_query = pKf->mnRelocQuery;
        msgKf.mn_reloc_words = pKf->mnRelocWords;
        msgKf.mn_reloc_score = pKf->mRelocScore;
        msgKf.mn_place_recognition_query = pKf->mnPlaceRecognitionQuery;
        msgKf.mn_place_recognition_words = pKf->mnPlaceRecognitionWords;
        msgKf.m_place_recognition_score = pKf->mPlaceRecognitionScore;

        msgKf.mb_current_place_recognition = pKf->mbCurrentPlaceRecognition;


        //msgKf.mf_scale = pKf->mfScale;
        
        // Pose relative to parent (this is computed when bad flag is activated)
        msgKf.m_tcp = CppToRos::SophusSE3fToPose(pKf->mTcp); //Sophus::SE3f mTcp;

        msgKf.mn_origin_map_id = pKf->mnOriginMapId;

        geometry_msgs::msg::Pose mTcw = CppToRos::SophusSE3fToPose(pKf->GetPose());
        msgKf.m_tcw = mTcw;  //Sophus::SE3<float> mTcw;
        msgKf.m_rcw = mTcw.orientation; //Eigen::Matrix3f mRcw;
        
        // MapPoints associated to keypoints
        const std::vector<ORB_SLAM3::MapPoint*>& mvpMapPoints = pKf->GetMapPointMatches();
        std::vector<std::string> mvpMapPointBackup(mvpMapPoints.size());

        //std::vector<map_point> msgMps;
        msgKf.mvp_map_points.reserve(mvpMapPoints.size());
        //std::set<std::string> mspErasedMPIds = pKf->GetMap()->GetErasedMPIds();
        //std::cout << " ************ mspUpdateMapPointIds.size()=" << mspUpdateMapPointIds.size() << std::endl;
        //std::pair<std::set<ORB_SLAM3::MapPoint*>, std::vector<unsigned long int>> MPsAndInidices = pKf->GetMapPointsAndIndices();
        //std::vector<unsigned long int> mvpIndices= MPsAndInidices.second;

        // For save relation without pointer, this is necessary for save/load function
        for(size_t i=0; i<mvpMapPoints.size(); ++i)
        {
            ORB_SLAM3::MapPoint* mp = mvpMapPoints[i];
            if(mp && !mp->isBad())//&& !mspErasedMPIds.count(mp->mstrHexId))
                mvpMapPointBackup[i] = mp->mstrHexId;
            else 
                mvpMapPointBackup[i] = "";
        }
        msgKf.mv_backup_map_points_id = mvpMapPointBackup; //std::vector<long long int> mvBackupMapPointsId;

        //std::cout << "mbProcessMPs" << std::endl;
        if(!mspUpdateMapPointIds.empty())
        {
            //std::cout << "mspUpdateMapPointIds not empty" << std::endl;
            for (size_t i=0; i<mvpMapPoints.size(); ++i) {
                ORB_SLAM3::MapPoint* mp = mvpMapPoints[i];
                if(mp)
                {
                    //if((pKf->GetLastModule() <= 2 && mp->GetLastModule() == 3))
                    //    continue;
                    if(mspUpdateMapPointIds.find(mp->mstrHexId) != mspUpdateMapPointIds.end())
                    {

                        msgKf.mvp_map_points.emplace_back(MapPointConverter::ORBSLAM3MapPointToROS(mp, pKf->mnId));
                        //mspUpdateMapPointIds.erase(mp->mstrHexId);
                        //msgIndices.push_back(mvpIndices[i]);
                        //const orbslam3_interfaces::msg::MapPoint& mRosMP = MapPointConverter::ORBSLAM3MapPointToROS(mp, pKf->mnId); 
                        //msgMps.push_back(mRosMP);
                        //msgMps.push_back(mRosMP);
                    }
                    //mspErasedMPIds.erase(mp->mstrHexId);
                }
                //if(msgMps.size() >= 200)
                //{
                //  break;
                //}

            }
        } else if(mspUpdateMapPointIds.empty() && mbProcessMPs){
           
            //std::cout << "mspUpdateMapPointIds empty" << std::endl;
            for (size_t i=0; i<mvpMapPoints.size(); ++i) {
                ORB_SLAM3::MapPoint* mp = mvpMapPoints[i];
                if(mp)
                {
                        //if((pKf->GetLastModule() <= 2 && mp->GetLastModule() == 3))
                        //    continue;
                        msgKf.mvp_map_points.emplace_back(MapPointConverter::ORBSLAM3MapPointToROS(mp, pKf->mnId));
                      //const orbslam3_interfaces::msg::MapPoint& mRosMP = MapPointConverter::ORBSLAM3MapPointToROS(mp, pKf->mnId); 
                      //msgMps.push_back(mRosMP);
                      //mspUpdateMapPointIds.erase(mp->mstrHexId);
                      //mspErasedMPIds.erase(mp->mstrHexId);
                }
                //if(msgMps.size() >= 200)
                //{
                //  break;
                //}
            }
        }
        
        //msgKf.mvp_map_points = &msgMps; //std::vector<MapPoint*> mvpMapPoints;
        //msgKf.mv_map_points_indices = msgIndices;


        
        const std::vector<orb_keyframe*>& mvpOrderedConnectedKeyFrames = pKf->GetVectorCovisibleKeyFrames();
        std::vector<long unsigned int> mvpOrderedConnectedKeyFramesId;
        mvpOrderedConnectedKeyFramesId.reserve(mvpOrderedConnectedKeyFrames.size());
        for(size_t i=0;i<mvpOrderedConnectedKeyFrames.size();i++)
        {
          orb_keyframe* tempKF = mvpOrderedConnectedKeyFrames[i];
          if(tempKF && !tempKF->isBad())
            continue;
          mvpOrderedConnectedKeyFramesId.push_back(tempKF->mnId);
        }
        msgKf.mvp_ordered_connected_keyframes_id = mvpOrderedConnectedKeyFramesId; //std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;      
        
        std::vector<int> mvOrderedWeights;
        mvOrderedWeights.reserve(mvpOrderedConnectedKeyFrames.size());
        for(size_t i=0;i<mvpOrderedConnectedKeyFrames.size();i++)
        {
          orb_keyframe* tempKF = mvpOrderedConnectedKeyFrames[i];
          if(tempKF && !tempKF->isBad())
            continue;
          mvOrderedWeights.push_back(pKf->GetWeight(tempKF));
        }

        msgKf.mv_ordered_weights = mvOrderedWeights; //std::vector<int> mvOrderedWeights;
        
        // For save relation without pointer, this is necessary for save/load function
        msgKf.m_backup_connected_keyframe_id_weights = CppToRos::MapToRosIntTupleVector(pKf->GetBackupConnectedKeyFrameIdWeights()); //std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;
        
        // For save relation without pointer, this is necessary for save/load function
        orb_keyframe* pKfP = pKf->GetParent();
        msgKf.m_backup_parent_id = (pKfP != nullptr) ? pKfP->mnId : -1;

        std::vector<long unsigned int> mspChildrensId;
        mspChildrensId.reserve(pKf->GetChilds().size());
        const std::set<orb_keyframe*>& mspChildrens=pKf->GetChilds();
        
        for(std::set<ORB_SLAM3::KeyFrame*>::iterator it = mspChildrens.begin(); it != mspChildrens.end(); ++it)
        {
          orb_keyframe* pKF = *it;
          if(pKF && !pKF->isBad())
            continue;
          mspChildrensId.push_back(pKF->mnId);
        }
        msgKf.mv_backup_childrens_id = mspChildrensId; //std::vector<long unsigned int> mvBackupChildrensId;
        

        // Bad flags
        msgKf.mb_not_erase = pKf->GetNotErase(); //bool mbNotErase;
        msgKf.mb_to_be_erased = pKf->GetToBeErased(); //bool mbToBeErased;
        msgKf.mb_bad = pKf->isBad(); //bool mbBad;    

        // Map mp_map //Map* mpMap;
        orb_map* pM = pKf->GetMap();
        msgKf.mp_map_id = (pM != nullptr) ? pM->GetId() : -1; 
        
        // Backup variables for inertial
        orb_keyframe* pKfPrev = pKf->mPrevKF;
        orb_keyframe* pKfNext = pKf->mNextKF;

        msgKf.m_backup_prev_kf_id = (pKfPrev) ? pKfPrev->mnId : -1;
        msgKf.m_backup_next_kf_id = (pKfNext) ? pKfNext->mnId : -1;

        return msgKf;

      }

      static keyframe::SharedPtr ORBSLAM3KeyFrameToROS(orb_keyframe* pKf, std::set<std::string>& mspUpdateMapPointIds, bool mbProcessMPs) {
        keyframe::SharedPtr mpRosKF = FormDefaultKeyFrameMessage();
        mpRosKF->system_id = std::getenv("SLAM_SYSTEM_ID");
        // Public 1
        mpRosKF->b_imu = pKf->bImu;
        mpRosKF->target = pKf->mnNextTarget;

        // public 2
        mpRosKF->n_next_id = pKf->nNextId;
        mpRosKF->mn_id = pKf->mnId;
        mpRosKF->mn_frame_id = pKf->mnFrameId; //const

        mpRosKF->m_time_stamp = pKf->mTimeStamp;//const

        // Grid (to speed up feature matching)
        mpRosKF->mn_grid_cols = pKf->mnGridCols; //const
        mpRosKF->mn_grid_rows = pKf->mnGridRows;//const
        mpRosKF->mf_grid_element_width_inv = pKf->mfGridElementWidthInv;// const
        mpRosKF->mf_grid_element_height_inv = pKf->mfGridElementHeightInv;// const
        
        // Variables used by the tracking
        mpRosKF->mn_track_reference_for_frame = pKf->mnTrackReferenceForFrame;
        mpRosKF->mn_fuse_target_for_kf = pKf->mnFuseTargetForKF;

        // Variables used by the local mapping
        mpRosKF->mn_ba_local_for_kf = pKf->mnBALocalForKF;
        mpRosKF->mn_ba_fixed_for_kf = pKf->mnBAFixedForKF;

        // Number of optimizations by BA (amount of iterations in BA)
        mpRosKF->mn_number_of_opt = pKf->mnNumberOfOpt;

        // Variables used by the keyframe database
        mpRosKF->mn_loop_query = pKf->mnLoopQuery;
        mpRosKF->mn_loop_words = pKf->mnLoopWords; 
        mpRosKF->m_loop_score = pKf->mLoopScore;
        mpRosKF->mn_reloc_query = pKf->mnRelocQuery;
        mpRosKF->mn_reloc_words = pKf->mnRelocWords;
        mpRosKF->mn_reloc_score = pKf->mRelocScore;
        mpRosKF->mn_merge_query = pKf->mnMergeQuery;
        mpRosKF->mn_merge_words = pKf->mnMergeWords;
        mpRosKF->m_merge_score = pKf->mMergeScore;
        mpRosKF->mn_place_recognition_query = pKf->mnPlaceRecognitionQuery;
        mpRosKF->mn_place_recognition_words = pKf->mnPlaceRecognitionWords;
        mpRosKF->m_place_recognition_score = pKf->mPlaceRecognitionScore;

        mpRosKF->mb_current_place_recognition = pKf->mbCurrentPlaceRecognition;

        // Variables used by loop closing
        mpRosKF->m_tcw_gba = CppToRos::SophusSE3fToPose(pKf->mTcwGBA);
        mpRosKF->m_tcw_bef_gba = CppToRos::SophusSE3fToPose(pKf->mTcwBefGBA); //Sophus::SE3f mTcwBefGBA;
        mpRosKF->m_vwb_gba = CppToRos::EigenVector3fToVector3(pKf->mVwbGBA);
        mpRosKF->m_vwb_bef_gba = CppToRos::EigenVector3fToVector3(pKf->mVwbBefGBA);
        
        mpRosKF->m_bias_gba = OrbToRos::ImuBiasToRosBias(pKf->mBiasGBA);
        
        mpRosKF->mn_ba_global_for_kf = pKf->mnBAGlobalForKF;

        // Variables used by merging
        mpRosKF->m_tcw_merge = CppToRos::SophusSE3fToPose(pKf->mTcwMerge); //Sophus::SE3f mTcwMerge;
        mpRosKF->m_tcw_bef_merge = CppToRos::SophusSE3fToPose(pKf->mTcwBefMerge);// Sophus::SE3f mTcwBefMerge;
        mpRosKF->m_twc_bef_merge = CppToRos::SophusSE3fToPose(pKf->mTwcBefMerge);// Sophus::SE3f mTwcBefMerge;
        mpRosKF->m_vwb_merge = CppToRos::EigenVector3fToVector3(pKf->mVwbMerge);
        mpRosKF->m_vwb_bef_merge = CppToRos::EigenVector3fToVector3(pKf->mVwbBefMerge);
        
        mpRosKF->m_bias_merge = OrbToRos::ImuBiasToRosBias(pKf->mBiasMerge); //IMU::Bias mBiasMerge;

        mpRosKF->mn_merge_corrected_for_kf = pKf->mnMergeCorrectedForKF;
        mpRosKF->mn_merge_for_kf = pKf->mnMergeForKF;
        mpRosKF->mf_scale_merge = pKf->mfScaleMerge;
        mpRosKF->mn_ba_local_for_merge = pKf->mnBALocalForMerge;

        mpRosKF->mf_scale = pKf->mfScale;
        
        // Calibration parameters
        mpRosKF->fx = pKf->fx;
        mpRosKF->fy = pKf->fy;
        mpRosKF->cx = pKf->cx;
        mpRosKF->cy = pKf->cy;
        mpRosKF->invfx = pKf->invfx;
        mpRosKF->invfy = pKf->invfy;
        mpRosKF->mbf = pKf->mbf; 
        mpRosKF->mb = pKf->mb;
        mpRosKF->m_th_depth = pKf->mThDepth;// const
        mpRosKF->m_dist_coef = CppToRos::CVMatToImage(pKf->mDistCoef);

        // Number of KeyPoints
        mpRosKF->n = pKf->N; //const

        //std::cout << "line 459" << std::endl;
        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        mpRosKF->mv_keys = CppToRos::CVKeypointVectorToPose2DVector(pKf->mvKeys); //const std::vector<cv::KeyPoint> mvKeys;
        mpRosKF->mv_keys_un = CppToRos::CVKeypointVectorToPose2DVector(pKf->mvKeysUn); //const std::vector<cv::KeyPoint> mvKeysUn;
        mpRosKF->mvu_right = pKf->mvuRight; //const std::vector<float> mvuRight; // negative value for monocular points
        mpRosKF->mv_depth = pKf->mvuRight;//const std::vector<float> mvDepth; // negative value for monocular points 
        mpRosKF->m_descriptors = CppToRos::CVMatToImage(pKf->mDescriptors.clone()); 

        // BoW
        //mpRosKF->m_bow_vec = OrbToRos::DBoW2VectorToRosBowVector(pKf->mBowVec); //DBoW2::BowVector mBowVec;
        //msgKf.m_feat_vec = OrbToRos::DBoW2FeatVectorToRosBowFeatureVector(pKf->mFeatVec); //DBoW2::FeatureVector mFeatVec;

        // Pose relative to parent (this is computed when bad flag is activated)
        mpRosKF->m_tcp = CppToRos::SophusSE3fToPose(pKf->mTcp); //Sophus::SE3f mTcp;

        
        // Scale
        mpRosKF->mn_scale_levels = pKf->mnScaleLevels; //const int mnScaleLevels;
        mpRosKF->mf_scale_factor = pKf->mfScaleFactor;//const float mfScaleFactor;
        mpRosKF->mf_log_scale_factor = pKf->mfLogScaleFactor;//const float mfLogScaleFactor;
        mpRosKF->mv_scale_factors = pKf->mvScaleFactors;//const std::vector<float> mvScaleFactors;
        mpRosKF->mv_level_sigma2 = pKf->mvLevelSigma2;//const std::vector<float> mvLevelSigma2;
        mpRosKF->mv_inv_level_sigma2 = pKf->mvInvLevelSigma2;//const std::vector<float> mvInvLevelSigma2;

        // Image bounds and calibration
        mpRosKF->mn_min_x = pKf->mnMinX;//const int mnMinX;
        mpRosKF->mn_min_y = pKf->mnMinY;//const int mnMinY;
        mpRosKF->mn_max_x = pKf->mnMaxX;//const int mnMaxX;
        mpRosKF->mn_max_y = pKf->mnMaxY;//const int mnMaxY;

        // Preintegrated IMU measurements from previous keyframe
        //KeyFrame m_prev_kf
        //KeyFrame m_next_kf
        if(pKf->mpImuPreintegrated != nullptr) {

          //mpRosKF->mp_imu_preintegrated = OrbToRos::ImuPreintegratedToRosPreintegrated(pKf->mpImuPreintegrated); //IMU::Preintegrated* mpImuPreintegrated;
          //mpRosKF->m_backup_imu_preintegrated = OrbToRos::ImuPreintegratedToRosPreintegrated(pKf->mpImuPreintegrated);//IMU::Preintegrated mBackupImuPreintegrated;
        }
        //IMU::Calib mImuCalib;

        mpRosKF->mn_origin_map_id = pKf->mnOriginMapId;

        mpRosKF->m_name_file = pKf->mNameFile;
          
        mpRosKF->mn_dataset = pKf->mnDataset;
        
        
        std::vector<orb_keyframe*> mvpLoopCandKFs=pKf->mvpLoopCandKFs;
        std::vector<long unsigned int> mvpLoopCandKFsId;
        mvpLoopCandKFsId.reserve(mvpLoopCandKFs.size());
        
        std::vector<orb_keyframe*> mvpMergeCandKFs=pKf->mvpMergeCandKFs;
        std::vector<long unsigned int> mvpMergeCandKFsId;
        mvpMergeCandKFsId.reserve(mvpMergeCandKFs.size());

        for(size_t i = 0; i<mvpLoopCandKFs.size();i++)
        {
            ORB_SLAM3::KeyFrame* tempKF = mvpLoopCandKFs[i];
            if(tempKF && !tempKF->isBad())
                continue;
            mvpLoopCandKFsId.push_back(tempKF->mnId);
        }

        for(size_t i = 0; i<mvpMergeCandKFs.size();i++)
        {
            ORB_SLAM3::KeyFrame* tempKF = mvpMergeCandKFs[i];
            if(tempKF && !tempKF->isBad())
                continue;
            mvpMergeCandKFsId.push_back(tempKF->mnId);
        }

        mpRosKF->mvp_loop_cand_kfs_id = mvpLoopCandKFsId;
        mpRosKF->mvp_merge_cand_kfs_id = mvpMergeCandKFsId;

        // ------------------------------------------------------------------------
        // protected

        geometry_msgs::msg::Pose mTcw = CppToRos::SophusSE3fToPose(pKf->GetPose());
        mpRosKF->m_tcw = mTcw;  //Sophus::SE3<float> mTcw;
        mpRosKF->m_rcw = mTcw.orientation; //Eigen::Matrix3f mRcw;
        
        geometry_msgs::msg::Pose mTwc = CppToRos::SophusSE3fToPose(pKf->GetPoseInverse());
        mpRosKF->m_twc = mTwc; //Sophus::SE3<float> mTwc;
        mpRosKF->m_rwc = mTwc.orientation; //Eigen::Matrix3f mRwc;
        // IMU position
        mpRosKF->m_owb = CppToRos::EigenVector3fToVector3(pKf->GetImuPosition()); //Eigen::Vector3f mOwb;
        // Velocity (Only used for inertial SLAM)
        mpRosKF->m_vw = CppToRos::EigenVector3fToVector3(pKf->GetVelocity()); //Eigen::Vector3f mVw;
        mpRosKF->mb_has_velocity = pKf->isVelocitySet();
        
        // Transformation matrix between cameras in stereo fisheye
        mpRosKF->m_tlr = CppToRos::SophusSE3fToPose(pKf->GetRelativePoseTrl()); // Sophus::SE3f GetRelativePoseTrl();
        mpRosKF->m_trl = CppToRos::SophusSE3fToPose(pKf->GetRelativePoseTlr()); // Sophus::SE3f GetRelativePoseTlr();

        // Imu bias 
        mpRosKF->m_imu_bias = OrbToRos::ImuBiasToRosBias(pKf->GetImuBias()); //IMU::Bias mImuBias;

        //std::cout << "line 548" << std::endl;
        // MapPoints associated to keypoints
        //std::vector<unsigned long int> msgIndices;
        
        const std::vector<ORB_SLAM3::MapPoint*>& mvpMapPoints = pKf->GetMapPointMatches();
        std::vector<std::string> mvpMapPointBackup(mvpMapPoints.size());

        std::vector<map_point> msgMps;
        mpRosKF->mvp_map_points.reserve(mvpMapPoints.size());

        std::set<std::string>& mspErasedMPIds = pKf->GetMap()->GetErasedMPIds();
        std::cout << " ************ mspUpdateMapPointIds.size()=" << mspUpdateMapPointIds.size() << ", mspErasedMPIds.size()=" << mspErasedMPIds.size() << std::endl;
        //std::pair<std::set<ORB_SLAM3::MapPoint*>, std::vector<unsigned long int>> MPsAndInidices = pKf->GetMapPointsAndIndices();
        //std::vector<unsigned long int> mvpIndices= MPsAndInidices.second;
        //
        // For save relation without pointer, this is necessary for save/load function
        for(size_t i=0; i<mvpMapPoints.size(); ++i)
        {
            ORB_SLAM3::MapPoint* mp = mvpMapPoints[i];
            if(mp && !mp->isBad())//&& !mspErasedMPIds.count(mp->mstrHexId))
                mvpMapPointBackup[i] = mp->mstrHexId;
            else 
                mvpMapPointBackup[i] = "";
        }
        mpRosKF->mv_backup_map_points_id = mvpMapPointBackup; //std::vector<long long int> mvBackupMapPointsId;
         
        //std::cout << "mbProcessMPs" << std::endl;
        if(!mspUpdateMapPointIds.empty())
        {
            //std::cout << "mspUpdateMapPointIds not empty" << std::endl;
            for (size_t i=0; i<mvpMapPoints.size(); ++i) {
                ORB_SLAM3::MapPoint* mp = mvpMapPoints[i];
                if(mp && !mp->isBad())
                {
                    if(mspUpdateMapPointIds.find(mp->mstrHexId) != mspUpdateMapPointIds.end())
                    {

                        //msgIndices.push_back(mvpIndices[i]);
                        mpRosKF->mvp_map_points.emplace_back(MapPointConverter::ORBSLAM3MapPointToROS(mp, pKf->mnId));
                      //const orbslam3_interfaces::msg::MapPoint& mRosMP = MapPointConverter::ORBSLAM3MapPointToROS(mp, pKf->mnId); 
                      //msgMps.push_back(mRosMP);
                    }
                    mspUpdateMapPointIds.erase(mp->mstrHexId);
                }

            }
        } else if(mspUpdateMapPointIds.empty() && mbProcessMPs){
           
            //std::cout << "mspUpdateMapPointIds empty" << std::endl;
            for (size_t i=0; i<mvpMapPoints.size(); ++i) {
                ORB_SLAM3::MapPoint* mp = mvpMapPoints[i];
                if(mp && !mp->isBad())
                {
                      //msgIndices.push_back(mvpIndices[i]);
                        mpRosKF->mvp_map_points.emplace_back(MapPointConverter::ORBSLAM3MapPointToROS(mp, pKf->mnId));
                      //const orbslam3_interfaces::msg::MapPoint& mRosMP =  
                      //msgMps.push_back(mRosMP);
                      mspUpdateMapPointIds.erase(mp->mstrHexId);
                      //mspUpdateMapPointIds.erase(mp->mstrHexId);
                }

            }
        }
        
        //mpRosKF->mvp_map_points = &msgMps; //std::vector<MapPoint*> mvpMapPoints;
        //mpRosKF->mv_map_points_indices = msgIndices;


        // BoW
        //KeyFrameDatabase mp_key_frame_db //KeyFrameDatabase* mpKeyFrameDB;
        //ORBVocabulary* mpORBvocabulary;


        // Grid over the image to speed up feature matching
        mpRosKF->m_grid = CppToRos::VectorToGrid3D(pKf->GetMGrid()); //std::vector< std::vector <std::vector<size_t> > > mGrid;

        //std::map<KeyFrame*,int> mConnectedKeyFrameWeights;                    // Done in m_backup_connected_keyframe_id_weights
        
        const std::vector<orb_keyframe*>& mvpOrderedConnectedKeyFrames = pKf->GetVectorCovisibleKeyFrames();
        std::vector<long unsigned int> mvpOrderedConnectedKeyFramesId;
        mvpOrderedConnectedKeyFramesId.reserve(mvpOrderedConnectedKeyFrames.size());
        for(size_t i=0;i<mvpOrderedConnectedKeyFrames.size();i++)
        {
          orb_keyframe* tempKF = mvpOrderedConnectedKeyFrames[i];
          if(tempKF && !tempKF->isBad())
            continue;
          mvpOrderedConnectedKeyFramesId.push_back(tempKF->mnId);
        }
        mpRosKF->mvp_ordered_connected_keyframes_id = mvpOrderedConnectedKeyFramesId; //std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;      
        
        std::vector<int> mvOrderedWeights;
        mvOrderedWeights.reserve(mvpOrderedConnectedKeyFrames.size());
        for(size_t i=0;i<mvpOrderedConnectedKeyFrames.size();i++)
        {
          orb_keyframe* tempKF = mvpOrderedConnectedKeyFrames[i];
          if(tempKF && !tempKF->isBad())
            continue;
          mvOrderedWeights.push_back(pKf->GetWeight(tempKF));
        }

        mpRosKF->mv_ordered_weights = mvOrderedWeights; //std::vector<int> mvOrderedWeights;
        
        // For save relation without pointer, this is necessary for save/load function
        mpRosKF->m_backup_connected_keyframe_id_weights = CppToRos::MapToRosIntTupleVector(pKf->GetBackupConnectedKeyFrameIdWeights()); //std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;
        
        // Spanning Tree and Loop Edges
        mpRosKF->mb_first_connection = pKf->GetFirstConnection();
        
        // For save relation without pointer, this is necessary for save/load function
        orb_keyframe* pKfP = pKf->GetParent();
        mpRosKF->m_backup_parent_id = (pKfP != nullptr) ? pKfP->mnId : -1;

        std::vector<long unsigned int> mspChildrensId;
        mspChildrensId.reserve(pKf->GetChilds().size());
        const std::set<orb_keyframe*>& mspChildrens=pKf->GetChilds();
        
        for(std::set<ORB_SLAM3::KeyFrame*>::iterator it = mspChildrens.begin(); it != mspChildrens.end(); ++it)
        {
          orb_keyframe* pKF = *it;
          if(pKF && !pKF->isBad())
            continue;
          mspChildrensId.push_back(pKF->mnId);
        }
        mpRosKF->mv_backup_childrens_id = mspChildrensId; //std::vector<long unsigned int> mvBackupChildrensId;
        
        std::vector<long unsigned int> mspLoopEdgesId;
        mspLoopEdgesId.reserve(pKf->GetLoopEdges().size());
        const std::set<orb_keyframe*>& mspLoopEdges=pKf->GetLoopEdges();

        for(std::set<ORB_SLAM3::KeyFrame*>::iterator it = mspLoopEdges.begin(); it != mspLoopEdges.end(); ++it)
        {
          orb_keyframe* pKF = *it;
          if(pKF && !pKF->isBad())
            continue;
          mspLoopEdgesId.push_back(pKF->mnId);
        }
        mpRosKF->mv_backup_loop_edges_id = mspLoopEdgesId; //std::vector<long unsigned int> mvBackupLoopEdgesId;
          
        std::vector<long unsigned int> mspMergeEdgesId;
        mspMergeEdgesId.reserve(pKf->GetMergeEdges().size());
        const std::set<orb_keyframe*>& mspMergeEdges=pKf->GetMergeEdges();

        for(std::set<ORB_SLAM3::KeyFrame*>::iterator it = mspMergeEdges.begin(); it != mspMergeEdges.end(); ++it)
        {
          orb_keyframe* pKF = *it;
          if(pKF && !pKF->isBad())
            continue;

          mspMergeEdgesId.push_back(pKF->mnId);
        }
        mpRosKF->mv_backup_merge_edges_id = mspMergeEdgesId; //std::vector<long unsigned int> mvBackupMergeEdgesId;
        

        // Bad flags
        mpRosKF->mb_not_erase = pKf->GetNotErase(); //bool mbNotErase;
        mpRosKF->mb_to_be_erased = pKf->GetToBeErased(); //bool mbToBeErased;
        mpRosKF->mb_bad = pKf->isBad(); //bool mbBad;    

        //float32 m_half_baseline //float mHalfBaseline; // Only for visualization

        // Map mp_map //Map* mpMap;
        orb_map* pM = pKf->GetMap();
        mpRosKF->mp_map_id = (pM != nullptr) ? pM->GetId() : -1; 
        
        // Backup variables for inertial
        orb_keyframe* pKfPrev = pKf->mPrevKF;
        orb_keyframe* pKfNext = pKf->mNextKF;

        mpRosKF->m_backup_prev_kf_id = (pKfPrev) ? pKfPrev->mnId : -1;
        mpRosKF->m_backup_next_kf_id = (pKfNext) ? pKfNext->mnId : -1;

        // Backup for Cameras
        ORB_SLAM3::GeometricCamera* mpCamera = pKf->mpCamera;
        ORB_SLAM3::GeometricCamera* mpCamera2 = pKf->mpCamera2;

        mpRosKF->mn_backup_id_camera = (mpCamera) ? mpCamera->GetId() : -1; 
        mpRosKF->mn_backup_id_camera2 = (mpCamera2) ? mpCamera2->GetId() : -1;

        // Calibration
        mpRosKF->m_k_calib = CppToRos::EigenMatrix3ToMatrix(pKf->GetCalibrationMatrix()); //Eigen::Matrix3f mK_;
        
        mpRosKF->mn_last_module = pKf->GetLastModule();
        mpRosKF->mb_lc_done = pKf->mbLCDone;

        // ---------------------------------------------------------------------------------
        // public
        // GeometricCamera* mpCamera, *mpCamera2;

        //Indexes of stereo observations correspondences
        //mpRosKF->mv_left_to_right_match = pKf->mvLeftToRightMatch;
        //mpRosKF->mv_right_to_left_match = pKf->mvRightToLeftMatch;//std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

        // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
        //mpRosKF->mv_keys_right = CppToRos::CVKeypointVectorToPose2DVector(pKf->mvKeysRight); //const std::vector<cv::KeyPoint> mvKeysRight;
        mpRosKF->n_left = pKf->NLeft;
        mpRosKF->n_right = pKf->NRight;//const int NLeft, NRight;


        // Figure how to take nulls into consideration with grids. Not needed for monocular.
        //mpRosKF->m_grid_right = toGrid3D(pKf->mGridRight); //std::vector< std::vector <std::vector<size_t> > > mGridRight;

        return mpRosKF;

      }

      static keyframe::SharedPtr FormDefaultKeyFrameMessage()
      {
        keyframe msgKf;
        keyframe::SharedPtr mpRosKF=std::make_shared<keyframe>(msgKf);

        mpRosKF->m_backup_prev_kf_id = -1;
        mpRosKF->m_backup_next_kf_id = -1; 
        mpRosKF->m_backup_parent_id = -1;       
        mpRosKF->mp_map_id = -1;
        return mpRosKF;

      }
  };
};


#endif
