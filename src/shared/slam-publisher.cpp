#include "slam-publisher.hpp"
//using std::placeholders::_1;

SLAMPublisher::SLAMPublisher() 
: Node("SLAM_Publisher") {
  std::cout << "\n======== Initializing SLAM publisher =========" << std::endl;
  
  keyframe_publisher_ = this->create_publisher<orbslam3_interfaces::msg::KeyFrame>(
      "/KeyFrame", 
      10);

  publisher_ = this->create_publisher<std_msgs::msg::String>(
      "Chatter", 
      10);
}

SLAMPublisher::~SLAMPublisher() {
  // Stop all threads
  std::cout << "SLAMPublisher node destroyed." << std::endl;
}

void SLAMPublisher::publishKeyFrame(ORB_SLAM3::KeyFrame* pKf) {
  orbslam3_interfaces::msg::KeyFrame msgKf = KeyFrameConstructor(pKf);

  keyframe_publisher_->publish(msgKf);
}

void SLAMPublisher::publishMessage(const std::string& message_text) {
  auto message = std_msgs::msg::String();
  message.data = message_text;
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s", message.data.c_str());
  publisher_->publish(message);
}

orbslam3_interfaces::msg::KeyFrame SLAMPublisher::KeyFrameConstructor(ORB_SLAM3::KeyFrame* pKf) {
  orbslam3_interfaces::msg::KeyFrame msgKf;




  // Public 1
  msgKf.b_imu = pKf->bImu;

  // public 2
  msgKf.n_next_id = pKf->nNextId;
  msgKf.mn_id = pKf->mnId;
  msgKf.mn_frame_id = pKf->mnFrameId; //const

  msgKf.m_time_stamp = pKf->mTimeStamp;//const

  // Grid (to speed up feature matching)
  msgKf.mn_grid_cols = pKf->mnGridCols; //const
  msgKf.mn_grid_rows = pKf->mnGridRows;//const
  msgKf.mf_grid_element_width_inv = pKf->mfGridElementWidthInv;// const
  msgKf.mf_grid_element_height_inv = pKf->mfGridElementHeightInv;// const

  //// Variables used by the tracking
  msgKf.mn_track_reference_for_frame = pKf->mnTrackReferenceForFrame;
  msgKf.mn_fuse_target_for_kf = pKf->mnFuseTargetForKF;

  //// Variables used by the local mapping
  msgKf.mn_ba_local_for_kf = pKf->mnBALocalForKF;
  msgKf.mn_ba_fixed_for_kf = pKf->mnBAFixedForKF;

  //// Number of optimizations by BA (amount of iterations in BA)
  msgKf.mn_number_of_opt = pKf->mnNumberOfOpt;

  //// Variables used by the keyframe database
  msgKf.mn_loop_query = pKf->mnLoopQuery;
  msgKf.mn_loop_words = pKf->mnLoopWords; 
  msgKf.m_loop_score = pKf->mLoopScore;
  msgKf.mn_reloc_query = pKf->mnRelocQuery;
  msgKf.mn_reloc_words = pKf->mnRelocWords;
  msgKf.mn_reloc_score = pKf->mRelocScore;
  msgKf.mn_merge_query = pKf->mnMergeQuery;
  msgKf.mn_merge_words = pKf->mnMergeWords;
  msgKf.m_merge_score = pKf->mMergeScore;
  msgKf.mn_place_recognition_query = pKf->mnPlaceRecognitionQuery;
  msgKf.mn_place_recognition_words = pKf->mnPlaceRecognitionWords;
  msgKf.m_place_recognition_score = pKf->mPlaceRecognitionScore;

  msgKf.mb_current_place_recognition = pKf->mbCurrentPlaceRecognition;

  //// Variables used by loop closing
  ////std_msgs/Float32MultiArray m_tcw_gba #Sophus::SE3f mTcwGBA;
  ////std_msgs/Float32MultiArray m_tcw_bef_gba #Sophus::SE3f mTcwBefGBA;
  //geometry_msgs/Vector3 m_vwb_gba
  //geometry_msgs/Vector3 m_vwb_bef_gba
  ////IMU::Bias mBiasGBA;
  msgKf.mn_ba_global_for_kf = pKf->mnBAGlobalForKF;

  //// Variables used by merging
  ////std_msgs/Float32MultiArray m_tcw_merge #Sophus::SE3f mTcwMerge;
  ////std_msgs/Float32MultiArray m_tcw_bef_merge #Sophus::SE3f mTcwBefMerge;
  ////std_msgs/Float32MultiArray m_twc_bef_merge #Sophus::SE3f mTwcBefMerge;
  //geometry_msgs/Vector3 m_vwb_merge
  //geometry_msgs/Vector3 m_vwb_bef_merge
  ////IMU::Bias mBiasMerge;
  msgKf.mn_merge_corrected_for_kf = pKf->mnMergeCorrectedForKF;
  msgKf.mn_merge_for_kf = pKf->mnMergeForKF;
  msgKf.mf_scale_merge = pKf->mfScaleMerge;
  msgKf.mn_ba_local_for_merge = pKf->mnBALocalForMerge;

  msgKf.mf_scale = pKf->mfScale;

  //// Calibration parameters
  msgKf.fx = pKf->fx;
  msgKf.fy = pKf->fy;
  msgKf.cx = pKf->cx;
  msgKf.cy = pKf->cy;
  msgKf.invfx = pKf->invfx;
  msgKf.invfy = pKf->invfy;
  msgKf.mbf = pKf->mbf; 
  msgKf.mb = pKf->mb;
  msgKf.m_th_depth = pKf->mThDepth;// const
  //sensor_msgs/Image m_dist_coef

  //// Number of KeyPoints
  msgKf.n = pKf->N; //const

  //// KeyPoints, stereo coordinate and descriptors (all associated by an index)
  ////std_msgs/Float32MultiArray mv_keys #const std::vector<cv::KeyPoint> mvKeys;
  ////std_msgs/Float32MultiArray mv_keys_yb #const std::vector<cv::KeyPoint> mvKeysUn;
  msgKf.mvu_right = pKf->mvuRight; //const std::vector<float> mvuRight; // negative value for monocular points
  msgKf.mv_depth = pKf->mvuRight;//const std::vector<float> mvDepth; // negative value for monocular points
  
  
  // PRINT TYPE OF THE CV::MAT!!!
  // std::cout << "\n***************************************************************" << std::endl;
  //std::cout << pKf->mDescriptors.type() << std::endl;
  


  cv_bridge::CvImage img_bridge;
  sensor_msgs::msg::Image img_msg;
  img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC1, pKf->mDescriptors);
  img_bridge.toImageMsg(img_msg);//const cv::Mat mDescriptors;
  msgKf.m_descriptors = img_msg; 
  //// BoW
  ////DBoW2::BowVector mBowVec;
  ////DBoW2::FeatureVector mFeatVec;

  //// Pose relative to parent (this is computed when bad flag is activated)
  ////std_msgs/Float32MultiArray m_tcp #Sophus::SE3f mTcp;

  //// Scale
  msgKf.mn_scale_levels = pKf->mnScaleLevels; //const int mnScaleLevels;
  msgKf.mf_scale_factor = pKf->mfScaleFactor;//const float mfScaleFactor;
  msgKf.mf_log_scale_factor = pKf->mfLogScaleFactor;//const float mfLogScaleFactor;
  msgKf.mv_scale_factors = pKf->mvScaleFactors;//const std::vector<float> mvScaleFactors;
  msgKf.mv_level_sigma2 = pKf->mvLevelSigma2;//const std::vector<float> mvLevelSigma2;
  msgKf.mv_inv_level_sigma2 = pKf->mvInvLevelSigma2;//const std::vector<float> mvInvLevelSigma2;

  //// Image bounds and calibration
  msgKf.mn_min_x = pKf->mnMinX;//const int mnMinX;
  msgKf.mn_min_y = pKf->mnMinY;//const int mnMinY;
  msgKf.mn_max_x = pKf->mnMaxX;//const int mnMaxX;
  msgKf.mn_max_y = pKf->mnMaxY;//const int mnMaxY;

  //// Preintegrated IMU measurements from previous keyframe
  ////KeyFrame m_prev_kf
  ////KeyFrame m_next_kf

  ////IMU::Preintegrated* mpImuPreintegrated;
  ////IMU::Calib mImuCalib;

  msgKf.mn_origin_map_id = pKf->mnOriginMapId;

  msgKf.m_name_file = pKf->mNameFile;
    
  msgKf.mn_dataset = pKf->mnDataset;

  ////KeyFrame[] mvp_loop_cand_kfs #std::vector <KeyFrame*> mvpLoopCandKFs;
  ////KeyFrame[] mvp_merge_cand_kfs #std::vector <KeyFrame*> mvpMergeCandKFs;


  //// ------------------------------------------------------------------------
  //// protected

  ////Sophus::SE3<float> mTcw;
  ////Eigen::Matrix3f mRcw;
  ////Sophus::SE3<float> mTwc;
  ////Eigen::Matrix3f mRwc;

  //// IMU position
  ////Eigen::Vector3f mOwb;
  //// Velocity (Only used for inertial SLAM)
  ////Eigen::Vector3f mVw;
  //bool mb_has_velocity

//// Transformation matrix between cameras in stereo fisheye
////Sophus::SE3<float> mTlr;
////Sophus::SE3<float> mTrl;

//// Imu bias
////IMU::Bias mImuBias;

//// MapPoints associated to keypoints
////1MapPoint[] mvp_map_points #std::vector<MapPoint*> mvpMapPoints;
//// For save relation without pointer, this is necessary for save/load function
  //int64[] mv_backup_map_points_id //std::vector<long long int> mvBackupMapPointsId;

//// BoW
  //KeyFrameDatabase mp_key_frame_db //KeyFrameDatabase* mpKeyFrameDB;
////ORBVocabulary* mpORBvocabulary;

//// Grid over the image to speed up feature matching
////std::vector< std::vector <std::vector<size_t> > > mGrid;

////std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
////KeyFrame[] mvp_orfered_connected_keyframes #std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
  //int32[] mv_ordered_weights //std::vector<int> mvOrderedWeights;
  ////For save relation without pointer, this is necessary for save/load function
  ////std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

  //// Spanning Tree and Loop Edges
  //bool mb_first_connection //bool mbFirstConnection;
  ////KeyFrame mp_parent //KeyFrame* mpParent;
  ////KeyFrame[] msp_childrens //std::set<KeyFrame*> mspChildrens;
  //// KeyFrame[] msp_loop_edges //std::set<KeyFrame*> mspLoopEdges;
  //// KeyFrame[] msp_merge_edges //std::set<KeyFrame*> mspMergeEdges;
  //// For save relation without pointer, this is necessary for save/load function
  //int64 m_backup_parent_id //long long int mBackupParentId;
  //uint64[] mv_backup_childrens_id //std::vector<long unsigned int> mvBackupChildrensId;
  //uint64[] mv_backup_loop_edges_id //std::vector<long unsigned int> mvBackupLoopEdgesId;
  //uint64[] mv_backup_merge_edges_id //std::vector<long unsigned int> mvBackupMergeEdgesId;

  //// Bad flags
  //bool mb_not_erase //bool mbNotErase;
  //bool mb_to_be_erased //bool mbToBeErased;
  //bool mb_bad //bool mbBad;    

  //float32 m_half_baseline //float mHalfBaseline; // Only for visualization

  //// Map mp_map //Map* mpMap;

  //// Backup variables for inertial
  //int64 m_backup_prev_kf_id //long long int mBackupPrevKFId;
  //int64 m_backup_next_kf_id //long long int mBackupNextKFId;
  ////IMU::Preintegrated mBackupImuPreintegrated;

  ////Backup for Cameras
  //uint32 mn_backup_id_camera 
  //uint32 mn_backup_id_camera2 //unsigned int mnBackupIdCamera, mnBackupIdCamera2;

  ////Calibration
  ////Eigen::Matrix3f mK_;

  ////Mutex
  ////std::mutex mMutexPose; // for pose, velocity and biases
  ////std::mutex mMutexConnections;
  ////std::mutex mMutexFeatures;
  ////std::mutex mMutexMap;




  // ---------------------------------------------------------------------------------
  //// public
  //// GeometricCamera* mpCamera, *mpCamera2;

  ////Indexes of stereo observations correspondences
  msgKf.mv_left_to_right_match = pKf->mvLeftToRightMatch;
  msgKf.mv_right_to_left_match = pKf->mvRightToLeftMatch;//std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

  ////Sophus::SE3f GetRelativePoseTrl();
  ////Sophus::SE3f GetRelativePoseTlr();

  //// KeyPoints in the right image (for stereo fisheye, coordinates are needed)
  ////const std::vector<cv::KeyPoint> mvKeysRight;

  msgKf.n_left = pKf->NLeft;
  msgKf.n_right = pKf->NRight;//const int NLeft, NRight;

  //std::vector< std::vector <std::vector<size_t> > > mGridRight;


  return msgKf;
}


