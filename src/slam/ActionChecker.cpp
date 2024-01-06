#include "ActionChecker.hpp"

ActionChecker::ActionChecker(ORB_SLAM3::Atlas* mpAtlas, ORB_SLAM3::KeyFrameDatabase* mpKfDb): mbFinish(false), mbStop(false) {
  mpKfDb_=mpKfDb; 
  mpAtlas_=mpAtlas;

  // Initialize maps for the SLAM data, unprocessed and processed
  mpUnprocOrbKeyFrames=std::map<long unsigned int, std::tuple<ORB_SLAM3::KeyFrame*, orbslam3_interfaces::msg::KeyFrame::SharedPtr>>();
  mpUnprocOrbMapPoints=std::map<long unsigned int, std::tuple<ORB_SLAM3::MapPoint*, orbslam3_interfaces::msg::MapPoint::SharedPtr>>();
  mpUnprocOrbMaps=std::map<long unsigned int, std::tuple<ORB_SLAM3::Map*, orbslam3_interfaces::msg::Map::SharedPtr>>();

  mpOrbKeyFrames=std::map<long unsigned int, ORB_SLAM3::KeyFrame*>();
  mpOrbMapPoints=std::map<long unsigned int, ORB_SLAM3::MapPoint*>();
  mpOrbMaps=std::map<long unsigned int, ORB_SLAM3::Map*>();
  

}



ActionChecker::~ActionChecker() {

}


void ActionChecker::Run() {
  while(1) 
  {
    std::cout << "This should run every second." << std::endl;
      
    if(CheckNewKeyFrames())
    {
      unique_lock<mutex> lock(mMutexNewKFs);
      // Check if variables can be filled with new data
      std::cout << "Number of unprocessed KeyFrames before=" << mpUnprocOrbKeyFrames.size() << ", ";
      for (auto it = mpUnprocOrbKeyFrames.begin(); it != mpUnprocOrbKeyFrames.end();)
      {

        bool bUnprocessed = false;
        auto& mtORKf = it->second;
        ORB_SLAM3::KeyFrame* mopKf = std::get<0>(mtORKf);
        orbslam3_interfaces::msg::KeyFrame::SharedPtr mrpKf = std::get<1>(mtORKf);
        Converter::KeyFrameConverter::FillKeyFrameData(mopKf, mrpKf, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

        if(!bUnprocessed)
        {
          InsertKeyFrame(mopKf);
          it = mpUnprocOrbKeyFrames.erase(it);
        } else {
          ++it;
        }
      }
      std::cout << "and after=" << mpUnprocOrbKeyFrames.size() << std::endl;
    }

    if(CheckNewMapPoints())
    {
      // Check map points
      std::cout << "Number of unprocessed MapPoints before=" << mpUnprocOrbMapPoints.size() << ", ";
      for (auto it = mpUnprocOrbMapPoints.begin(); it != mpUnprocOrbMapPoints.end();)
      {

        bool bUnprocessed = false;
        auto& mtORMP = it->second;
        ORB_SLAM3::MapPoint* mopMp = std::get<0>(mtORMP);
        orbslam3_interfaces::msg::MapPoint::SharedPtr mrpMp = std::get<1>(mtORMP);
        Converter::MapPointConverter::FillMapPointData(mopMp, mrpMp, mpOrbKeyFrames, mpOrbMaps, mpOrbMapPoints, &bUnprocessed);

        if(!bUnprocessed)
        {
          InsertMapPoint(mopMp);
          it = mpUnprocOrbMapPoints.erase(it);
        } else {
          ++it;
        }
      }
      std::cout << "and after=" << mpUnprocOrbMapPoints.size() << std::endl;
    }

    if(CheckNewROSActions())
    {
      std::cout << "New ros actions..." << std::endl;
      while(CheckNewROSActions()) {
        
        int categoryIdx=mRosActions[0];
        //std::cout << "#ROS actions left=" << mRosActions.size() << ", next category=" << categoryIdx << std::endl;
        if(categoryIdx==0){   
          unique_lock<mutex> lock(mMutexNewAAs);
          
          orbslam3_interfaces::msg::AtlasActions::SharedPtr mAtlasRosMsg = mvpAtlasRosActions[0]; 
          
          if(PerformAtlasAction(mAtlasRosMsg)) 
          {
            mRosActions.erase(mRosActions.begin());
            mvpAtlasRosActions.erase(mvpAtlasRosActions.begin());
          } else {
            std::cout << "Not found from the vector, break..." << std::endl;
            break;
          }
        } else if(categoryIdx==1) { 
          unique_lock<mutex> lock(mMutexNewKFAs);
          orbslam3_interfaces::msg::KeyFrameActions::SharedPtr mKfRosMsg = mvpKfRosActions[0]; 
          
          if(PerformKeyFrameAction(mKfRosMsg)) 
          {
            mRosActions.erase(mRosActions.begin());
            mvpKfRosActions.erase(mvpKfRosActions.begin());
          } else {
            std::cout << "Not found from the vector, break..." << std::endl;
            break;
          }
        } else if(categoryIdx==2) {
          unique_lock<mutex> lock(mMutexNewMPAs);
          orbslam3_interfaces::msg::MapPointActions::SharedPtr mMpRosMsg = mvpMpRosActions[0]; 
          
          if(PerformMapPointAction(mMpRosMsg)) 
          {
            mRosActions.erase(mRosActions.begin());
            mvpMpRosActions.erase(mvpMpRosActions.begin());
          } else {
            std::cout << "Not found from the vector, break..." << std::endl;
            break;
          }
        } else if(categoryIdx==3) {
          unique_lock<mutex> lock(mMutexNewKFDBAs);
          orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr mKFDBRosMsg = mvpKFDBRosActions[0]; 
          
          if(PerformKFDBAction(mKFDBRosMsg)) 
          {
            mRosActions.erase(mRosActions.begin());
            mvpKFDBRosActions.erase(mvpKFDBRosActions.begin());
          } else {
            std::cout << "Not found from the vector, break..." << std::endl;
            break;
          }
        }
      }

    }
    
    if(mbFinish) break;
    
    usleep(1000000);
  }
}

bool ActionChecker::CheckNewKeyFrames() {
  unique_lock<mutex> lock(mMutexNewKFs);
  return (!mpUnprocOrbKeyFrames.empty());
}

bool ActionChecker::CheckNewMapPoints() {
  unique_lock<mutex> lock(mMutexNewMPs);
  return (!mpUnprocOrbMapPoints.empty());
}


bool ActionChecker::CheckNewROSActions() {
  unique_lock<mutex> lock(mMutexNewKFAs);
  return (!mRosActions.empty());
}



void ActionChecker::InsertKeyFrame(ORB_SLAM3::KeyFrame* pKF)
{
  unique_lock<mutex> lock(mMutexNewKFs);
  //std::cout << "Insert KF." << std::endl;
  mpOrbKeyFrames[pKF->mnId] = pKF;
}

void ActionChecker::InsertUnprocessedKeyFrame(ORB_SLAM3::KeyFrame* opKF, orbslam3_interfaces::msg::KeyFrame::SharedPtr rpKF) 
{
  unique_lock<mutex> lock(mMutexNewKFs);
  //std::cout << "Insert Unprosessed KF." << std::endl;
  mpUnprocOrbKeyFrames[opKF->mnId] = std::make_tuple(opKF, rpKF);
}

void ActionChecker::InsertMapPoint(ORB_SLAM3::MapPoint* pMP)
{
  unique_lock<mutex> lock(mMutexNewMPs);
  //std::cout << "Insert MapPoint." << std::endl;
  mpOrbMapPoints[pMP->mnId] = pMP;
}


void ActionChecker::InsertMap(ORB_SLAM3::Map* pM)
{
  unique_lock<mutex> lock(mMutexNewMs);
  //std::cout << "Insert Map." << std::endl;
  mpOrbMaps[pM->GetId()] = pM;
}

void ActionChecker::InsertUnprocessedMapPoint(ORB_SLAM3::MapPoint* opMP, orbslam3_interfaces::msg::MapPoint::SharedPtr rpMP)
{
  unique_lock<mutex> lock(mMutexNewMPs);
  //std::cout << "Insert Unprosessed MapPoint." << std::endl;
  mpUnprocOrbMapPoints[opMP->mnId] = std::make_tuple(opMP, rpMP);
}

void ActionChecker::InsertAtlasAction(orbslam3_interfaces::msg::AtlasActions::SharedPtr rpAA)
{
  unique_lock<mutex> lock(mMutexNewAAs);
  //std::cout << "Insert Atlas Action." << std::endl;
  mRosActions.push_back(0);
  mvpAtlasRosActions.push_back(rpAA);
}

void ActionChecker::InsertKeyFrameAction(orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rpKFA)
{
  unique_lock<mutex> lock(mMutexNewKFAs);
  //std::cout << "Insert KF Action." << std::endl;
  mRosActions.push_back(1);
  mvpKfRosActions.push_back(rpKFA);
}

void ActionChecker::InsertMapPointAction(orbslam3_interfaces::msg::MapPointActions::SharedPtr rpMPA)
{
  unique_lock<mutex> lock(mMutexNewMPAs);
  //std::cout << "Insert MapPoint Action." << std::endl;
  mRosActions.push_back(2);
  mvpMpRosActions.push_back(rpMPA);
}


void ActionChecker::InsertKFDBAction(orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr rpKFDBA)
{
  unique_lock<mutex> lock(mMutexNewKFDBAs);
  //std::cout << "Insert KFDB Action." << std::endl;
  mRosActions.push_back(3);
  mvpKFDBRosActions.push_back(rpKFDBA);
}


/*        KEYFRAME - ACTION        */
bool ActionChecker::PerformKeyFrameAction(const orbslam3_interfaces::msg::KeyFrameActions::SharedPtr rKF) {

  unique_lock<mutex> lock(mMutexNewKFs);
  int actionId = Parser::Action::parseKFAction(rKF, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);
  
  if(actionId == -1) 
    return false;
  
  //std::cout << "Got new KeyFram action=" << actionId << std::endl;
  //RCLCPP_INFO(this->get_logger(), "Got new KeyFrame action=%d", actionId);
  
  ORB_SLAM3::KeyFrame* kf_ = mpOrbKeyFrames[rKF->kf_id];
  //RCLCPP_INFO(this->get_logger(), "kf id=%d", kf_->mnId);
  
  // Change to switch statement and create enum for the actionids
  if (actionId==0)  kf_->SetPose(Converter::RosToCpp::PoseToSophusSE3f(rKF->set_pose), true);
  if (actionId==1)  kf_->SetVelocity(Converter::RosToCpp::Vector3ToEigenVector3f(rKF->set_velocity), true);
  if (actionId==2)  kf_->AddConnection(mpOrbKeyFrames[rKF->add_connection_id], rKF->add_connection_weight, true);
  if (actionId==3)  kf_->AddMapPoint(mpOrbMapPoints[rKF->add_map_point_mp_id], rKF->add_map_point_vector_idx, true);
  if (actionId==4)  kf_->EraseMapPointMatch(rKF->erase_map_point_vector_idx, true);
  if (actionId==5)  kf_->EraseMapPointMatch(mpOrbMapPoints[rKF->erase_map_point_id], true);
  if (actionId==6)  kf_->ReplaceMapPointMatch(rKF->replace_map_point_vector_idx, mpOrbMapPoints[rKF->replace_map_point_id], true);
  if (actionId==7)  kf_->AddChild(mpOrbKeyFrames[rKF->add_child_id], true);
  if (actionId==8)  kf_->EraseChild(mpOrbKeyFrames[rKF->erase_child_id], true);
  if (actionId==9)  kf_->ChangeParent(mpOrbKeyFrames[rKF->change_parent_id], true);  
  if (actionId==10) kf_->AddLoopEdge(mpOrbKeyFrames[rKF->add_loop_edge_id], true);
  if (actionId==11) kf_->AddMergeEdge(mpOrbKeyFrames[rKF->add_merge_edge_id], true);
  if (actionId==12) kf_->EraseConnection(mpOrbKeyFrames[rKF->erase_connection_id], true);
  if (actionId==13) kf_->SetNewBias(Converter::RosToOrb::RosBiasToOrbImuBias(rKF->set_new_bias), true);
  if (actionId==14) kf_->UpdateMap(mpOrbMaps[rKF->update_map_id], true);
  if (actionId==15) kf_->ComputeBoW(true);
  if (actionId==16) kf_->UpdateBestCovisibles(true);
  if (actionId==17) kf_->UpdateConnections(true, true);
  if (actionId==18) kf_->SetFirstConnection(true, true);
  if (actionId==19) kf_->SetNotErase(true);
  if (actionId==20) kf_->SetErase(true);
  if (actionId==21) kf_->SetBadFlag(true);

  return true;
}


/*        ATLAS - ACTION        */
bool ActionChecker::PerformAtlasAction(const orbslam3_interfaces::msg::AtlasActions::SharedPtr rAA) {

  unique_lock<mutex> lock(mMutexNewAAs);
  int actionId = Parser::Action::parseAtlasAction(rAA, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);
  
  if(actionId == -1) 
    return false;
  
  std::cout << "**************** Got new atlas action=" << actionId << std::endl;
  //RCLCPP_INFO(this->get_logger(), "Got new atlas action %d.", actionId);
  
  if (actionId==0)  mpAtlas_->AddMapPoint(mpOrbMapPoints[rAA->add_map_point_id], true);
  if (actionId==1)  mpAtlas_->ChangeMap(mpOrbMaps[rAA->change_map_to_id], true);
  if (actionId==2)  mpAtlas_->AddKeyFrame(mpOrbKeyFrames[rAA->add_kf_id], true);
  if (actionId==3)  mpAtlas_->SetMapBad(mpOrbMaps[rAA->set_map_bad_id], true);
  //if (actionId==4)  mpAtlas_->CreateNewMap(true);
  if (actionId==5)  mpAtlas_->InformNewBigChange(true);
  if (actionId==6)  mpAtlas_->clearMap(true);
  if (actionId==7)  mpAtlas_->clearAtlas(true);
  if (actionId==8)  mpAtlas_->RemoveBadMaps(true);
  if (actionId==9)  mpAtlas_->SetInertialSensor(true);  
  if (actionId==10) mpAtlas_->SetImuInitialized(true);
  
  return true;
}


/*    MAPPOINT - ACTION    */
bool ActionChecker::PerformMapPointAction(const orbslam3_interfaces::msg::MapPointActions::SharedPtr rMpA) {

  unique_lock<mutex> lock(mMutexNewMPs);
  int actionId = Parser::Action::parseMapPointAction(rMpA, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);
  if(actionId == -1) 
    return false;
  
  //std::cout << "Got new MapPoint action=" << actionId << std::endl;
  //RCLCPP_INFO(this->get_logger(), "Got new MapPoint action=%d", actionId);
  
  ORB_SLAM3::MapPoint* pMp = mpOrbMapPoints[rMpA->mp_id];
  //RCLCPP_INFO(this->get_logger(), "kf id=%d", kf_->mnId);
  
  // Change to switch statement and create enum for the actionids
  if (actionId==0)  pMp->SetWorldPos(Converter::RosToCpp::Vector3ToEigenVector3f(rMpA->set_pose), true);
  if (actionId==1)  pMp->AddObservation(mpOrbKeyFrames[rMpA->add_observation_kf_id], rMpA->add_observation_vector_idx, true);
  if (actionId==2)  pMp->EraseObservation(mpOrbKeyFrames[rMpA->erase_observation_kf_id], true);
  if (actionId==3)  pMp->SetBadFlag(true);
  if (actionId==4)  pMp->Replace(mpOrbMapPoints[rMpA->replace_id], true);
  if (actionId==5)  pMp->IncreaseVisible(rMpA->increase_visibility_n, true);
  if (actionId==6)  pMp->IncreaseFound(rMpA->increase_found_n, true);
  if (actionId==7)  pMp->ComputeDistinctiveDescriptors(true);
  if (actionId==8)  pMp->UpdateNormalAndDepth(true);
  if (actionId==9)  pMp->SetNormalVector(Converter::RosToCpp::Vector3ToEigenVector3f(rMpA->set_normal_vector), true);
  if (actionId==10) pMp->UpdateMap(mpOrbMaps[rMpA->update_map_id], true);  

  return true;


}


/*        KEYFRAME - ACTION        */
bool ActionChecker::PerformKFDBAction(const orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr rKfdb) {

  int actionId = Parser::Action::parseKFDBAction(rKfdb, mpOrbMaps, mpOrbKeyFrames, mpOrbMapPoints);
  if(actionId == -1) 
    return false;
  
  //RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d", actionId);
  
  if(actionId==0) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->add_kf_id];
    mpKfDb_->add(pKf, true);
  }

  if(actionId==1) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->erase_kf_id];
    mpKfDb_->erase(pKf, true);
  }

  if(actionId==2) {
    mpKfDb_->clear(true);
  }

  if(actionId==3) {
    ORB_SLAM3::Map* pMap = mpOrbMaps[rKfdb->clear_map_id]; 
    mpKfDb_->clearMap(pMap, true);
  }

  if(actionId==4) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->detect_candidates_kf_id];
    
    float minScore = rKfdb->detect_candidates_min_score;
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpLoopCands;
    for(size_t id : rKfdb->detect_candidates_loop_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpLoopCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }

    std::vector<ORB_SLAM3::KeyFrame*> mvpMergeCands;
    for(size_t id : rKfdb->detect_candidates_merge_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpMergeCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }

    mpKfDb_->DetectCandidates(pKf, minScore, mvpLoopCands, mvpMergeCands, true);
  }


  if(actionId==5) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->detect_best_candidates_kf_id];
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpLoopCands;
    for(size_t id : rKfdb->detect_best_candidates_loop_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpLoopCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }

    std::vector<ORB_SLAM3::KeyFrame*> mvpMergeCands;
    for(size_t id : rKfdb->detect_best_candidates_merge_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpMergeCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }
  
    int nMinWords = rKfdb->detect_best_candidates_n_min_words;

    mpKfDb_->DetectBestCandidates(pKf, mvpLoopCands, mvpMergeCands, nMinWords, true);
  }

  if(actionId==6) {
    ORB_SLAM3::KeyFrame* pKf = mpOrbKeyFrames[rKfdb->detect_n_best_candidates_kf_id];
    
    std::vector<ORB_SLAM3::KeyFrame*> mvpLoopCands;
    for(size_t id : rKfdb->detect_n_best_candidates_loop_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpLoopCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }

    std::vector<ORB_SLAM3::KeyFrame*> mvpMergeCands;
    for(size_t id : rKfdb->detect_n_best_candidates_merge_cand_ids) {
      if(mpOrbKeyFrames.find(id) != mpOrbKeyFrames.end()) {
        mvpMergeCands.push_back(mpOrbKeyFrames[id]);
      } else {
        return false;
      }
    }
  
    int nNumCands = rKfdb->detect_n_best_candidates_n_num_cands;

    //RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d Just before function", actionId);
    mpKfDb_->DetectNBestCandidates(pKf, mvpLoopCands, mvpMergeCands, nNumCands, true);
    //RCLCPP_INFO(this->get_logger(), "Got new KF DB action=%d Just after function", actionId);
  }

  return true;
}







