#include "Observer.h"
//#include "Atlas.h"
//#include "KeyFrame.h"
//#include <cv_bridge/cv_bridge.h>

#include "../src/slam/slam-wrapper-node.hpp"


class ObserverImpl : public ORB_SLAM3::Observer {

  public:
    ObserverImpl() {
      //publisher_node_ = std::make_shared<SlamWrapperNode>();
    }

    ~ObserverImpl() {
      std::cout << "ObserverImpl Destructor." << std::endl;
      slam_node_ = nullptr;
    };

    //void onMapAdded(ORB_SLAM3::Map* pM) override {
    //  std::cout << "ObserverImpl : Map added." << std::endl;
    //  slam_node_->publishMap(pM); 
    //}

    /* KEYFRAME ACTION FUNCTIONS*/
    void onKFAction(unsigned long int hostKfId, int actionId, unsigned long int id) {
      orbslam3_interfaces::msg::KeyFrameActions::SharedPtr kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, id);
      kfAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg->kf_id = hostKfId; 

      //slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(unsigned long int hostKfId, int actionId, bool boolAction) {
      orbslam3_interfaces::msg::KeyFrameActions::SharedPtr kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, boolAction);
      kfAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg->kf_id = hostKfId; 
      
      //slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(unsigned long int hostKfId, int actionId, unsigned long int id, long int vectorIdx) {
      orbslam3_interfaces::msg::KeyFrameActions::SharedPtr kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, id, vectorIdx);
      kfAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg->kf_id = hostKfId; 
      
      //slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(unsigned long int hostKfId, int actionId, Eigen::Vector3f t) {
      orbslam3_interfaces::msg::KeyFrameActions::SharedPtr kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, t);
      kfAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg->kf_id = hostKfId; 
      
      //slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(unsigned long int hostKfId, int actionId, Sophus::SE3<float> p) {
      orbslam3_interfaces::msg::KeyFrameActions::SharedPtr kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, p);
      kfAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg->kf_id = hostKfId; 
      
      //slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(unsigned long int hostKfId, int actionId, ORB_SLAM3::IMU::Bias b) {
      orbslam3_interfaces::msg::KeyFrameActions::SharedPtr kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, b);
      kfAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg->kf_id = hostKfId; 
      
      //slam_node_->publishKFAction(kfAMsg);
    }
    



    /* ATLAS ACTION FUNCTIONS */
    void onAtlasAction(int actionId, unsigned long int id) override {
      orbslam3_interfaces::msg::AtlasActions::SharedPtr aMsg = Parser::Action::FormAtlasActionRosMsg(actionId, id);
      aMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
    
      //slam_node_->publishAtlasAction(aMsg);
    }

    void onAtlasAction(int actionId, bool boolAction) override {
      orbslam3_interfaces::msg::AtlasActions::SharedPtr aMsg = Parser::Action::FormAtlasActionRosMsg(actionId, boolAction);
      aMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
    
      //slam_node_->publishAtlasAction(aMsg);
    }





    /* KFDB ACTION FUNCTIONS */
    void onKFDBAction(int actionId, bool boolAction) 
    {
      orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr kfdbMsg = Parser::Action::FormKFDBActionRosMsg(actionId, boolAction);
      kfdbMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      
      //slam_node_->publishKFDBAction(kfdbMsg);
    }

    void onKFDBAction(int actionId, unsigned long int id)
    {
      orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr kfdbMsg = Parser::Action::FormKFDBActionRosMsg(actionId, id);
      kfdbMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      
      //slam_node_->publishKFDBAction(kfdbMsg);
    }

    void onKFDBAction(int actionId, unsigned long int id, float minScore, std::vector<unsigned long int> vpLoopCandId, std::vector<unsigned long int> vpMergeCandId)
    {
      orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr kfdbMsg = Parser::Action::FormKFDBActionRosMsg(actionId, id, minScore, vpLoopCandId, vpMergeCandId);
      kfdbMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      
      //slam_node_->publishKFDBAction(kfdbMsg);
    }
    
    void onKFDBAction(int actionId, unsigned long int id, std::vector<unsigned long int> vpLoopCandId, std::vector<unsigned long int> vpMergeCandId, int n)
    {
      orbslam3_interfaces::msg::KeyFrameDatabaseActions::SharedPtr kfdbMsg = Parser::Action::FormKFDBActionRosMsg(actionId, id, vpLoopCandId, vpMergeCandId, n);
      kfdbMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      
      //slam_node_->publishKFDBAction(kfdbMsg);
    }








    /* MAPPOINT ACTION FUNCTIONS */
    void onMapPointAction(unsigned long int hostMpId, int actionId, bool boolAction )
    {
      orbslam3_interfaces::msg::MapPointActions::SharedPtr mpAMsg = Parser::Action::FormMapPointActionRosMsg(actionId, boolAction);
      mpAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      mpAMsg->mp_id = hostMpId; 
      
      //slam_node_->publishMapPointAction(mpAMsg);
    }

    void onMapPointAction(unsigned long int hostMpId, int actionId, unsigned long int id, int idx)
    {
      orbslam3_interfaces::msg::MapPointActions::SharedPtr mpAMsg = Parser::Action::FormMapPointActionRosMsg(actionId, id, idx);
      mpAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      mpAMsg->mp_id = hostMpId; 
      
      //slam_node_->publishMapPointAction(mpAMsg);
    }

    void onMapPointAction(unsigned long int hostMpId, int actionId, unsigned long int id)
    {
      orbslam3_interfaces::msg::MapPointActions::SharedPtr mpAMsg = Parser::Action::FormMapPointActionRosMsg(actionId, id);
      mpAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      mpAMsg->mp_id = hostMpId; 
      
      //slam_node_->publishMapPointAction(mpAMsg);
    }
    
    void onMapPointAction(unsigned long int hostMpId, int actionId, int n)
    {
      orbslam3_interfaces::msg::MapPointActions::SharedPtr mpAMsg = Parser::Action::FormMapPointActionRosMsg(actionId, n);
      mpAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      mpAMsg->mp_id = hostMpId; 
      
      //slam_node_->publishMapPointAction(mpAMsg);
    }

    void onMapPointAction(unsigned long int hostMpId, int actionId, Eigen::Vector3f vec)
    {
      orbslam3_interfaces::msg::MapPointActions::SharedPtr mpAMsg = Parser::Action::FormMapPointActionRosMsg(actionId, vec);
      mpAMsg->system_id = std::getenv("SLAM_SYSTEM_ID"); 
      mpAMsg->mp_id = hostMpId; 
      
      //slam_node_->publishMapPointAction(mpAMsg);
    }








    void onChangeLMActive(bool bActive) override {
      if(slam_node_) {
        slam_node_->publishLMActivityChange(bActive);
      } 
    }

    void onMapAddedById(unsigned long int id) override {
      std::cout << "ObserverImpl : Map added by id " << id << "." << std::endl;
      //slam_node_->publishNewMapId(id); 
    }

    void onKeyframeAdded(ORB_SLAM3::KeyFrame* kf) override {
      if(slam_node_) {
        slam_node_->publishKeyFrame(kf);
      }
    }

    void onMapPointAdded(ORB_SLAM3::MapPoint* pMp) override {
      if(slam_node_) {
        slam_node_->publishMapPoint(pMp);
      }
    }
    
    void onLocalMapUpdated(ORB_SLAM3::Map* pM) override {
      std::mutex mMutexNewMP;
      std::lock_guard<std::mutex> lock(mMutexNewMP);
      
      if(slam_node_) {
        slam_node_->publishMap(pM);
      }
    }

    void onKeyframeChanged(int keyframeId) override {
      //publisher_node_->publishMessage("onKeyframeChanged function noticed difference.");

      // std::cout << "This is from ObserverImpl" << std::endl;
      // std::cout << keyframeId << std::endl;
    }

    void attachSlamNode(std::shared_ptr<SlamWrapperNode> slam_node) {
      slam_node_ = slam_node;
    }

    void deleteSlamNode(std::shared_ptr<SlamWrapperNode> slam_node) {
      slam_node_.reset();
    }
  private:
    std::shared_ptr<SlamWrapperNode> slam_node_;
};
