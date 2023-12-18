#include "Observer.h"
//#include "Atlas.h"
//#include "KeyFrame.h"
#include "../src/slam/slam-wrapper-node.hpp"


class ObserverImpl : public ORB_SLAM3::Observer {

  public:
    ObserverImpl() {
      //publisher_node_ = std::make_shared<SlamWrapperNode>();
    }

    ~ObserverImpl() {

    };

    //void onMapAdded(ORB_SLAM3::Map* pM) override {
    //  std::cout << "ObserverImpl : Map added." << std::endl;
    //  slam_node_->publishMap(pM); 
    //}

    /* KEYFRAME ACTION FUNCTIONS*/
    void onKFAction(unsigned long int hostKfId, int actionId, unsigned long int id) {
      orbslam3_interfaces::msg::KeyFrameActions kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, id);
      kfAMsg.system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg.kf_id = hostKfId; 

      slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(unsigned long int hostKfId, int actionId, bool boolAction) {
      orbslam3_interfaces::msg::KeyFrameActions kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, boolAction);
      kfAMsg.system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg.kf_id = hostKfId; 
      
      slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(unsigned long int hostKfId, int actionId, unsigned long int id, long int vectorIdx) {
      orbslam3_interfaces::msg::KeyFrameActions kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, id, vectorIdx);
      kfAMsg.system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg.kf_id = hostKfId; 
      
      slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(int actionId, Eigen::Vector3f t) {
      orbslam3_interfaces::msg::KeyFrameActions kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, t);
      kfAMsg.system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg.kf_id = hostKfId; 
      
      slam_node_->publishKFAction(kfAMsg);
    }

    void onKFAction(unsigned long int hostKfId, int actionId, Sophus::SE3<float> p) {
      orbslam3_interfaces::msg::KeyFrameActions kfAMsg = Parser::Action::FormKFActionRosMsg(actionId, p);
      kfAMsg.system_id = std::getenv("SLAM_SYSTEM_ID"); 
      kfAMsg.kf_id = hostKfId; 
      
      slam_node_->publishKFAction(kfAMsg);
    }

    /* ATLAS ACTION FUNCTIONS */
    void onAtlasAction(int actionId, unsigned long int id) override {
      orbslam3_interfaces::msg::AtlasActions aMsg = Parser::Action::FormAtlasActionRosMsg(actionId, id);
      aMsg.system_id = std::getenv("SLAM_SYSTEM_ID"); 
    
      slam_node_->publishAtlasAction(aMsg);
    }

    void onAtlasAction(int actionId, bool boolAction) override {
      orbslam3_interfaces::msg::AtlasActions aMsg = Parser::Action::FormAtlasActionRosMsg(actionId, boolAction);
      aMsg.system_id = std::getenv("SLAM_SYSTEM_ID"); 
    
      slam_node_->publishAtlasAction(aMsg);
    }
    //void onAtlasActionAddKeyFrame(unsigned long int kfId) override {
    //  orbslam3_interfaces::msg::AtlasActions aMsg;
    //  aMsg.system_id = std::getenv("SLAM_SYSTEM_ID");
    //  aMsg.add_kf_id = kfId;
    //
    //  slam_node_->publishAtlasAction(aMsg);
    //}

    //void onAtlasActionAddMapPoint(unsigned long int mpId) override {
    //  orbslam3_interfaces::msg::AtlasActions aMsg;
    //  aMsg.system_id = std::getenv("SLAM_SYSTEM_ID");
    //  aMsg.add_map_point_id = mpId;
    //  aMsg.change_map_to_id = -1; 
    //  aMsg.add_kf_id = -1;
    //  aMsg.set_map_bad_id = -1;
    //
    //  slam_node_->publishAtlasAction(aMsg);
    //}

    void onMapAddedById(unsigned long int id) override {
      std::cout << "ObserverImpl : Map added by id " << id << "." << std::endl;
      //slam_node_->publishNewMapId(id); 
    }

    void onKeyframeAdded(ORB_SLAM3::KeyFrame* kf) override {
      std::cout << "ObserverImpl : KeyFrame added." << std::endl;
      slam_node_->publishKeyFrame(kf); 
    }

    void onKeyframeChanged(int keyframeId) override {
      //publisher_node_->publishMessage("onKeyframeChanged function noticed difference.");

      // std::cout << "This is from ObserverImpl" << std::endl;
      // std::cout << keyframeId << std::endl;
    }

    void attachSlamNode(std::shared_ptr<SlamWrapperNode> slam_node) {
      slam_node_ = slam_node;
    }

  private:
    std::shared_ptr<SlamWrapperNode> slam_node_;
};
