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


    void onActiveMapReset(unsigned long int mnMapId) override {
      std::cout << "Observerimpl - onActiveMapReset" << std::endl;
      if(slam_node_) {
        slam_node_->publishResetActiveMap(mnMapId);
      } 
    }

    void onLMResetRequested() override {
      std::cout << "Observerimpl - onLMResetRequested" << std::endl;
      if(slam_node_) {
        slam_node_->publishLMResetRequested();
      } 
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
