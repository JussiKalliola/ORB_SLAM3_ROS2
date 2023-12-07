#include "Observer.h"
#include "Atlas.h"
//#include "KeyFrame.h"
#include "../src/slam/slam-wrapper-node.hpp"


class ObserverImpl : public ORB_SLAM3::Observer {

  public:
    ObserverImpl() {
      //publisher_node_ = std::make_shared<SlamWrapperNode>();
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
