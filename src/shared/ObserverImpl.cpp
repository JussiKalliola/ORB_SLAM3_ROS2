#include "Observer.h"
//#include "KeyFrame.h"
#include "slam-publisher.hpp"

class ObserverImpl : public ORB_SLAM3::Observer {

  public:
    ObserverImpl() {
      // m_SLAM = pSLAM;
      publisher_node_ = std::make_shared<SLAMPublisher>();
    }

    void onKeyframeAdded(ORB_SLAM3::KeyFrame* kf) override {
      std::cout << "ObserverImpl : KeyFrame added." << std::endl;
    }

    void onKeyframeChanged(int keyframeId) override {
      publisher_node_->publishMessage("onKeyframeChanged function noticed difference.");
      std::cout << "This is from ObserverImpl" << std::endl;
      std::cout << keyframeId << std::endl;
    }

  private:
    // ORB_SLAM3::System* m_SLAM;
    
    std::shared_ptr<SLAMPublisher> publisher_node_;

};
