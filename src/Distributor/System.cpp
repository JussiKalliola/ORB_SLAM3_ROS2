#include "./System.hpp"


#include "./Observer.hpp"
#include "./MapHandler.hpp"
#include "./KeyFramePublisher.hpp"
#include "./KeyFrameSubscriber.hpp"
#include "../slam/slam-wrapper-node.hpp"


//namespace TempDistributor
//{

System::System() {

    // initialize all the handlers
    mpMapHandler = new MapHandler();
    mpKeyFramePublisher = new KeyFramePublisher();
    mpKeyFrameSubscriber = new KeyFrameSubscriber();

    // Attach hnadlers to observer
    std::shared_ptr<Observer> pObserver(new Observer(mpMapHandler, mpKeyFramePublisher, mpKeyFrameSubscriber)); 
    mpObserver = pObserver;
    
    // Attach observer to handlers
    if(mpObserver)
    {
      mpMapHandler->AttachObserver(mpObserver);
      mpKeyFramePublisher->AttachObserver(mpObserver);
      mpKeyFrameSubscriber->AttachObserver(mpObserver);
    }

    // Launch handler threads
    std::thread* mptMapHandler = new thread(&MapHandler::Run, mpMapHandler);
    std::thread* mptKeyFramePublisher = new thread(&KeyFramePublisher::Run, mpKeyFramePublisher);
    std::thread* mptKeyFrameSubscriber = new thread(&KeyFrameSubscriber::Run, mpKeyFrameSubscriber);

}

System::~System()
{

}

void System::LaunchThreads()
{

}
MapHandler* System::GetMapHandler()
{
  return mpMapHandler;
}

KeyFrameSubscriber* System::GetKeyFrameSubscriber()
{
  return mpKeyFrameSubscriber;
}

KeyFramePublisher* System::GetKeyFramePublisher()
{
  return mpKeyFramePublisher;
}
    


void System::AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM)
{
    pSLAM = mSLAM;

    // Attach node to the others
    mpObserver->AttachORBSLAMSystem(mSLAM);
    mpMapHandler->AttachORBSLAMSystem(mSLAM);
    mpKeyFramePublisher->AttachORBSLAMSystem(mSLAM);
    mpKeyFrameSubscriber->AttachORBSLAMSystem(mSLAM);
}

void System::AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node)
{
    pSLAMNode = slam_node;

    // Attach node to the others
    mpObserver->AttachSLAMNode(slam_node);
    mpMapHandler->AttachSLAMNode(slam_node);
    mpKeyFramePublisher->AttachSLAMNode(slam_node);
}


std::shared_ptr<Observer> System::GetObserver()
{
  return mpObserver;
}

//} // namespace TempDistributor
