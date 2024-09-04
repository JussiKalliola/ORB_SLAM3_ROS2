#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"

#include "../Distributor/Observer.hpp"
#include "../Distributor/System.hpp"

#include "../slam/slam-wrapper-node.hpp"
//#include "../slam/Distributor.hpp"


std::shared_ptr<SlamWrapperNode> slam_node;

void signalHandler(int signum) {
    if (signum == SIGINT) {
      slam_node->publishEndMsg();
      //keepRunning = false;
      rclcpp::shutdown(); // Ensure proper shutdown of ROS nodes
    }
}



int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings path_to_results visualization [OPTIONAL] result_file_name [OPTIONAL] subscribe [OPTIONAL] main [OPTIONAL]" << std::endl;
        exit(1);
    }

    string strResultFileName = "KeyFrameTrajectory.txt";
    bool visualization = false;
    bool subscribe_to_slam = true;
    bool main_system = true;
    bool mbOnlyTrack = false;
    // Check visualization 
    if (argc > 4)
    {
      visualization = Utility::checkTrueFalse(argv[4]);
      // Check if user has provided filename for the result file
      if (argc > 5)
      {
        strResultFileName = argv[5];
        // Check for subscription to slam system
        if (argc > 6)
        {
          subscribe_to_slam = Utility::checkTrueFalse(argv[6]);
          if (argc > 7) {
            main_system = Utility::checkTrueFalse(argv[7]);
          }
        }
      }
    }

    if(main_system)
      mbOnlyTrack=false;

    
    

    string strSaveToPath = argv[3]; 
    // First create the base directory and then sub directory
    if (Utility::createDirectory(strSaveToPath)) {
      if (Utility::createDirectory(strSaveToPath + "monocular/")) {
        if (!Utility::createDirectory(strSaveToPath + "monocular/" + "stats/")) 
            exit(1);
      } else {
        // Failer to create directory or it doesnt exists
        exit(1);

      }
    } else {
      // Failer to create directory or it doesnt exists
      exit(1);
    }

    strSaveToPath += "monocular/";


    char* systemId = std::getenv("SLAM_SYSTEM_ID");
    if(!systemId) {
        std::random_device rd;  // Obtain a random number from hardware
        std::mt19937 gen(rd()); // Seed the generator
        std::uniform_int_distribution<> distr(1, 100); // Define the range 
        setenv("SLAM_SYSTEM_ID", std::to_string(distr(gen)).c_str(),1);
        systemId = std::getenv("SLAM_SYSTEM_ID");
    }
    
    std::cout << "\n===================" << std::endl; 
    std::cout << "Given parameters" << std::endl; 
    std::cout << " - Results path=" + strSaveToPath + strResultFileName << std::endl;
    std::cout << " - Visualization=" << visualization << std::endl;
    std::cout << " - Subscribe to SLAM=" << subscribe_to_slam << std::endl;
    std::cout << " - Voc path=" << argv[1] << std::endl; 
    std::cout << " - Settings path=" << argv[2] << std::endl;
    std::cout << " - Main system=" << main_system << std::endl;
    std::cout << " - SLAM_SYSTEM_ID=" << systemId << std::endl;
    std::cout << "===================\n" << std::endl;


    std::signal(SIGINT, signalHandler);
    rclcpp::init(argc, argv); 

    std::string mStatSavePath=strSaveToPath+"stats/"; 
    std::shared_ptr<System> mpDistSystem= std::make_shared<System>(mStatSavePath);
    std::shared_ptr<Observer> mpDistObserver = mpDistSystem->GetObserver();
    //std::shared_ptr<Distributor> mpDistributor = std::make_shared<Distributor>();

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization, strSaveToPath, mpDistObserver, mbOnlyTrack);

    slam_node = std::make_shared<SlamWrapperNode>(&SLAM, mpDistSystem, subscribe_to_slam, strSaveToPath, strResultFileName); 

    // Start distribution thread
    //std::thread* mptDistribution = new thread(&Distributor::Run,mpDistributor);

    if(mpDistSystem)
    {
        mpDistSystem->AttachORBSLAMSystem(&SLAM);
        mpDistSystem->AttachSLAMNode(slam_node);
        mpDistSystem->LaunchThreads();
    }

    //if(mpDistributor != nullptr) 
    //{
    //  mpDistributor->AttachSLAMNode(slam_node);
    //  mpDistributor->AttachSLAMSystem(&SLAM);
    //}

    // If this system needs to subscribe to sensor data stream.
    if(main_system) {
      auto mono_node = std::make_shared<MonocularSlamNode>(&SLAM, slam_node, strSaveToPath, strResultFileName);
      rclcpp::executors::MultiThreadedExecutor multiThreadExecutor;
      
      multiThreadExecutor.add_node(mono_node);
      multiThreadExecutor.add_node(slam_node);

      // Spin monocular slam node
      //std::thread spin_mono([&]() {
      //  rclcpp::spin(mono_node);
      //});
      //
      //// Spin SLAM publisher&Subscriber
      //std::thread spin_slam([&]() {
      //  rclcpp::spin(slam_node);
      //});
      //
      //// Join threads
      //spin_mono.join();
      //spin_slam.join();
      
      multiThreadExecutor.spin();
    } else {
      rclcpp::spin(slam_node);
    }

    //slam_node_->publishEndMsg();
    //rclcpp::sleep_for(std::chrono::seconds(1));
    // Stop all threads
    // Save camera trajectory
    std::cout << "Saving data to the path=" << strSaveToPath  << strResultFileName << std::endl;
    SLAM.SaveKeyFrameTrajectoryTUM(strSaveToPath + strResultFileName);
    SLAM.SaveTrajectoryEuRoC(strSaveToPath + "Trajectory-" + systemId + ".txt");
    std::cout << "Shutdown SLAM System" << std::endl;
    SLAM.Shutdown();
    mpDistSystem->ShutDown();
    
    
    //rclcpp::shutdown();

    return 0;
}
