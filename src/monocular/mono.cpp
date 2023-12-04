#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"

#include "../slam/slam-wrapper-node.hpp"
#include "../shared/ObserverImpl.cpp"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings path_to_results visualization result_file_name" << std::endl;
        exit(1);
    }

    string strResultFileName = "KeyFrameTrajectory.txt";

    // Check if user has provided filename for the result file
    if (argc >= 5)
    {
      strResultFileName = argv[5];
    }
   
    

    string strSaveToPath = argv[3]; 
    // First create the base directory and then sub directory
    if (Utility::createDirectory(strSaveToPath)) {
      if (!Utility::createDirectory(strSaveToPath + "monocular/")) {
        // Failer to create directory or it doesnt exists
        exit(1);
      }
    } else {
      // Failer to create directory or it doesnt exists
      exit(1);
    }

    strSaveToPath += "monocular/";


    bool visualization = Utility::checkTrueFalse(argv[4]);
    
    std::cout << "===================" << std::endl; 
    std::cout << "Given parameters" << std::endl; 
    std::cout << " - Results path=" + strSaveToPath + strResultFileName << std::endl;
    std::cout << " - Visualization=" + visualization << std::endl;
    std::cout << std::string(" - Voc path=") + argv[1] << std::endl; 
    std::cout << std::string(" - Settings path=") + argv[2] << std::endl; 
    std::cout << "===================" << std::endl;

    rclcpp::init(argc, argv); 

    auto observer_impl_ = std::make_shared<ObserverImpl>();
    
    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization, strSaveToPath, observer_impl_);

    auto slam_node = std::make_shared<SlamWrapperNode>(&SLAM); 
    auto mono_node = std::make_shared<MonocularSlamNode>(&SLAM, strSaveToPath, strResultFileName);

    observer_impl_->attachSlamNode(slam_node);
   
    // Spin monocular slam node
    std::thread spin_mono([&]() {
      rclcpp::spin(mono_node);
    });
    
    // Spin SLAM publisher&Subscriber
    std::thread spin_slam([&]() {
      rclcpp::spin(slam_node);
    });

    // Join threads
    spin_mono.join();
    spin_slam.join();

    rclcpp::shutdown();

    return 0;
}
