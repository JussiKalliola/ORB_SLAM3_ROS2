#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"

#include "../shared/ObserverImpl.cpp"


int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings path_to_results visualization" << std::endl;
        exit(1);
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
    
    rclcpp::init(argc, argv); 

    auto observer_impl_ = std::make_shared<ObserverImpl>();
    
    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization, strSaveToPath, observer_impl_);

    // SLAM.attachObserver(observer_impl_);
    
    auto publisher_node_ = std::make_shared<SLAMPublisher>();
    
    auto node = std::make_shared<MonocularSlamNode>(&SLAM, strSaveToPath, publisher_node_);
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
