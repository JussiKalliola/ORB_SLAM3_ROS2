#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary[string] path_to_settings[string] path_to_results[string] do_rectify[????] visualization [boolean]" << std::endl;
        return 1;
    }

    // Check if results folder exists
    if (!Utility::createDirectory(argv[3])) {
      // Failer to create directory or it doesnt exists
      exit(1);
    }
    

    bool visualization = Utility::checkTrueFalse(argv[5]);
    
    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&pSLAM, argv[2], argv[4], argv[3]);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
