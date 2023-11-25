#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 6)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary[string] path_to_settings[string] save_to_path[string] do_rectify[boolean] [do_equalize][???] visualization[boolean]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    string strSaveToPath = argv[3]; 
    // First create the base directory and then sub directory
    if (Utility::createDirectory(strSaveToPath)) {
      if (!Utility::createDirectory(strSaveToPath + "stereo-inertial/")) {
        // Failer to create directory or it doesnt exists
        exit(1);
      }
    } else {
      // Failer to create directory or it doesnt exists
      exit(1);
    }

    strSaveToPath += "stereo-inertial";
    

    bool visualization = Utility::checkTrueFalse(argv[6]);
    bool do_equalize = Utility::checkTrueFalse(argv[4]);
    bool do_rectify = Utility::checkTrueFalse(argv[5]);

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, visualization);

    auto node = std::make_shared<StereoInertialNode>(&pSLAM, argv[2], do_rectify, do_equalize, strSaveToPath);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
