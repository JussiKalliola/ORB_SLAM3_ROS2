#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <locale> // For std::tolower
#include <cstdlib> // For exit function

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = stamp.sec + (stamp.nanosec * pow(10,-9));
    return seconds;
  }

  template<typename T>
  static void CompareTwoSerializedObjects(const T* pObj1, const T* pObj2)
  {

    // save data to archive
    {
        std::string msg;
        std::ostringstream os;
        boost::archive::text_oarchive oa(os);
        // write class instance to archive
        oa << pObj1;
        msg = os.str();
      // archive and stream closed when destructors are called
        std::ostringstream os2;
        boost::archive::text_oarchive oa2(os2);
        // write class instance to archive
        oa2 << pObj2;
        std::string msg2;
        msg2 = os2.str();
        std::cout << "Obj1.compare(Obj2) --------------------------- "<< msg.compare(msg2) << std::endl;
        std::cout << std::endl;
        std::cout << msg << std::endl;
        std::cout << msg2 << std::endl;
        std::cout << std::endl;
    }
    exit(1);
  }

  // Works only on Linux
  static bool createDirectory(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
      if (mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) {
        std::cout << "Directory created: " << path << std::endl;
        return true;
      } else {
        std::cerr << "Failed to create directory: " << path << std::endl;
        return false;
      }
    } else if (info.st_mode & S_IFDIR) {
      std::cout << "Directory already exists: " << path << std::endl;
      return true;
    } else {
      std::cerr << "Not a directory: " << path << std::endl;
      return false;
    }
  }
  
  // Check the user given visualization parameter (can be used for others)
  static bool checkTrueFalse(const std::string& str) {
    std::string lowercaseStr;
    
    for (char ch : str) {
      lowercaseStr += std::tolower(ch); // Convert the string to lowercase
    }

    if (lowercaseStr == "true") {
      return true; // Return 0 for true
    } else if (lowercaseStr == "false") {
      return false; // Return 0 for false
    } else {
      cout << "Parameter is not boolean; exit..." << endl;
      exit(1);
    }
  }

};

#endif
