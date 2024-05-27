
#ifndef CONVERTER_HPP_
#define CONVERTER_HPP_

#include "orbslam3_interfaces/msg/key_frame.hpp"
#include "orbslam3_interfaces/msg/key_frame_update.hpp"
#include "orbslam3_interfaces/msg/frame.hpp"
#include "orbslam3_interfaces/msg/atlas.hpp"
#include "orbslam3_interfaces/msg/key_frame_database.hpp"
#include "orbslam3_interfaces/msg/map.hpp"
#include "orbslam3_interfaces/msg/map_point.hpp"
#include "orbslam3_interfaces/msg/map_point_update.hpp"
#include "orbslam3_interfaces/msg/imu_bias.hpp"
#include "orbslam3_interfaces/msg/imu_preintegrated.hpp"
#include "orbslam3_interfaces/msg/matrix.hpp"
#include "orbslam3_interfaces/msg/int_tuple.hpp"
#include "orbslam3_interfaces/msg/key_value_pair.hpp"
#include "orbslam3_interfaces/msg/bow_feature_vector.hpp"
#include "orbslam3_interfaces/msg/bow_vector.hpp"
#include "orbslam3_interfaces/msg/grid3_d.hpp"
#include "orbslam3_interfaces/msg/key_point.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

namespace Converter {
  using geometry_pose = geometry_msgs::msg::Pose; 
  using geometry_vector3 = geometry_msgs::msg::Vector3;
  //using geometry_pose2d = geometry_msgs::msg::Pose2D;
  using sensor_image = sensor_msgs::msg::Image;
  using std_header = std_msgs::msg::Header;

  using keyframe = orbslam3_interfaces::msg::KeyFrame;
  using frame = orbslam3_interfaces::msg::Frame;
  using atlas = orbslam3_interfaces::msg::Atlas;
  using map_point = orbslam3_interfaces::msg::MapPoint;
  using map = orbslam3_interfaces::msg::Map;
  using imu_bias = orbslam3_interfaces::msg::IMUBias;
  using imu_preintegrated = orbslam3_interfaces::msg::IMUPreintegrated;
  using ros_matrix = orbslam3_interfaces::msg::Matrix;
  using int_tuple = orbslam3_interfaces::msg::IntTuple;
  using key_value_pair = orbslam3_interfaces::msg::KeyValuePair;
  using bow_feature_vector = orbslam3_interfaces::msg::BowFeatureVector;
  using bow_vector = orbslam3_interfaces::msg::BowVector;
  using grid_3d = orbslam3_interfaces::msg::Grid3D;
  using keypoint = orbslam3_interfaces::msg::KeyPoint;

  using orb_keyframe = ORB_SLAM3::KeyFrame;
  using orb_map_point = ORB_SLAM3::MapPoint;
  using orb_map = ORB_SLAM3::Map;
  
  class Utility {
    public:
      static string type2str(int type) {
        string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) {
          case CV_8U:  r = "8U"; break;
          case CV_8S:  r = "8S"; break;
          case CV_16U: r = "16U"; break;
          case CV_16S: r = "16S"; break;
          case CV_32S: r = "32S"; break;
          case CV_32F: r = "32F"; break;
          case CV_64F: r = "64F"; break;
          default:     r = "User"; break;
        }

        r += "C";
        r += (chans+'0');

        return r;
      }






      //
      //template <int Rows, int Cols>
      //struct MatrixTypeConverter {

      //  static ros_matrix EigenMatrixToROS(Eigen::Matrix<float, Rows, Cols> M) {
      //    ros_matrix msgM;    

      //    int rows = M.rows();
      //    int columns = M.cols();
      //    int size = rows*columns;
      //    float flattenedArray[size];
      //    
      //    std::vector<float> flattenedVector(M.data(), M.data() + size);

      //    
      //    msgM.rows = rows;
      //    msgM.columns = columns;
      //    msgM.data = flattenedVector;

      //    return msgM;
      //  }
      //};



  };

  class CppToRos {
    public:

      static grid_3d VectorToGrid3D(const std::vector<std::vector<std::vector<size_t>>>& g) {
        grid_3d msgG;

        msgG.width = g.size();
        //msgG.height = g[0].size();
        //msgG.depth = g[0][0].size();

        //std::cout << "mGrid width=" << msgG.width << ", height=" << msgG.height << ", depth=" << msgG.depth << std::endl;
        
        for (const auto& vec1 : g) {
          msgG.height.push_back(vec1.size());  
          for (const auto& vec2 : vec1) {
                //std::cout << "vec1.size()=" << vec1.size()  << ", vec2.size()=" << vec2.size() << std::endl;
                msgG.depth.push_back(vec2.size());  
                for (const auto& val : vec2) {
                    msgG.data.push_back(val);
                }
            }
        }
        return msgG;
      }
    
      static geometry_pose SophusSE3fToPose(const Sophus::SE3f& sP) {
        geometry_pose rP;
        
        rP.position.x = sP.translation()[0];
        rP.position.y = sP.translation()[1];
        rP.position.z = sP.translation()[2];

        rP.orientation.x = sP.unit_quaternion().x();
        rP.orientation.y = sP.unit_quaternion().y();
        rP.orientation.z = sP.unit_quaternion().z();
        rP.orientation.w = sP.unit_quaternion().w();

        return rP;
      }
    

      static geometry_vector3 EigenVector3fToVector3(const Eigen::Vector3f& T) {
        geometry_vector3 rT;

        rT.x = T.x();
        rT.y = T.y();
        rT.z = T.z();

        return rT;
      }


      static std::vector<keypoint> CVKeypointVectorToPose2DVector(const std::vector<cv::KeyPoint>& kps) {
        std::vector<keypoint> rKps;

        for (const auto& kp : kps) {
          keypoint rKp;
          
          rKp.x = kp.pt.x;
          rKp.y = kp.pt.y;
          rKp.size = kp.size;
          rKp.angle = kp.angle;
          rKp.response = kp.response;
          rKp.octave = kp.octave;
          rKp.class_id = kp.class_id;

          rKps.push_back(rKp);
        }

        return rKps;
      }

      static ros_matrix EigenMatrix3ToMatrix(const Eigen::Matrix3f& M) {
        ros_matrix msgM;    

        int rows = M.rows();
        int columns = M.cols();
        int size = rows*columns;
        float flattenedArray[size];
        
        std::vector<float> flattenedVector(M.data(), M.data() + size);

        
        msgM.rows = rows;
        msgM.columns = columns;
        msgM.data = flattenedVector;

        return msgM;
      }
    
      static sensor_image CVMatToImage(const cv::Mat& M) {
        cv_bridge::CvImage Ib;
        sensor_image I;
        
        Ib = cv_bridge::CvImage(std_header(), Utility::type2str(M.type()), M);
        Ib.toImageMsg(I);//const cv::Mat mDescriptors;
        
        return I;
      }

      static std::vector<int_tuple> MapToRosIntTupleVector(const std::map<long unsigned int, int>& map) {
        std::vector<int_tuple> msgIt;

        for (const auto& entry : map) {
          int_tuple t;
          t.x1 = entry.first; //std::get<0>(entry.second);
          t.x2 = entry.second; //std::get<1>(entry.second);

          msgIt.push_back(t);
        }

        return msgIt;
      }


      static std::vector<key_value_pair> MapToRosKeyValuePairVector(const std::map<orb_keyframe*, std::tuple<int,int>>& map) {
        std::vector<key_value_pair> msgKvp;

        for (const auto& entry : map) {
          key_value_pair kvp;
          int_tuple t;
          kvp.key = entry.first->mnId;
          t.x1 = std::get<0>(entry.second);
          t.x2 = std::get<1>(entry.second);

          kvp.value = t;
          msgKvp.push_back(kvp);
        }

        return msgKvp;
      }

    private:

      
  };

  class RosToCpp {
    public:

      static std::vector<std::vector<std::vector<size_t>>> Grid3DToVector(const grid_3d& rG) {
        
        unsigned int width = rG.width;
         
        std::vector<size_t> data = rG.data;
        //std::cout << "ROS2 Grid to Cpp Grid"<< std::endl;
        //std::cout << "Rows=" << rG.width << std::endl;
        //std::cout << ", Columns=";
        //for (const auto& height : rG.height)
        //{
        //  std::cout << height << ", ";
        //}
        //std::cout << " Total=" << rG.height.size() << std::endl;
        //std::cout << "Depths=";
        //for(const auto& depth : rG.depth)
        //{
        //  std::cout << depth << ", ";
        //}
        //std::cout << " Total=" << rG.depth.size() << std::endl;
        //std::cout << "Data=";
        //for(const auto& val : data)
        //{
        //  std::cout << val << ", ";
        //}
        //std::cout << std::endl;

        //std::cout << "mGrid width=" << width << ", height=" << height << ", depth=" << depth << std::endl;
        // Initialize the 3D matrix with zeros
        //std::vector<std::vector<std::vector<size_t>>> mGrid(
        //    0, std::vector<std::vector<size_t>>(
        //               0, std::vector<size_t>(0, 0)));

        // Iterate through the flattened array and fill the matrix
        // Initialize the 3D matrix with zeros
        std::vector<std::vector<std::vector<size_t>>> mGrid; 
        // Iterate through the flattened array and fill the matrix
        size_t index = 0;
        size_t depthIdx = 0;
        for (size_t x = 0; x < width; ++x)
        {
          //std::cout << ", x=" << x;
          unsigned int height = rG.height[x];
          std::vector<std::vector<size_t>> matrix;
          for (size_t y = 0; y < height; ++y)
          {
            //std::cout << ", y=" << y;
            unsigned int depth = rG.depth[depthIdx];
            //std::cout << ", depth=" << depth;
            std::vector<size_t> row(depth, 0);
            //mGrid[x][y].push_back(std::vector<size_t>());
            for (size_t z = 0; z < depth; ++z)
            {
              //std::cout << ", z=" << z;
              //std::cout << ", data=" << data[index++];
              //std::cout << data[index++];
              row[z] = data[index];
              index++;
              //mGrid[x][y][z] = data[index++];
            }
            depthIdx++;
            matrix.push_back(row);

          }
          mGrid.push_back(matrix);
        }

        //std::cout << std::endl;
        
        return mGrid;
      }
    
      static Sophus::SE3f PoseToSophusSE3f(const geometry_pose& rP) {
        Eigen::Quaternionf q = Eigen::Quaternionf(rP.orientation.w, rP.orientation.x, rP.orientation.y, rP.orientation.z);
        Eigen::Vector3f t = Eigen::Vector3f(rP.position.x, rP.position.y, rP.position.z);
        Sophus::SE3f sopP(q, t);
        
        return sopP;
      }
    

      static Eigen::Vector3f Vector3ToEigenVector3f(const geometry_vector3& rV) {
        Eigen::Vector3f eiV = Eigen::Vector3f(rV.x, rV.y, rV.z);
        return eiV;
      }


      static std::vector<cv::KeyPoint> KeypointVectorToCVKeypointVector(const std::vector<keypoint>& rKPs) {
        std::vector<cv::KeyPoint> cppKPs;

        for (const auto& rKP : rKPs) {
          cv::KeyPoint cvKP = cv::KeyPoint(rKP.x, rKP.y, rKP.size, rKP.angle, rKP.response, rKP.octave, rKP.class_id);
          cppKPs.push_back(cvKP);
        }

        return cppKPs;
      }

      static Eigen::Matrix3f MatrixToEigenMatrix3(const ros_matrix& rM) {
          
        std::vector<float> values = rM.data;

        // The Eigen::Map allows direct mapping of the vector's data to the matrix without a copy
        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> eiM(values.data());

        return eiM;
      }
    
      static cv::Mat ImageToCVMat(const sensor_image& rM) {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(rM, rM.encoding);
        cv::Mat cvM = cv_ptr->image;

        return cvM;
      }

      static std::map<long unsigned int, int> IntTupleVectorToMap(const std::vector<int_tuple>& rV) {
        std::map<long unsigned int, int> cppMap = std::map<long unsigned int, int>();
        for (int_tuple t : rV) {
          //std::cout << t.x1 <<", " << t.x2 << ", ";
          cppMap[t.x1] = t.x2;
        }
   
        //// Loop through the map using iterators
        //for (auto it = cppMap.begin(); it != cppMap.end(); ++it) {
        //    std::cout << "Key: " << it->first << ", Value: " << it->second << std::endl;
        //}
        return cppMap;
      }


      static std::map<orb_keyframe*, std::tuple<int,int>> KeyValuePairVectorToMap(const std::vector<key_value_pair>& rKVP, std::map<long unsigned int, orb_keyframe*>& mpOrbKeyFrames) {
        std::map<orb_keyframe*, std::tuple<int,int>> cppMap;
        
        for (const auto& kvp : rKVP)
        {
          if(mpOrbKeyFrames.find(kvp.key) != mpOrbKeyFrames.end()) {
            orb_keyframe* kf = mpOrbKeyFrames[kvp.key]; 
            cppMap.insert(std::make_pair(kf, std::make_tuple(kvp.value.x1, kvp.value.x2)));
          }
        }
        
        return cppMap;
      }

    private:


  };

  class OrbToRos {
    public: 
      static imu_bias ImuBiasToRosBias(const ORB_SLAM3::IMU::Bias& b) {
        imu_bias rB;
        rB.acc.x = b.bax; //imu::bias mbiasgba;
        rB.acc.y = b.bay; //imu::bias mbiasgba;
        rB.acc.z = b.baz; //imu::bias mbiasgba;
      
        rB.gyro.x = b.bwx; //imu::bias mbiasgba;
        rB.gyro.y = b.bwy; //imu::bias mbiasgba;
        rB.gyro.z = b.bwz; //imu::bias mbiasgba;
        
        return rB;
      }

      static imu_preintegrated ImuPreintegratedToRosPreintegrated(ORB_SLAM3::IMU::Preintegrated* piImu) {
        imu_preintegrated msgPiImu;
        msgPiImu.d_t = piImu->dT;

        ros_matrix C;
        int rowsC= piImu->C.rows();
        int columnsC = piImu->C.cols();
        int sizeC = rowsC * columnsC;
        std::vector<float> flattenedVectorC(piImu->C.data(), piImu->C.data()+sizeC);
        
        C.rows = rowsC;
        C.columns = columnsC;
        C.data = flattenedVectorC;
        
        msgPiImu.c = C;
        

        ros_matrix Info;
        int rowsInfo = piImu->Info.rows();
        int columnsInfo = piImu->Info.cols();
        int sizeInfo = rowsInfo * columnsInfo;
        std::vector<float> flattenedVectorInfo(piImu->Info.data(), piImu->Info.data()+sizeInfo);
        
        Info.rows = rowsInfo;
        Info.columns = columnsInfo;
        Info.data = flattenedVectorInfo;
        
        msgPiImu.info = Info;


        // Diagonal matrix so doesnt work like this, figure out other way.
        //ros_matrix Nga, NgaWalk;
        //int rowsNga = piImu->Nga.rows();
        //int columnsNga = piImu->Nga.cols();
        //int sizeNga = rowsNga * columnsNga;
        //std::vector<float> flattenedVectorNga(piImu->Nga.data(), piImu->Nga.data()+sizeNga);
        //Nga.data = flattenedVectorNga;
        //msgPiImu.nga = Nga;


        msgPiImu.b = ImuBiasToRosBias(piImu->GetOriginalBias());
        msgPiImu.d_r = CppToRos::EigenMatrix3ToMatrix(piImu->GetOriginalDeltaRotation()); // Eigen::Matrix3f dR;
        msgPiImu.d_v = CppToRos::EigenVector3fToVector3(piImu->GetOriginalDeltaVelocity());
        msgPiImu.d_p = CppToRos::EigenVector3fToVector3(piImu->GetOriginalDeltaPosition()); // Eigen::Vector3f dV, dP; 
        msgPiImu.j_rg = CppToRos::EigenMatrix3ToMatrix(piImu->JRg);
        msgPiImu.j_vg = CppToRos::EigenMatrix3ToMatrix(piImu->JVg);
        msgPiImu.j_va = CppToRos::EigenMatrix3ToMatrix(piImu->JVa);
        msgPiImu.j_pg = CppToRos::EigenMatrix3ToMatrix(piImu->JPg);
        msgPiImu.j_pa = CppToRos::EigenMatrix3ToMatrix(piImu->JPa);// Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
        msgPiImu.avg_a = CppToRos::EigenVector3fToVector3(piImu->avgA);
        msgPiImu.avg_w = CppToRos::EigenVector3fToVector3(piImu->avgW);// Eigen::Vector3f avgA, avgW;

        // Updated bias 
        msgPiImu.bu = ImuBiasToRosBias(piImu->GetUpdatedBias());
        // Dif between original and updated bias
        // This is used to compute the updated values of the preintegration
        ros_matrix db;
        Eigen::Matrix<float,6,1> delta_bias = piImu->GetDeltaBias();
        int rowsdb= delta_bias.rows();
        int columnsdb = delta_bias.cols();
        int sizedb = rowsdb * columnsdb;
        std::vector<float> flattenedVectordb(delta_bias.data(), delta_bias.data()+sizedb);
        
        db.rows =rowsdb;
        db.columns =columnsdb;
        db.data = flattenedVectordb;
        
        msgPiImu.db = db;

        return msgPiImu;
      }

      static std::vector<bow_vector> DBoW2VectorToRosBowVector(const DBoW2::BowVector& v) {
        std::vector<bow_vector> msgBv;

        std::cout << "converting bow vector from orb to ros" << std::endl;
        
        for (const auto& entry : v) {
          bow_vector mBv;
          mBv.word_id = entry.first;
          mBv.word_value = entry.second;
          msgBv.push_back(mBv);
        }

        return msgBv;

      } //DBoW2::bow_vectortor mBowVec;

      static std::vector<bow_feature_vector> DBoW2FeatVectorToRosBowFeatureVector(const DBoW2::FeatureVector& fV) {
        std::vector<bow_feature_vector> msgBfv;
        std::cout << "converting bow feature vector from orb to ros" << std::endl;

        for(const auto& entry : fV) {
          bow_feature_vector bFv;
          bFv.node_id = entry.first;
          bFv.features = entry.second;
          msgBfv.push_back(bFv);
        }

        return msgBfv;
      } //DBoW2::FeatureVector mFeatVec;
  };

  class RosToOrb {
    public: 
      static ORB_SLAM3::IMU::Bias RosBiasToOrbImuBias(const imu_bias& rB) {
        ORB_SLAM3::IMU::Bias oB = ORB_SLAM3::IMU::Bias(rB.acc.x, rB.acc.y, rB.acc.z, rB.gyro.x, rB.gyro.y, rB.gyro.z);
        
        return oB;
      }

      static ORB_SLAM3::IMU::Preintegrated* RosPreIntegratedImuToOrbPreintegratedImu(imu_preintegrated rPiImu) {
        ORB_SLAM3::IMU::Preintegrated oPiImu;
        //msgPiImu.d_t = piImu->dT;

        //ros_matrix C;
        //int rowsC= piImu->C.rows();
        //int columnsC = piImu->C.cols();
        //int sizeC = rowsC * columnsC;
        //std::vector<float> flattenedVectorC(piImu->C.data(), piImu->C.data()+sizeC);
        //
        //C.rows = rowsC;
        //C.columns = columnsC;
        //C.data = flattenedVectorC;
        //
        //msgPiImu.c = C;
        //

        //ros_matrix Info;
        //int rowsInfo = piImu->Info.rows();
        //int columnsInfo = piImu->Info.cols();
        //int sizeInfo = rowsInfo * columnsInfo;
        //std::vector<float> flattenedVectorInfo(piImu->Info.data(), piImu->Info.data()+sizeInfo);
        //
        //Info.rows = rowsInfo;
        //Info.columns = columnsInfo;
        //Info.data = flattenedVectorInfo;
        //
        //msgPiImu.info = Info;


        //// Diagonal matrix so doesnt work like this, figure out other way.
        ////ros_matrix Nga, NgaWalk;
        ////int rowsNga = piImu->Nga.rows();
        ////int columnsNga = piImu->Nga.cols();
        ////int sizeNga = rowsNga * columnsNga;
        ////std::vector<float> flattenedVectorNga(piImu->Nga.data(), piImu->Nga.data()+sizeNga);
        ////Nga.data = flattenedVectorNga;
        ////msgPiImu.nga = Nga;


        //msgPiImu.b = ImuBiasToRosBias(piImu->GetOriginalBias());
        //msgPiImu.d_r = CppToRos::EigenMatrix3ToMatrix(piImu->GetOriginalDeltaRotation()); // Eigen::Matrix3f dR;
        //msgPiImu.d_v = CppToRos::EigenVector3fToVector3(piImu->GetOriginalDeltaVelocity());
        //msgPiImu.d_p = CppToRos::EigenVector3fToVector3(piImu->GetOriginalDeltaPosition()); // Eigen::Vector3f dV, dP; 
        //msgPiImu.j_rg = CppToRos::EigenMatrix3ToMatrix(piImu->JRg);
        //msgPiImu.j_vg = CppToRos::EigenMatrix3ToMatrix(piImu->JVg);
        //msgPiImu.j_va = CppToRos::EigenMatrix3ToMatrix(piImu->JVa);
        //msgPiImu.j_pg = CppToRos::EigenMatrix3ToMatrix(piImu->JPg);
        //msgPiImu.j_pa = CppToRos::EigenMatrix3ToMatrix(piImu->JPa);// Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
        //msgPiImu.avg_a = CppToRos::EigenVector3fToVector3(piImu->avgA);
        //msgPiImu.avg_w = CppToRos::EigenVector3fToVector3(piImu->avgW);// Eigen::Vector3f avgA, avgW;

        //// Updated bias 
        //msgPiImu.bu = ImuBiasToRosBias(piImu->GetUpdatedBias());
        //// Dif between original and updated bias
        //// This is used to compute the updated values of the preintegration
        //ros_matrix db;
        //Eigen::Matrix<float,6,1> delta_bias = piImu->GetDeltaBias();
        //int rowsdb= delta_bias.rows();
        //int columnsdb = delta_bias.cols();
        //int sizedb = rowsdb * columnsdb;
        //std::vector<float> flattenedVectordb(delta_bias.data(), delta_bias.data()+sizedb);
        //
        //db.rows =rowsdb;
        //db.columns =columnsdb;
        //db.data = flattenedVectordb;
        //
        //msgPiImu.db = db;

        return &oPiImu;
      }

      static DBoW2::BowVector RosBowVectorToDBoW2Vector(const std::vector<bow_vector>& rBv) {
        DBoW2::BowVector oBv = DBoW2::BowVector();
        std::cout << "converting bow vector from ros to orb" << std::endl;
        for (const auto& x : rBv) {
          oBv.addWeight(x.word_id, x.word_value);
        }

        return oBv;

      } //DBoW2::bow_vectortor mBowVec;

      static DBoW2::FeatureVector RosBowFeatureVectorToDBoW2FeatureVector(const std::vector<bow_feature_vector>& rBFv) {
        DBoW2::FeatureVector oBFv = DBoW2::FeatureVector();
        std::cout << "converting bow feature vector from ros to orb" << std::endl;

        for(const auto& entry : rBFv) {
         unsigned int node_id = entry.node_id;
         for (const auto& feature : entry.features) {
           oBFv.addFeature(node_id, feature);
         }
        }

        return oBFv;
      } //DBoW2::FeatureVector mFeatVec;
  };
};


#endif
