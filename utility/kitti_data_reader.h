#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <ros/time.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>



class KittiDataReader {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KittiDataReader(const std::string data_path);

    bool cursorToNextData();
    cv::Mat getThisImage0();
    std::string getThisImage0FileName();
    cv::Mat getThisImage1();
    cv::Mat getThisDepth();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getThisLidarPointcloud();
    Eigen::Isometry3d getThisImage0Pose();
//    ros::Time getThisTimestamp();
    Eigen::Matrix<double,3,4> getCameraCalibP0();
    Eigen::Matrix<double,3,4> getCameraCalibP1();
    Eigen::Matrix<double,3,4> getCameraCalibP2();
    Eigen::Matrix<double,3,4> getCameraCalibP3();
    Eigen::Matrix4d getT_cam_lidar();

    int imageWidth() { return image_width_;}
    int imageHeight() { return image_height_;}
    int size() {return image0_list_.size();};
    int getCurrentCursorCnt() {return cursor_cnt_;}

private:
    // buffer
    std::vector<std::string> image0_list_;
    std::vector<std::string> image1_list_;
    std::vector<std::string> depth_list_;
    std::vector<std::string> velodyne_list_;
    std::vector<std::vector<double>> pose_list_;
//    std::vector<ros::Time> times_list_;
    Eigen::Matrix<double,3,4> P0_;
    Eigen::Matrix<double,3,4> P1_;
    Eigen::Matrix<double,3,4> P2_;
    Eigen::Matrix<double,3,4> P3_;
    Eigen::Matrix<double,4,4> Tr_;

    // itr
    std::vector<std::string>::iterator image0_list_itr_;
    std::vector<std::string>::iterator image1_list_itr_;
    std::vector<std::string>::iterator depth_list_itr_;
    std::vector<std::string>::iterator velodyne_list_itr_;
    std::vector<std::vector<double>>::iterator pose_list_itr_;
//    std::vector<ros::Time>::iterator times_list_itr_;

    //
    int cursor_cnt_ = 0;
    //
    const int image_width_ = 1241;
    const int image_height_ = 376;
};