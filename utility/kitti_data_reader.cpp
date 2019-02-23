#include "kitti_data_reader.h"
#include <fstream>
#include "file-system-tools.h"

KittiDataReader::KittiDataReader(const std::string data_path) {
    const std::string image0_path = data_path + "image_0/";
    const std::string image1_path = data_path + "image_1/";
    const std::string velodyne_path = data_path + "velodyne/";
    const std::string pose_file = data_path + "pose.txt";
    const std::string calib_file = data_path + "calib.txt";
    const std::string times_file = data_path + "times.txt";

    const std::string depth_path = data_path + "depth/";

    /// image0
    common::getAllFilesInFolder(image0_path, &image0_list_);
    std::cout<<"image0_list: " << image0_list_.size() << std::endl;

    std::sort(image0_list_.begin(),image0_list_.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    /// image1
    common::getAllFilesInFolder(image1_path, &image1_list_);
    std::cout<<"image1_list: " << image1_list_.size() << std::endl;
    std::sort(image1_list_.begin(),image1_list_.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    /// external disp
    if (common::pathExists(depth_path)) {
        common::getAllFilesInFolder(depth_path, &depth_list_);
        std::cout<<"depth_list: " << depth_list_.size() << std::endl;
        std::sort(depth_list_.begin(),depth_list_.end(), [](std::string a, std::string b) {
            return !common::compareNumericPartsOfStrings(a,b);
        });
    }

    /// velodyne
    common::getAllFilesInFolder(velodyne_path, &velodyne_list_);
    std::cout<<"velodyne_list: " << velodyne_list_.size() << std::endl;
    std::sort(velodyne_list_.begin(),velodyne_list_.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    /// groudtruth
    std::string one_line;
    std::ifstream pose_ifs(pose_file);
    if (!pose_ifs.is_open()) {
        std::cout<< "Can NOT open: " << pose_file << std::endl;
    }
    while (!pose_ifs.eof()) {
        std::getline(pose_ifs, one_line);
        std::stringstream stream(one_line);
        if (one_line.empty())
            break;
        std::vector<double> e(16);
        stream >> e[0] >>  e[1] >>  e[2] >>  e[3]
                >> e[4] >>  e[5] >>  e[6] >>  e[7]
                >> e[8] >>  e[9] >>  e[10] >>  e[11];
        e[12] = 0;
        e[13] = 0;
        e[14] = 0;
        e[15] = 1;
        pose_list_.push_back(e);

    }
    std::cout<<"pose_list: " << pose_list_.size() << std::endl;

    /// ts
//    std::ifstream times_ifs(times_file);
//    if (!times_ifs.is_open()) {
//        std::cout<< "Can NOT open: " << times_file << std::endl;
//    }
//    while (!times_ifs.eof()) {
//        std::getline(times_ifs, one_line);
//        std::stringstream stream(one_line);
//        if (one_line.empty())
//            break;
//
//        double ts;
//        stream >> ts;
//        ros::Time time(ts);
//        times_list_.push_back(time);
//    }
//    std::cout<<"times_ifs: " << times_list_.size() << std::endl;


    /// calibs
    std::ifstream calib_ifs(calib_file);
    if (!calib_ifs.is_open()) {
        std::cout<< "Can NOT open: " << calib_file << std::endl;
    }


    
    std::getline(calib_ifs, one_line);
    std::stringstream stream(one_line);
    std::string temp;
    stream >> temp >> P0_(0,0) >> P0_(0,1) >> P0_(0,2) >> P0_(0,3)
                    >> P0_(1,0) >> P0_(1,1) >> P0_(1,2) >> P0_(1,3)
                    >> P0_(2,0) >> P0_(2,1) >> P0_(2,2) >> P0_(2,3);

    std::getline(calib_ifs, one_line);
    std::stringstream stream1(one_line);
    stream1 >> temp >> P1_(0,0) >> P1_(0,1) >> P1_(0,2) >> P1_(0,3)
           >> P1_(1,0) >> P1_(1,1) >> P1_(1,2) >> P1_(1,3)
           >> P1_(2,0) >> P1_(2,1) >> P1_(2,2) >> P1_(2,3);

    std::getline(calib_ifs, one_line);
    std::stringstream stream2(one_line);
    stream2 >> temp >> P2_(0,0) >> P2_(0,1) >> P2_(0,2) >> P2_(0,3)
            >> P2_(1,0) >> P2_(1,1) >> P2_(1,2) >> P2_(1,3)
            >> P2_(2,0) >> P2_(2,1) >> P2_(2,2) >> P2_(2,3);

    std::getline(calib_ifs, one_line);
    std::stringstream stream3(one_line);
    stream3 >> temp >> P3_(0,0) >> P3_(0,1) >> P3_(0,2) >> P3_(0,3)
            >> P3_(1,0) >> P3_(1,1) >> P3_(1,2) >> P3_(1,3)
            >> P3_(2,0) >> P3_(2,1) >> P3_(2,2) >> P3_(2,3);

    
    std::getline(calib_ifs, one_line);
    std::stringstream stream4(one_line);
    Tr_.setIdentity();
    stream4 >> temp >> Tr_(0,0) >> Tr_(0,1) >> Tr_(0,2) >> Tr_(0,3)
            >> Tr_(1,0) >> Tr_(1,1) >> Tr_(1,2) >> Tr_(1,3)
            >> Tr_(2,0) >> Tr_(2,1) >> Tr_(2,2) >> Tr_(2,3);


    image0_list_itr_ = image0_list_.begin();
    image1_list_itr_ = image1_list_.begin();
    velodyne_list_itr_ = velodyne_list_.begin();
//    times_list_itr_ = times_list_.begin();
    pose_list_itr_ = pose_list_.begin();

    depth_list_itr_ = depth_list_.begin();
}

bool KittiDataReader::cursorToNextData() {
    image0_list_itr_ ++;
    if (image0_list_itr_ == image0_list_.end() )
        return false;
    image1_list_itr_ ++;

    depth_list_itr_ ++;

    velodyne_list_itr_ ++;
    pose_list_itr_ ++;
//    times_list_itr_ ++;

    cursor_cnt_ ++;
    return true;
}

cv::Mat KittiDataReader::getThisImage0() {
    cv::Mat image = cv::imread(*image0_list_itr_, cv::IMREAD_GRAYSCALE);
    return image;
}

std::string KittiDataReader::getThisImage0FileName() {
    return *image0_list_itr_;
}


cv::Mat KittiDataReader::getThisImage1() {
    cv::Mat image = cv::imread(*image1_list_itr_, cv::IMREAD_GRAYSCALE);
    return image;
}

cv::Mat KittiDataReader::getThisDepth() {
    cv::Mat image = cv::imread(*depth_list_itr_, CV_LOAD_IMAGE_UNCHANGED);
    return image;
}

Eigen::Isometry3d KittiDataReader::getThisImage0Pose() {
    std::vector<double> pose_vec = *pose_list_itr_;
    Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> pose(pose_vec.data());
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.matrix() = pose;
    return iso;
}

//ros::Time KittiDataReader::getThisTimestamp() {
//    return *times_list_itr_;
//}

Eigen::Matrix<double,3,4> KittiDataReader::getCameraCalibP0() {
    return P0_;
}

Eigen::Matrix<double,3,4> KittiDataReader::getCameraCalibP1() {
    return P1_;
}

Eigen::Matrix<double,3,4> KittiDataReader::getCameraCalibP2() {
    return P2_;
}

Eigen::Matrix<double,3,4> KittiDataReader::getCameraCalibP3() {
    return P3_;
}

Eigen::Matrix4d KittiDataReader::getT_cam_lidar() {
    return Tr_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KittiDataReader::getThisLidarPointcloud() {
    // load point cloud
    std::string bin_file = *velodyne_list_itr_;
    std::fstream input(bin_file, std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << bin_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZ point;
        float tem;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &tem, sizeof(float));
        points->push_back(point);
    }
    input.close();
//    std::cout << "Read KTTI point cloud with " << i << " points" << std::endl;
    return points;
}