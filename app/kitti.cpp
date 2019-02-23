#include <iostream>
#include "kitti_data_reader.h"
#include "file-system-tools.h"
#include "elas.h"
#include "image.h"
#include <string>
using  namespace std;

int main (int argc, char** argv) {
    std::string data_path = "/home/pang/data/dataset/kitti/00/";
    KittiDataReader kittiDataReader(data_path);

    std::string disp_save_path = data_path + "/elas";
    if (!common::pathExists(disp_save_path)){
        common::createPath(disp_save_path);
    }

    int cnt = 0;
    while (kittiDataReader.cursorToNextData()) { {
        cv::Mat image_left = kittiDataReader.getThisImage0();
        cv::Mat image_right = kittiDataReader.getThisImage1();

        cv::imshow("left", image_left);
        cv::imshow("right", image_right);
        cv::waitKey(3);

            // save disparity images
            std::string output_1 = disp_save_path + "/left_"+ std::to_string(cnt) + ".pgm";
            std::string output_2 = disp_save_path + "/right_"+ std::to_string(cnt) + ".pgm";
            std::cout << "output_1: " << output_1 << std::endl;
            std::cout << "output_2: " << output_2 << std::endl;


            // get image width and height
            int32_t width  = image_left.cols;
            int32_t height = image_left.rows;

            // allocate memory for disparity images
            const int32_t dims[3] = {width,height,width}; // bytes per line = width
            cv::Mat1f D1_data(height, width);
            cv::Mat1f D2_data(height, width);

            // process
            Elas::parameters param;
            param.postprocess_only_left = false;
            Elas elas(param);
            float* D_left_ptr = reinterpret_cast<float*>(D1_data.data);
            float* D_right_ptr = reinterpret_cast<float*>(D2_data.data);
            elas.process(image_left.data,image_right.data,D_left_ptr,D_right_ptr,dims);

            // copy float to uchar

            cv::Mat1b D1(height, width);
            cv::Mat1b D2(height, width);

            double min, max;
            cv::minMaxLoc(D1_data, &min, &max);
            D1_data.convertTo(D1,CV_8U,255.0/max);
            cv::minMaxLoc(D2_data, &min, &max);
            D2_data.convertTo(D2,CV_8U,255.0/max);
            std::cout << "min max :" << min << " " <<max << std::endl;

            cv::imshow("D1", D1);
            cv::imshow("D2", D2);
            cv::waitKey(1);

//            savePGM(D1,output_1.c_str());
//            savePGM(D2,output_2.c_str());

            cnt ++;

    }

    }
    return 0;
}