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
            float* D1_data = (float*)malloc(width*height*sizeof(float));
            float* D2_data = (float*)malloc(width*height*sizeof(float));

            // process
            Elas::parameters param;
            param.postprocess_only_left = false;
            Elas elas(param);
            uchar* left_ptr = reinterpret_cast<uchar*>(image_left.data);
            uchar* right_ptr = reinterpret_cast<uchar*>(image_right.data);
            elas.process(left_ptr,right_ptr,D1_data,D2_data,dims);

            // find maximum disparity for scaling output disparity images to [0..255]
            float disp_max = 0;
            for (int32_t i=0; i<width*height; i++) {
                if (D1_data[i]>disp_max) disp_max = D1_data[i];
                if (D2_data[i]>disp_max) disp_max = D2_data[i];
            }

            // copy float to uchar
            image<uchar> *D1 = new image<uchar>(width,height);
            image<uchar> *D2 = new image<uchar>(width,height);
            for (int32_t i=0; i<width*height; i++) {
                D1->data[i] = (uint8_t)max(255.0*D1_data[i]/disp_max,0.0);
                D2->data[i] = (uint8_t)max(255.0*D2_data[i]/disp_max,0.0);
            }



            savePGM(D1,output_1.c_str());
            savePGM(D2,output_2.c_str());

            // free memory

            delete D1;
            delete D2;
            free(D1_data);
            free(D2_data);
            cnt ++;

    }

    }
    return 0;
}