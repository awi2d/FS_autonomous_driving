#define debugMode true
#include <iostream>
#include <Eigen/Dense> //git clone https://gitlab.com/libeigen/eigen.git & set path in CMake to correct dir
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching.hpp>
#if debugMode
#include <chrono>  //mesuring time
#include <fstream>  // reading file
#include <regex>  // reading file
#include <iomanip>
#include <thread>
#endif

#include "StateEstimator.h"
#include "VisualPipeline.h"

//test if finaly opencv is correctly installed, linked in CMakeList and PATH viariable contains Path to correct /bin Folder in oopencv without being over 1024 chars long
bool my_show_img(const std::string& filepath){
    cv::Mat image = cv::imread(filepath, 1);
    if ( !image.data )
    {
        printf("No image data \n");
        return false;
    }
    namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    imshow("Display Image", image);
    cv::waitKey(0);
    return true;
}

int main() {
    std::cout << "hallo Welt!\n";
    std::string frames_path = "C:/Users/Idefix/PycharmProjects/datasets/Basler Cam recordings (Accel)/left_cam_frames/frame";
    std::string jpg = ".jpg";

    StateEstimator* state_estimator = new StateEstimator(true, false, true, true, true, true, true, true);

    std::string conedet_path = "C:/Users/Idefix/PycharmProjects/datasets/Flos_objectdetection/2022_04_08_best.onnx";
    std::string keypointdet_path = "C:/Users/Idefix/PycharmProjects/tmpProject/keypoint_regression_best.onnx";

    VisualPipeline* visual_pipeline = new VisualPipeline(conedet_path, keypointdet_path);

    for(int t=0; t<10; t++){//1798 frames
        printf("t = %i\n", t);
        //visual pipeline
        std::string img_path = frames_path+std::to_string(170*t)+".jpg";
        printf("img_path = %s\n", img_path.c_str());
        Eigen::MatrixXd cone_pos = visual_pipeline->get_relative_cone_positions(img_path);

        //state estimation.
        //... = read csv;
        //KF.call(...);
        //car_state = KF.getstate();

        //SLAM
        //slam.?(car_state, cone_pos);  //filling out this \? is main part of this work.
        //map = slam.get();

        //controller
        //controls = controler.get(car_state, map);

        //send controls to other programm parts/can?
    }
    return 0;
}
