#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching.hpp>

#include "VisualPipeline.h"

    VisualPipeline::VisualPipeline(const std::string& conedetection_model_path, const std::string& keypoint_model_path){
        printf("VP: readNetFromONNX(%s)\n", conedetection_model_path.c_str());
        this->cone_detection = cv::dnn::readNetFromONNX(conedetection_model_path);
        printf("VP: readNetFromONNX(%s)\n", keypoint_model_path.c_str());
        this->keypoint_detection = cv::dnn::readNetFromONNX(keypoint_model_path);
    }

    Eigen::MatrixXd VisualPipeline::get_relative_cone_positions(const std::string& frame_path){
        Eigen::Matrix<double, 1, 2> err_out;
        err_out << 0, 0; //maybe better Matrix* as input and return status code?

        //load image. (maybe have frame come passed as frame instead of path.?)
        this->image = cv::imread(frame_path, 1);
        if ( !image.data )
        {
            printf("No image data \n");
            return err_out;
        }
        cv::resize(this->image, this->image, cv::Size(this->img_h, this->img_w), 0, 0, cv::INTER_LINEAR);
        //detect bounding boxes in frame
        cv::Mat blob;
        cv::Scalar mean{0.4151, 0.3771, 0.4568};  // what are these lines doing?
        cv::Scalar std{0.2011, 0.2108, 0.1896};
        bool swapRB = false;
        bool crop = false;
        cv::dnn::blobFromImage(image, blob, 1.0, cv::Size(this->img_w, this->img_h), mean, swapRB, crop);
        if (std.val[0] != 0.0 && std.val[1] != 0.0 && std.val[2] != 0.0) {
            cv::divide(blob, std, blob);
        }

        //blob.size = 1, 3, 1216, 7077993, 6029412
        printf("blob.size = %i, %i, %i, %i, %i\n", blob.size[0], blob.size[1], blob.size[2], blob.size[4], blob.size[5]);
        this->cone_detection.setInput(blob);
        cv::Mat prob = this->cone_detection.forward();
        printf("prop.size = %i, %i, %i, %i\n", prob.size[0], prob.size[1], prob.size[2], prob.size[3]);
        //cv::Mat tmp(prob.size[2], prob.size[3], CV_32F, prob.ptr<float>());
        //std::cout << "prob = \n" << tmp << "end_of_prob\n";

        // Apply sigmoid
        cv::Mat probReshaped = prob.reshape(1, prob.total() * prob.channels());
        std::vector<float> probVec = probReshaped.isContinuous() ? probReshaped : probReshaped.clone();

        cv::Point classIdPoint;
        double confidence;
        minMaxLoc(prob.reshape(1, 1), 0, &confidence, 0, &classIdPoint);
        int classId = classIdPoint.x;
        std::cout << " ID " << classId << " confidence " << confidence << std::endl;

        //[(bb[0], img[bb[1]:bb[3], bb[2]:bb[4]]) for bb in detected_bounding_boxes]
        //detect keypoints in cone images
        //detect PnP from camera to keypoints
        return err_out;
    }
