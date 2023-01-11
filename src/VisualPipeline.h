#ifndef VELOCITY_ESTIMATOR_VISUALPIPELINE_H
#define VELOCITY_ESTIMATOR_VISUALPIPELINE_H
#include <Eigen/Dense>

class VisualPipeline {
private:
    cv::Mat image;
    cv::dnn::Net cone_detection;
    cv::dnn::Net keypoint_detection;
    int img_h = 1216; //size of input images.
    int img_w = 1920;
public:
    VisualPipeline(const std::string& conedetection_model_path, const std::string& keypoint_model_path);
    Eigen::MatrixXd get_relative_cone_positions(const std::string& frame_path);
};


#endif //VELOCITY_ESTIMATOR_VISUALPIPELINE_H
