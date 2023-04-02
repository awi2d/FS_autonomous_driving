#ifndef VELOCITY_ESTIMATOR_VISUALPIPELINE_H
#define VELOCITY_ESTIMATOR_VISUALPIPELINE_H
#include <opencv2/dnn.hpp>
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
    Eigen::MatrixXd real_get_relative_cone_positions(const std::string& frame_path); // "real" vp that uses NN. currently not working
    static std::vector<std::tuple<int, distheading>> sim_get_relative_cone_positions(unsigned int camL_frnr, unsigned int camR_frnr); //simulate vp, read bb and keypoints from files
};


#endif //VELOCITY_ESTIMATOR_VISUALPIPELINE_H
