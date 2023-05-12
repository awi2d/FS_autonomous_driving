#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <utility>

#include "util.h"
#include "cppTypes.h"
#include "VisualPipeline.h"

VisualPipeline::VisualPipeline(const std::string& conedetection_model_path, const std::string& keypoint_model_path){
    // read neural nets from files
    //printf("VP: readNetFromONNX(%s)\n", conedetection_model_path.c_str());
    //this->cone_detection = cv::dnn::readNetFromONNX(conedetection_model_path);
    //printf("VP: readNetFromONNX(%s)\n", keypoint_model_path.c_str());
    //this->keypoint_detection = cv::dnn::readNetFromONNX(keypoint_model_path);
}

std::map<type_frnr, std::tuple<distheading, distheading>> orangecone_cache;  // distheading to blueside and yellowside orange cone, respectively
    std::tuple<distheading, distheading> get_orangecone_distheading(type_frnr frnr, std::string base_path="C:/Users/Idefix/PycharmProjects/eTeam_pyutil/vp_labels/"){
    if(orangecone_cache.empty()){
        std::string filename = std::move(base_path);
        filename.append("camL3_orangedistheading_l_r.txt");
        if(file_exists(filename)){
            std::ifstream file(filename);
            std::string line;
            std::string cell;
            while(std::getline(file,line)){
                // line = 1280,3.899880142336186,-0.44527546783681604,4.815976572849486,0.23268253044752063
                std::vector<std::string> line_split = split(line, ',');
                // /cones/cone_19028.jpg
                int key = str2int(line_split[0]);
                std::tuple<distheading, distheading> res = {{str2double(line_split[1]), str2double(line_split[2])}, {str2double(line_split[3]), str2double(line_split[4])}};
                orangecone_cache[key] = res;
            }
        }else{
            printf("ERROR: file doesnt exists, change path in get_cone_keypoints to correct location of cone_annotations.csv\n");
        }
    }
    auto it = orangecone_cache.find(frnr);
    if (it != orangecone_cache.end()) {
        return it->second;
    }
    printf("ERROR: key %i not in orangecone_cache.\n", frnr);
    std::tuple<distheading, distheading> res = {{-1, -1}, {-1, -1}};  // TODO this is a bad way to represent empty
    return res;
}


    std::vector<std::tuple<Color, distheading>> VisualPipeline::sim_get_relative_cone_positions(type_frnr camL3_frnr, type_frnr camR3_frnr) {
        // simulate what the correct implementation of the visual pipeline should do, but use handmade annotations instead of neural networks to get bounding boxes and keypoints.
        std::vector<std::tuple<Color, distheading>> vp_det;
        std::vector<boundingbox> bbs = get_boundingbox("camL3", camL3_frnr);
        //printf("number of bounding boxes of camL3_frnr=%u = %zu\n", camL3_frnr, bbs.size());
        bool orange_cones_not_added = true;
        for (unsigned int bbi = 0; bbi < bbs.size(); bbi++) {
            //printf("bbs[%i] = (%i, %f, %f, %f, %f)\n", bbi, std::get<0>(bbs[bbi]), std::get<1>(bbs[bbi]), std::get<2>(bbs[bbi]), std::get<3>(bbs[bbi]), std::get<4>(bbs[bbi]));
            if (std::get<0>(bbs[bbi]) == Color::blue || std::get<0>(bbs[bbi]) == Color::yellow) {  // color is blue or yellow (not orange)
                cone_keypoints kp = get_cone_keypoints("camL3", camL3_frnr, bbi);
                if (kp.size() == 7) {  // all cones have 7 keypoints or None, if they were not visible.
                    distheading relative_cone_position = customPnP(kp, bbs[bbi]);
                    //printf("distheading(bbi=%u) = (%f, %f)\n", bbi, std::get<0>(tmp), std::get<1>(tmp));
                    if (std::get<0>(relative_cone_position) < 10) {//ignore cone detections 10 or more meter away
                        vp_det.emplace_back(std::get<0>(bbs[bbi]), relative_cone_position);  // bounding box[0] = Color of that cone
                    }
                    //printf("vp_det[%i] = (%f, %f)\n", bbi, std::get<0>(vp_det), std::get<1>(vp_det));
                }
            } else {
                if (orange_cones_not_added) {  // only colors are blue, yellow and orange -> cone is orange (only execute once)
                    //for orange cone the keypoints doesnt work.
                    std::tuple<distheading, distheading> orange_det = get_orangecone_distheading(camL3_frnr);
                    if (std::get<0>(std::get<0>(orange_det)) > 0) {  // -1 used as error-value
                        orange_cones_not_added = false;
                        vp_det.emplace_back(Color::orange_bs, std::get<0>(orange_det));  // orange cone on the blue side
                        vp_det.emplace_back(Color::orange_ys, std::get<1>(orange_det));  // orange cone on the yellow side
                    }
                }
            }
        }
        return vp_det;
    }

    Eigen::MatrixXd VisualPipeline::real_get_relative_cone_positions(const std::string& frame_path){
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

        //blob.size = 1, 3, 1216, 7077993, 6029412  // maybe some of these sizes are random values from memory because c++ doest check for out-of-bounds array acceses?
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

