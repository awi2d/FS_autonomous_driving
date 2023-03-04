#define debugMode true
#include <iostream>
#include <Eigen/Dense> //git clone https://gitlab.com/libeigen/eigen.git & set path in CMake to correct dir

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching.hpp>
#if debugMode
#include <chrono>  //mesuring time
#include <fstream>  // reading file
#include <regex>  // reading file
#include <vector>
#include <string>
#endif

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

// read csv file (copied from https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c):
std::vector<std::vector<double>> read_csv(std::istream& str)
{
    // return[rowindex][cellindex] = cellindex cell of row rowindex.
    std::vector<std::vector<double>> result;
    std::string line;
    std::string cell;
    while(std::getline(str,line)){
        //printf("read_csv: line = %s\n", line.c_str());
        std::vector<double> line_result;
        std::stringstream lineStream(line);


        while(std::getline(lineStream,cell, ','))
        {
            double tmp = ::atof(cell.c_str());
            //printf("read_csv: cell = %s = %f\n", cell.c_str(), tmp);  // empty string gets translated to 0
            line_result.push_back(tmp);
        }
        // This checks for a trailing comma with no data after it.
        if (!lineStream && cell.empty())
        {
            // If there was a trailing comma then add an empty element.
            line_result.emplace_back(0);
        }
        result.push_back(line_result);
    }
    return result;
}

#include <sys/stat.h>
#include <algorithm>

typedef double heading;
typedef double meter;
typedef double second;
typedef int droneFrnr;
typedef std::tuple<double, double> pxpos;
typedef std::tuple<int, double, double, double, double> boundingbox;  // (cls, posw, posh, sizew, sizeh)
typedef std::tuple<meter, heading> distheading;
typedef std::vector<pxpos> cone_keypoints; // always has length 7.
typedef std::tuple<meter, meter, heading> pose;
//copied from https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exists-using-standard-c-c11-14-17-c
inline bool file_exists(const std::string& name) {
    struct stat buffer{};
    return (stat (name.c_str(), &buffer) == 0);
}

std::vector<std::string> split(const std::string& str, char delemiter){
    std::vector<std::string> res;
    std::stringstream stst(str);
    std::string tmp;
    while(std::getline(stst, tmp, delemiter)){
        res.push_back(tmp);
    }
    if (!stst && tmp.empty())
    {
        // If there was a trailing ${delemiter} then add an empty element.
        res.emplace_back("");
    }
    return res;
}

double str2double(const std::string& s){
    double tmp = ::atof(s.c_str());
    return tmp;
}
int str2int(const std::string& s){
    int tmp = std::stoi(s);
    return tmp;
}

pxpos str2pxpos(const std::string& s){
    std::vector<std::string> s_split = split(s, '#');
    pxpos res{str2double(s_split[0]), str2double(s_split[1])};
    return res;
}

// read file (copied from https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c):
std::vector<boundingbox> get_boundingbox(const std::string& cam, int framenr)
{
    // should have the same behavior as
    // def get_boundingboxes(cam: str, framenr: int) -> [cone_bounding_box]:
    std::string filename = "C:/Users/Idefix/PycharmProjects/tmpProject/vp_labels/"+cam+"_bb/"+cam+"_frame_"+std::to_string(framenr)+".txt";
    std::vector<boundingbox> result;
    if(file_exists(filename)){
        std::ifstream file(filename);

        std::string line;
        std::string cell;
        while(std::getline(file,line)){
            // line = "0 0.5234 0.54241 0.12345 0.12345"
            std::vector<std::string> line_split = split(line, ' ');
            boundingbox res {str2int(line_split[0]), str2double(line_split[1]), str2double(line_split[2]), str2double(line_split[3]), str2double(line_split[4])};
            result.push_back(res);
        }
    }
    return result;  // empty if file doesnt exist
}
std::map<std::tuple<int, int>, std::vector<pxpos>> keypoints_cache;
std::vector<pxpos> get_cone_keypoints(const std::string& cam, int framenr, int cone){
    // should have the same behavior as
    // def get_cone_keypoints(cam, framenr, cone) -> [(normalised_px_w, normalised_px_h)]:
    if(keypoints_cache.empty()){
        std::string filename = "C:/Users/Idefix/PycharmProjects/tmpProject/vp_labels/cone_annotations.csv";  // hardcoeded filenames in the sourcecode are a good idea, right?
        if(file_exists(filename)){
            std::ifstream file(filename);
            std::string line;
            std::string cell;
            while(std::getline(file,line)){
                // line = "/cones/camL3_frame_1297.jpg_cone_17.jpg,0.7255555555555555#0.16666666666666666,0.6055555555555555#0.3466666666666667,0.51#0.5488888888888889,0.39111111111111113#0.7433333333333333,0.7822222222222223#0.36777777777777776,0.7911111111111111#0.6077777777777778,0.7811111111111111#0.7711111111111111"
                std::vector<std::string> line_split = split(line, ',');
                std::vector<pxpos> res {str2pxpos(line_split[1]), str2pxpos(line_split[2]), str2pxpos(line_split[3]), str2pxpos(line_split[4]), str2pxpos(line_split[5]), str2pxpos(line_split[6]), str2pxpos(line_split[7])};
                std::tuple<int, int> key = {framenr, cone};
                keypoints_cache[key] = res;
            }
        }else{
            printf("ERROR: file doesnt exists, change path in get_cone_keypoints to correct location of cone_annotations.csv\n");
        }
    }
    std::tuple<int, int> key = {framenr, cone};
    return keypoints_cache[key];  // empty if file doesnt exist
}


std::vector<std::tuple<int, pose>> get_car_poses(){
    std::string filename = "C:/Users/Idefix/PycharmProjects/tmpProject/vp_labels/droneview/droneFrnr_trueCarMPose.txt";
    if(!file_exists(filename)){
        printf("ERROR: file %s does not exist", filename.c_str());
        std::vector<std::tuple<int, pose>> res;
        return res;
    }
    std::ifstream file(filename);
    std::string line;
    std::string cell;
    std::vector<std::tuple<int, pose>> res;
    while(std::getline(file,line)){
        // line = "/cones/camL3_frame_1297.jpg_cone_17.jpg,0.7255555555555555#0.16666666666666666,0.6055555555555555#0.3466666666666667,0.51#0.5488888888888889,0.39111111111111113#0.7433333333333333,0.7822222222222223#0.36777777777777776,0.7911111111111111#0.6077777777777778,0.7811111111111111#0.7711111111111111"
        std::vector<std::string> line_split = split(line, ',');
        pose carpose = {str2double(line_split[1]), str2double(line_split[2]), str2double(line_split[3])};// pos_north, pos_east, heading
        std::tuple<int, pose> tmp = {str2int(line_split[0]), carpose};
        res.push_back(tmp);
    }
    return res;

}

distheading customPnP(const cone_keypoints& keypoints, boundingbox bb){
    int cls; double posw, posh, bb_sizew, bb_sizeh;
    std::tie(cls, posw, posh, bb_sizew, bb_sizeh) = bb;
    int imgsize_h = 1200; int imgsize_w = 1920;
    double obj_Distm[7][7] = {
            {0, 0.09931057723962547, 0.18581319480106973, 0.2893731266180254, 0.09931057723962547, 0.18581319480106973, 0.2893731266180254},
            {0.09931057723962547, 0, 0.08783333333333333, 0.18816666666666668, 0.058826410187308546, 0.126375, 0.20548663807186746},
            {0.18581319480106973, 0.08783333333333333, 0, 0.1, 0.126375, 0.08119296009798978, 0.156},
            {0.2893731266180254, 0.18816666666666668, 0.1, 0, 0.20548663807186746, 0.156, 0.1159014116265954},
            {0.09931057723962547, 0.058826410187308546, 0.126375, 0.20548663807186746, 0, 0.08783333333333333, 0.18816666666666668},
            {0.18581319480106973, 0.126375, 0.08119296009798978, 0.156, 0.08783333333333333, 0, 0.1},
            {0.2893731266180254, 0.20548663807186746, 0.156, 0.1159014116265954, 0.18816666666666668, 0.1, 0}
    };  // distances between keypoints on physical cone in m.
    double tmp[21];
    int tmpi=0;
    for(int i=0; i<6; i++){
        for(int j=i+1; j<7; j++){
            double pxdist = std::sqrt(std::pow((std::get<0>(keypoints[i])*imgsize_w*bb_sizew-std::get<0>(keypoints[j])*imgsize_w*bb_sizew), 2)
                    + std::pow((std::get<1>(keypoints[i])*imgsize_h*bb_sizeh-std::get<1>(keypoints[j])*imgsize_h*bb_sizeh), 2));  // = dist between keypoints i and j in pixel space
            double mpropx = obj_Distm[i][j]/pxdist;
            tmp[tmpi] = mpropx;
            tmpi++;
        }
    }
    std::sort(std::begin(tmp), std::end(tmp));
    double dist = -1.97349351e-01 + 1.82927777e+03*tmp[10];
    double avg_widthpos = 0;
    for(int i=0; i<7; i++){
        avg_widthpos += std::get<0>(keypoints[i])/7;
    }
    avg_widthpos = posw-0.5*bb_sizew+bb_sizew*avg_widthpos;
    double heading = -0.90914045 + 1.07626391*avg_widthpos;
    distheading res {dist, heading};
    return res;
}

pose get_at_time(std::vector<std::tuple<droneFrnr, pose>> data, double time){
    if(time <= std::get<0>(data[0])){
        printf("warning: time %f is before time range of data (%i, %i)\n", time, std::get<0>(data[0]), std::get<0>(data[data.size()-1]));
        return std::get<1>(data[0]);
    }
    if(time >= std::get<0>(data[data.size()-1])){
        printf("warning: time %f is after time range of data (%i, %i)\n", time, std::get<0>(data[0]), std::get<0>(data[data.size()-1]));
        return std::get<1>(data[data.size()-1]);
    }
    for(int i=1; i<data.size(); i++){
        if(std::get<0>(data[i]) > time){
            if(time == std::get<0>(data[i-1])){
                return std::get<1>(data[i-1]);
            }
            double w0 = abs(time-std::get<0>(data[i-1]));
            double w1 = abs(std::get<0>(data[i])-time);
            double sum = w0+w1;
            pose v0 = std::get<1>(data[i]);
            pose v1 = std::get<1>(data[i-1]);
            //(w1*y[i-1]+w0*y[i])/sum
            pose res = {(std::get<0>(v0)*w0+std::get<0>(v1)*w1)/sum, (std::get<1>(v0)*w0+std::get<1>(v1)*w1)/sum, (std::get<1>(v0)*w0+std::get<1>(v1)*w1)/sum};
            return res;

        }
    }
    printf("ERROR: unreachable code in get_at_time(data[0].time=%i, data[-1].time=%i, time=%f)", std::get<0>(data[0]), std::get<0>(data[data.size()-1]), time);
    return std::get<1>(data[0]);
}

int main() {
    //std::get<i>(tupel) == i-th element of tupel
    std::vector<std::tuple<int, pose>> carposes = get_car_poses();
    pose old_carpose = get_at_time(carposes, (1280/20.0+29.56)*25.0);
    for(int camL3_frnr=1280; camL3_frnr<2643; camL3_frnr++){
        std::vector<boundingbox> bbs = get_boundingbox("camL3", camL3_frnr);
        //camL3_frnr/20 = drone3_frnr/25-29.56 -> drone3_frnr = (camL3_frnr/20+29.56)*25
        pose carpose = get_at_time(carposes, (camL3_frnr/20.0+29.56)*25.0);
        printf("carpose[%f] = (%f, %f, %f)\n", (camL3_frnr/20.0+29.56)*25.0, std::get<0>(carpose), std::get<1>(carpose), std::get<1>(carpose));
        for(int bbi=0; bbi < bbs.size(); bbi++){
            cone_keypoints kp = get_cone_keypoints("camL3", camL3_frnr, bbi);
            if(kp.size() == 7){
                distheading vp_det = customPnP(kp, bbs[bbi]);
                printf("vp_det[%i] = (%f, %f)\n", bbi, std::get<0>(vp_det), std::get<1>(vp_det));
            }
            // slam_frontend

            // add vp constraints
            // add odometry constraint
            // add gnss constraint


        }
    }

    return 0;
    std::ifstream file("C:/Users/Idefix/PycharmProjects/tmpProject/merged_rundata_csv/alldata_2022_12_17-14_43_59_id3.csv");
    std::vector<std::vector<double>> csv_content = read_csv(file);

    printf("csv_content.size() = %zu\n", csv_content.size());

    std::string frames_pathL = "C:/Users/Idefix/PycharmProjects/datasets/testrun_2022_12_17/cam_footage/left_cam_14_46_00";
    std::string frames_pathR = "C:/Users/Idefix/PycharmProjects/datasets/testrun_2022_12_17/cam_footage/right_cam_14_46_00";
    std::string jpg = ".jpg";

    //StateEstimator* state_estimator = new StateEstimator(false, true, true, true, true, false);

    std::string conedet_path = "C:/Users/Idefix/PycharmProjects/datasets/Flos_objectdetection/2022_04_08_best.onnx";
    std::string keypointdet_path = "C:/Users/Idefix/PycharmProjects/tmpProject/keypoint_regression_best.onnx";

    //VisualPipeline* visual_pipeline = new VisualPipeline(conedet_path, keypointdet_path);

    //SLAMg2o* slam = new SLAMg2o();
    return 0;
    for(int i=2; i<csv_content.size(); i++){
        std::vector<double> sensordata = csv_content[i];  // alwas has length 19
        // sensordata =
        // [0:time, 1:BMS_SOC_UsbFlRec, 2:Converter_L_N_actual_UsbFlRec, 3:Converter_R_N_actual_UsbFlRec,
        // 4:Converter_L_RPM_Actual_Filtered_UsbFlRec, 5:Converter_R_RPM_Actual_Filtered_UsbFlRec, 6:Converter_L_Torque_Out_UsbFlRec, 7:Converter_R_Torque_Out_UsbFlRec,
        // 8:ECU_ACC_X_UsbFlRec, 9:ECU_ACC_Y_UsbFlRec, 10:ECU_ACC_Z_UsbFlRec,
        // 11:GNSS_heading_UsbFlRec, 12:GNSS_latitude_UsbFlRec, 13:GNSS_longitude_UsbFlRec, 14:GNSS_speed_over_ground_UsbFlRec,
        // 15:SWS_angle_UsbFlRec, 16:cam_left, 17:cam_right, 18:cam_drone]
        double t = sensordata[0];  // time of sensordatadict

        /*
        //visual pipeline
        std::string img_pathL = frames_pathL+std::to_string(sensordata[16])+".jpg";  // path to frame of left cam
        std::string img_pathR = frames_pathR+std::to_string(sensordata[17])+".jpg";  // path to frame of right cam
        printf("img_path = %s\n", img_pathL.c_str());
        Eigen::MatrixXd cone_pos = visual_pipeline->get_relative_cone_positions(img_pathL);
        //Eigen::MatrixXd cone_pos = visual_pipeline->get_relative_cone_positions(img_pathR);
        */

        //state estimation.
        /*
        Vector5d gps;
        gps << sensordata[12], sensordata[13], sensordata[14], sensordata[11], 0;  // TODO remove yawrate from gps
        Vector3d imu;
        imu << sensordata[8], sensordata[9], 0;  // TODO remove yawrate from gps
        Eigen::Vector2d rh;  // [turning_speed_rear_left_wheel, turning_speed_rear_right_wheel]
        rh << sensordata[4], sensordata[5];
        Eigen::Vector2d u;  // [Trq_Drive, wheelangel]
        u << sensordata[6], sensordata[15];  // TODO dont ignore right Torque
        Vector10d true_x;
        true_x << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        state_estimator->call(t, gps, imu, rh, u, true_x);
         */
        //car_state = state_estimator.getstate();

        //SLAM
        //slam.?(car_state, cone_pos);  //filling out this \? is main part of this work.
        //map = slam.get();

        //controller
        //controls = controler.get(car_state, map);

        //send controls to other programm parts/can?
    }
    return 0;
}
