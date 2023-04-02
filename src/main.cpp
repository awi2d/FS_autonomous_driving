#define debugMode true
#include <iostream>
#include <Eigen/Dense> //git clone https://gitlab.com/libeigen/eigen.git & set path in CMake to correct dir
#include "util.h"
#include "VisualPipeline.h"
#include "SLAMg2o.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching.hpp>
#include <algorithm>
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
            double tmp = str2double(cell);
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

int main() {
    printf("__cplusplus = %ld\n", __cplusplus);//__cplusplus = 201402
    //std::get<i>(tupel) == i-th element of tupel

    std::ifstream file("C:/Users/Idefix/PycharmProjects/tmpProject/merged_rundata_csv/alldata_2022_12_17-14_43_59_id3.csv");
    std::vector<std::vector<double>> csv_content = read_csv(file);

    printf("csv_content.size() = %zu\n", csv_content.size());

    std::string frames_pathL = "C:/Users/Idefix/PycharmProjects/datasets/testrun_2022_12_17/cam_footage/left_cam_14_46_00";
    std::string frames_pathR = "C:/Users/Idefix/PycharmProjects/datasets/testrun_2022_12_17/cam_footage/right_cam_14_46_00";
    std::string jpg = ".jpg";

    //StateEstimator* state_estimator = new StateEstimator(false, true, true, true, true, false);

    std::string conedet_path = "C:/Users/Idefix/PycharmProjects/datasets/Flos_objectdetection/2022_04_08_best.onnx";
    std::string keypointdet_path = "C:/Users/Idefix/PycharmProjects/tmpProject/keypoint_regression_best.onnx";

    VisualPipeline* visual_pipeline = new VisualPipeline(conedet_path, keypointdet_path);
    SLAM* slam = new SLAM(true, false, false);
    slam->print();

    int old_camL3_frnr = 1280;
    double current_lat=0, current_lng=0, current_heading=0, current_speed=0, old_heading=0;
    double current_heading_time = 0, old_heading_time = 0;
    std::tuple<radiants, radiants> gps_base = {0, 0};
    for(int i=2; i<csv_content.size(); i++){
        std::vector<double> sensordata = csv_content[i];  // alwas has length 19
        // sensordata =
        // [0:time, 1:BMS_SOC_UsbFlRec, 2:Converter_L_N_actual_UsbFlRec, 3:Converter_R_N_actual_UsbFlRec,
        // 4:Converter_L_RPM_Actual_Filtered_UsbFlRec, 5:Converter_R_RPM_Actual_Filtered_UsbFlRec, 6:Converter_L_Torque_Out_UsbFlRec, 7:Converter_R_Torque_Out_UsbFlRec,
        // 8:ECU_ACC_X_UsbFlRec, 9:ECU_ACC_Y_UsbFlRec, 10:ECU_ACC_Z_UsbFlRec,
        // 11:GNSS_heading_UsbFlRec, 12:GNSS_latitude_UsbFlRec, 13:GNSS_longitude_UsbFlRec, 14:GNSS_speed_over_ground_UsbFlRec,
        // 15:SWS_angle_UsbFlRec, 16:cam_left, 17:cam_right, 18:cam_drone]
        double t = sensordata[0];  // time of sensordatadict
        int camL3_frnr = ssdt2camL(t);
        //printf("time[%i] = %f\n", i, t);
        //state estimation.
        /*
        Vector5d gps;
        gps << sensordata[12], sensordata[13], sensordata[14], sensordata[11], 0;  // TODO remove yawrate from gps
        Vector3d imu;
        imu << sensordata[8], sensordata[9], 0;  // TODO remove yawrate from imu
        Eigen::Vector2d rh;  // [turning_speed_rear_left_wheel, turning_speed_rear_right_wheel]
        rh << sensordata[4], sensordata[5];
        Eigen::Vector2d u;  // [Trq_Drive, wheelangel]
        u << sensordata[6], sensordata[15];  // TODO dont ignore right Torque
        Vector10d true_x;
        true_x << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        state_estimator->call(t, gps, imu, rh, u, true_x);
         */
        //car_state = state_estimator.getstate();
        if(sensordata[13] != 0){
            current_lng = sensordata[13];
        }
        if(sensordata[11] != 0 && current_heading != sensordata[11]*pi/180){
            old_heading = current_heading;
            current_heading = sensordata[11]*pi/180;
            old_heading_time = current_heading_time;
            current_heading_time = t;
        }
        if(sensordata[14] != 0){
            current_speed = sensordata[14];
        }
        //visual pipeline and SLAM
        if(camL3_frnr != old_camL3_frnr && camL3_frnr > 1280 && camL3_frnr < 2611){ // new camera image -> do visual pipeline and slam
            old_camL3_frnr = camL3_frnr;
            std::vector<std::tuple<int, distheading>> vp_det = VisualPipeline::sim_get_relative_cone_positions(camL3_frnr, 0);
            double yawrate = 0;
            if(old_heading != 0 && current_heading != 0){  // only compute yawrate if both heading values are valid.
                yawrate = to_range(current_heading-old_heading)/(current_heading_time-old_heading_time);
            }
            printf("do visual pipeline at t=%f, camL3_frnr=%i\n  speed=%f, yawrate=%f-%f=%f, number_of_detections = %zu\n", t, camL3_frnr, current_speed, current_heading, old_heading, yawrate, vp_det.size());
            slam->add_vpdetections(current_speed, yawrate, vp_det);//current_heading-old_heading
        }
        // new gnss measurement := lat != 0 && lat != old_lat
        if(sensordata[12] != 0 && sensordata[12] != current_lat && camL3_frnr > 1280 && camL3_frnr < 2611){
            if(std::get<0>(gps_base) == 0 && std::get<1>(gps_base) == 0){
                //first gnss constraint will be (0, 0)
                gps_base = {sensordata[12]*pi/180, current_lng*pi/180};
                printf("set gps_base to (%f, %f)\n", std::get<0>(gps_base), std::get<1>(gps_base));
            }
            if(camL3_frnr > 1700){  // gps-measurements bofore this are more wrong than right
                std::tuple<meter, meter> mpos = gps_to_meter(sensordata[12]*pi/180, current_lng*pi/180, std::get<0>(gps_base), std::get<1>(gps_base));
                //printf("add gnss constraint at t=%f, camL3_frnr=%i,gnsspos=(%f, %f), mpos=(%f, %f), v=%f, yaw=%f\n", t, camL3_frnr, sensordata[12]*pi/180, current_lng*pi/180, std::get<0>(mpos), std::get<1>(mpos), current_speed, current_heading);
                pose_ext gps = {std::get<0>(mpos), std::get<1>(mpos), current_speed, current_heading, 0};//lat, long, speed, heading, yawrate
                slam->add_GNSS_mes(gps);
                slam->optimise();
            }
            //slam->optimise();  // buildSystem(): NaN within Jacobian for edge 0x1127aea0 for vertex 0, and chi=nan in final optimise
            current_lat = sensordata[12];
        }

        //controller
        //controls = controler.get(car_state, map);

        //send controls to other programm parts/can?
    }
    printf("\nfinished iterating over sensordata\n");
    slam->print();
    //slam->do_clustering = true;
    slam->save_graph("before");
    slam->full_optimise();
    slam->print();
    slam->save_graph("after");
    printf("\nfinished succesfully\n");
    return 0;
}
