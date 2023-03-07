// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <iostream>

#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "simulator.h"
#include "types_tutorial_slam2d.h"
#include "vertex_point_xy.h"

#include <sys/stat.h>
#include <algorithm>

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;
const double pi = 3.14159265358979323846;

typedef double heading;
typedef double meter;
typedef double second;
typedef double radiants;
typedef double velocity;
typedef double yawrate;
typedef int droneFrnr;
typedef std::tuple<double, double> pxpos;
typedef std::tuple<int, double, double, double, double> boundingbox;  // (cls, posw, posh, sizew, sizeh)
typedef std::tuple<meter, heading> distheading;
typedef std::vector<pxpos> cone_keypoints; // always has length 7.
typedef std::tuple<meter, meter, heading, velocity, yawrate> pose;
typedef Eigen::Vector2d m_position;  // (meter_north, meter_east)
typedef std::tuple<double, int> vertex_metainfo;  // (negative-information score, class 0:Blue, 1:yellow, 2:orange)
typedef unsigned int type_frnr;

//copied from https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exists-using-standard-c-c11-14-17-c
inline bool file_exists(const std::string& name) {
    struct stat buffer{};
    return (stat (name.c_str(), &buffer) == 0);
}

void print_vector(std::vector<int> vec){
    printf(" %zu:[", vec.size());
    for(int i=0; i<vec.size(); i++){
        printf("%i, ", vec[i]);
    }
    printf("]\n");
}
void print_vector(std::vector<double> vec){
    printf(" %zu:[", vec.size());
    for(int i=0; i<vec.size(); i++){
        printf("%f, ", vec[i]);
    }
    printf("]\n");
}

radiants angle_dist(radiants a, radiants b){
    return pi - fabs(fmod(fabs(a - b), 2*pi) - pi);
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
std::vector<boundingbox> get_boundingbox(const std::string& cam, type_frnr framenr, std::string base_path)
{
    // should have the same behavior as
    // def get_boundingboxes(cam: str, framenr: int) -> [cone_bounding_box]:
    std::string filename = base_path;
    filename.append(""+cam+"_bb/"+cam+"_frame_"+std::to_string(framenr)+".txt");
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

std::map<std::tuple<type_frnr, unsigned int>, std::vector<pxpos>> keypoints_cache;
std::vector<pxpos> get_cone_keypoints(const std::string& cam, type_frnr framenr, unsigned int cone, std::string base_path){
    // should have the same behavior as
    // def get_cone_keypoints(cam, framenr, cone) -> [(normalised_px_w, normalised_px_h)]:
    if(keypoints_cache.empty()){
        std::string filename = base_path;
        filename.append("cone_annotations.csv");
        if(file_exists(filename)){
            std::ifstream file(filename);
            std::string line;
            std::string cell;
            while(std::getline(file,line)){
                // line = "/cones/camL3_frame_1297.jpg_cone_17.jpg,0.7255555555555555#0.16666666666666666,0.6055555555555555#0.3466666666666667,0.51#0.5488888888888889,0.39111111111111113#0.7433333333333333,0.7822222222222223#0.36777777777777776,0.7911111111111111#0.6077777777777778,0.7811111111111111#0.7711111111111111"
                std::vector<std::string> line_split = split(line, ',');
                std::vector<std::string> name_split = split(line_split[0], '_');
                // /cones/cone_19028.jpg
                if(name_split.size() == 5){
                    std::string t_cam = name_split[0];
                    int t_frnr = str2int(name_split[2].substr(0, name_split[2].size()-4));  // remove .jpg
                    int t_cone = str2int(name_split[4].substr(0, name_split[4].size()-4));
                    std::vector<pxpos> res {str2pxpos(line_split[1]), str2pxpos(line_split[2]), str2pxpos(line_split[3]), str2pxpos(line_split[4]), str2pxpos(line_split[5]), str2pxpos(line_split[6]), str2pxpos(line_split[7])};
                    std::tuple<int, int> key = {t_frnr, t_cone};
                    keypoints_cache[key] = res;
                }
            }
        }else{
            printf("ERROR: file doesnt exists, change path in get_cone_keypoints to correct location of cone_annotations.csv\n");
        }
    }
    std::tuple<type_frnr, unsigned int> key = {framenr, cone};
    //return keypoints_cache.at(key);  // throws if key is not in map.
    auto it = keypoints_cache.find(key);
    if (it != keypoints_cache.end()) {
        return it->second;
    }
    //printf("ERROR: get_cone_keypoints: no resolt for key = (%i, %i)\n", framenr, cone);
    return std::vector<pxpos>();  // empty if file doesnt exist
}


std::vector<std::tuple<int, pose>> get_car_poses(std::string filename){

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

        std::vector<std::string> line_split = split(line, ',');
        pose carpose = {str2double(line_split[1]), str2double(line_split[2]), str2double(line_split[3]), str2double(line_split[4]), str2double(line_split[5])};// pos_north, pos_east, heading, speed, yawrate
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
            {0.00000000000000000, 0.09931057723962547, 0.18581319480106973, 0.2893731266180254, 0.09931057723962547, 0.18581319480106973, 0.2893731266180254},
            {0.09931057723962547, 0.00000000000000000, 0.08783333333333333, 0.18816666666666668, 0.058826410187308546, 0.126375, 0.20548663807186746},
            {0.18581319480106973, 0.08783333333333333, 0.00000000000000000, 0.1, 0.126375, 0.08119296009798978, 0.156},
            {0.28937312661802540, 0.18816666666666668, 0.1, 0.00000000000000000, 0.20548663807186746, 0.156, 0.1159014116265954},
            {0.09931057723962547, 0.05882641018730854, 0.126375, 0.20548663807186746, 0.00000000000000000, 0.08783333333333333, 0.18816666666666668},
            {0.18581319480106973, 0.12637500000000000, 0.08119296009798978, 0.156, 0.08783333333333333, 0.00000000000000000, 0.1},
            {0.28937312661802540, 0.20548663807186746, 0.156, 0.1159014116265954, 0.18816666666666668, 0.1, 0.00000000000000000}
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

m_position distazimuth_to_meter(meter dist, radiants heading){
    m_position res;
    res << cos(heading)*dist, sin(heading)*dist;
    return res;
}
distheading meter_pose_to_distazimuth(const m_position& pos0, pose pos1){
    //p1+distazimuth_to_meter(meter_pose_to_distazimuth(p0, p1))
    double mv0 = pos0(0)-std::get<0>(pos1);
    double mv1 = pos0(1)-std::get<1>(pos1);
    distheading res {sqrt(mv0*mv0+mv1*mv1), atan2(mv1, mv0)};
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
    for(unsigned int i=1; i<data.size(); i++){
        if(std::get<0>(data[i]) > time){
            if(time == std::get<0>(data[i-1])){
                return std::get<1>(data[i-1]);
            }
            double w0 = abs(time-std::get<0>(data[i-1]));
            double w1 = abs(std::get<0>(data[i])-time);
            double sum = w0+w1;
            w0 = w0/sum;
            w1 = w1/sum;
            pose v0 = std::get<1>(data[i]);
            pose v1 = std::get<1>(data[i-1]);
            //(w1*y[i-1]+w0*y[i])/sum
            pose res = {(std::get<0>(v0)*w0+std::get<0>(v1)*w1), (std::get<1>(v0)*w0+std::get<1>(v1)*w1), (std::get<2>(v0)*w0+std::get<2>(v1)*w1), (std::get<3>(v0)*w0+std::get<3>(v1)*w1), (std::get<4>(v0)*w0+std::get<4>(v1)*w1)};
            return res;

        }
    }
    printf("ERROR: unreachable code in get_at_time(data[0].time=%i, data[-1].time=%i, time=%f)", std::get<0>(data[0]), std::get<0>(data[data.size()-1]), time);
    return std::get<1>(data[0]);
}

std::map<type_frnr, std::tuple<distheading, distheading>> orangecone_cache;
std::tuple<distheading, distheading> get_orangecone_distheading(type_frnr frnr, std::string base_path){
    if(orangecone_cache.empty()){
        std::string filename = base_path;
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
    std::tuple<distheading, distheading> res = {{0, 0}, {0, 0}};  // TODO this is a bad way to represent empty
    return res;
}

int main(int argc, char *argv[]) {
    cerr << "Hallo Welt!" << endl;
    printf("__cplusplus = %ld\n", __cplusplus);//__cplusplus = 201402
    type_frnr speedmult = 1;
    std::string base_path = "C:/Users/Idefix/PycharmProjects/tmpProject/vp_labels/";
    if(argc == 2){
        printf("argv[0] = %s\n", argv[0]);
        printf("argv[1] = %s\n", argv[1]);
        speedmult = str2int(argv[1]);
    }
    if(argc == 3){
        //argv[1] = speedmult
        printf("argv[2] = %s\n", argv[2]);
        base_path = argv[2];
    }
    bool do_clustering = false;
    bool do_frontend = true;
    bool do_negativeinformation = false;
    std::string name = "full_";
    name.append(std::to_string(speedmult));

    /*********************************************************************************
     * init
     ********************************************************************************/

    // init optimizer
    typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
    typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    ParameterSE2Offset* sensorOffset = new ParameterSE2Offset;
    sensorOffset->setOffset();
    sensorOffset->setId(0);
    optimizer.addParameter(sensorOffset);

    //double gnss_meter_error = 0.632 meter;
    //double gnss_heading_error = 0.0156 radiants;
    Eigen::Matrix3d gnss_information;
    gnss_information <<
        1.0/(0.632*0.632), 0, 0, //0.632*0.632;  // translational noise squared
        0, 1.0/(0.632*0.632), 0, //0.632*0.632;  // translational noise squared
        0, 0, 1.0/(0.0156*0.0156);// rotational noise squared
    printf("gnss_information = [\n[%f, %f, %f],\n[%f, %f, %f],\n[%f, %f, %f], \n]\n", gnss_information(0,0), gnss_information(0,1), gnss_information(0,2), gnss_information(1,0), gnss_information(1,1), gnss_information(1,2), gnss_information(2,0), gnss_information(2,1), gnss_information(2,2));
    Eigen::Matrix3d odometry_information;
    odometry_information <<
        1/(0.35*0.35), 0, 0,
        0, 1/(0.01*0.01), 0,  // I am quite sure that the car does not go sideways
        0, 0, 1/(0.3818*0.3818);
    odometry_information *= 10;  // no reasening behind this, the car trajectory just doesnt look smooth enough
    //std::string filename = "C:/Users/Idefix/PycharmProjects/tmpProject/vp_labels/droneview/droneFrnr_trueCarMPose.txt";
    std::string filename = base_path;
    filename.append("droneview/droneFrnr_gnssMPose.txt");
    std::vector<std::tuple<int, pose>> carposes = get_car_poses(filename);

    // init variables for for(camL3_frnr)
    int vertex_id = 3;
    int current_carpose_vertex_id = 3;
    int last_carpose_vertex_id = 0;
    current_carpose_vertex_id = vertex_id;
    //SE2 carpose_se2 = SE2(0, 0, 0);
    SE2 last_carpose_se2 = SE2(0, 0, 0);
    std::vector<std::tuple<int, m_position>> vp_det;  // (class/color, dist_heading)
    std::map<int, vertex_metainfo> conevertex_metainfo;  // conevertex_metainfo[conevertex_id] = (negative information, cls)


    // add and fix the first robot pose to account for gauge freedom
    VertexSE2* zero_vertex = new VertexSE2;
    zero_vertex->setId(0);
    SE2 zero_se2 = SE2(0, 0, 0);
    zero_vertex->setEstimate(zero_se2);  // type(initguess) = g2o::SE2
    zero_vertex->setFixed(true);
    optimizer.addVertex(zero_vertex);
    optimizer.setVerbose(false);

    // add orange cone landmarks
    VertexPointXY* borangec = new VertexPointXY;
    borangec->setId(1);
    int blueside_orangecone_id = 1;
    m_position bs_oc_pos;
    bs_oc_pos << 16, 3;  // true pos = 15.2027, 1.49728
    borangec->setEstimate(bs_oc_pos);
    optimizer.addVertex(borangec);
    vertex_id++;

    VertexPointXY* yorangec = new VertexPointXY;
    yorangec->setId(2);
    int yellowside_orangecone_id = 2;
    m_position ys_oc_pos;
    ys_oc_pos << 13, -1;  // true pos = 13.0089, -0.584126
    yorangec->setEstimate(ys_oc_pos);
    optimizer.addVertex(yorangec);
    vertex_id++;

    /*********************************************************************************
     * creating the optimization problem by iterate over sensordata
     ********************************************************************************/

    for(type_frnr camL3_frnr=1280; camL3_frnr<2643; camL3_frnr += speedmult) {
        //printf("\nfrnr=%i\n", camL3_frnr);
        std::vector<boundingbox> bbs = get_boundingbox("camL3", camL3_frnr, base_path);
        //camL3_frnr/20 = drone3_frnr/25-29.56 -> drone3_frnr = (camL3_frnr/20+29.56)*25
        pose carpose = get_at_time(carposes, (camL3_frnr / 20.0 + 29.56) * 25.0);
        vp_det.clear();
        bool yellow_cones_not_added = true;
        for (unsigned int bbi = 0; bbi < bbs.size(); bbi++) {
            if (std::get<0>(bbs[bbi]) == 0) {  //  || std::get<0>(bbs[bbi]) == 1
                //blue cone
                cone_keypoints kp = get_cone_keypoints("camL3", camL3_frnr, bbi, base_path);
                if (kp.size() == 7) {
                    distheading tmp = customPnP(kp, bbs[bbi]);
                    if (std::get<0>(tmp) < 10) {
                        Eigen::Vector2d disthead;
                        disthead << std::get<0>(tmp), std::get<1>(tmp);

                        //printf("camL3_frnr=%i, bbi=%i, relpos=(%f, %f), abspos=(%f, %f)\n", camL3_frnr, bbi, relpos(0), relpos(1), abspos(0), abspos(1));
                        //relpos and abspos are correct
                        vp_det.emplace_back(std::get<0>(bbs[bbi]), disthead);
                    }//ignore cone detections 10 or more meter away
                    //printf("vp_det[%i] = (%f, %f)\n", bbi, std::get<0>(vp_det), std::get<1>(vp_det));
                }
            } else {
                if (std::get<0>(bbs[bbi]) == 2 && yellow_cones_not_added) {
                    //orange cone, keypoints & vp doesnt work.
                    std::tuple<distheading, distheading> orange_det = get_orangecone_distheading(camL3_frnr, base_path);
                    if (std::get<0>(std::get<0>(orange_det)) > 0) {  // 0 used as error-value
                        yellow_cones_not_added = false;
                        // convert distheading-tupel to Eigen::Vector2d.
                        Eigen::Vector2d bs_oc;
                        bs_oc << std::get<0>(std::get<0>(orange_det)), std::get<1>(std::get<0>(orange_det));
                        Eigen::Vector2d ys_oc;
                        ys_oc << std::get<0>(std::get<1>(orange_det)), std::get<1>(std::get<1>(orange_det));
                        vp_det.emplace_back(2, bs_oc);  // orange cone on the blue side has cls 2
                        vp_det.emplace_back(3, ys_oc);
                    }
                }
            }
        }//for(bb)
        // add new carpose vertice
        //printf("camL3_frnr = %i, add VertexSE2 %i\n", camL3_frnr, vertex_id);
        //printf("vp_det = (%zu)\n", vp_det.size());
        //for(int i=0; i<vp_det.size(); i++){
        //    printf("  vp_det[%i] = (%i, (%f, %f))\n", i, std::get<0>(vp_det[i]), std::get<1>(vp_det[i])(0), std::get<1>(vp_det[i])(1));
        //}
        VertexSE2 *carpose_vertex = new VertexSE2;
        carpose_vertex->setId(vertex_id);
        current_carpose_vertex_id = vertex_id;
        SE2 carpose_se2 = SE2(std::get<0>(carpose), std::get<1>(carpose), std::get<2>(carpose));
        carpose_vertex->setEstimate(carpose_se2);
        optimizer.addVertex(carpose_vertex);
        vertex_id++;

        // add odometry constraint
        if (last_carpose_vertex_id > 0) {  // do not add odometry to first pose
            EdgeSE2 *odometry_constraing = new EdgeSE2;
            odometry_constraing->vertices()[0] = optimizer.vertex(last_carpose_vertex_id);
            odometry_constraing->vertices()[1] = optimizer.vertex(current_carpose_vertex_id);
            //void EdgeSE2::computeError() {_error = (_inverseMeasurement * (v1->estimate().inverse() * v2->estimate())).toVector;}
            SE2 odometry_measurement = SE2(std::get<3>(carpose), 0,
                                           std::get<4>(carpose));  // SE2(speed, 0, yawrate). maybe?
            odometry_constraing->setMeasurement(last_carpose_se2.inverse() *
                                                carpose_se2);  // measurement ?= SE2(meterdist in direction of travel, meterdist perpendicular to travel, change in angle, gl.36
            odometry_constraing->setInformation(odometry_information);
            optimizer.addEdge(odometry_constraing);
        }

        // add gnss constraint
        if (camL3_frnr > 1920) {  // TODO true carpose
            //if calling optimise before first gnss is added: cholosky failure
            EdgeSE2 *gnss_constraing = new EdgeSE2;
            gnss_constraing->vertices()[0] = optimizer.vertex(0);
            gnss_constraing->vertices()[1] = optimizer.vertex(current_carpose_vertex_id);
            gnss_constraing->setMeasurement(carpose_se2);  // prev.truePose.inverse() * p.truePose, truePose of type SE2
            gnss_constraing->setInformation(gnss_information);
            optimizer.addEdge(gnss_constraing);
        }



        // slam_frontend
        for (auto &i: vp_det) {
            int color = std::get<0>(i);
            Eigen::Vector2d dist_heading = std::get<1>(i);
            int detected_landmark_id = -1;  // id of the cone that is nearest to abs_conepos
            if (color == 0 || color == 1) {
                double dist_association = 1.0;  // dist between vertex(detected_landmark_id) and abs_conepos
                // blue or yellow cone -> match with map
                m_position relpos = distazimuth_to_meter(dist_heading(0), dist_heading(1) + std::get<2>(carpose));
                m_position abs_conepos;
                abs_conepos << relpos(0) + std::get<0>(carpose), relpos(1) + std::get<1>(carpose);
                for (auto &it: conevertex_metainfo) {
                    int coneid = it.first;
                    VertexPointXY *maped_cone = dynamic_cast<VertexPointXY *>(optimizer.vertex(coneid));
                    auto maped_cone_pos = maped_cone->estimate();

                    // if(colors are equal && dist(abs_conepos, maped_cone_pos) < dist_association){
                    if (color == std::get<1>(conevertex_metainfo[coneid]) &&
                        std::pow(abs_conepos[0] - maped_cone_pos[0], 2) +
                        std::pow(abs_conepos[1] - maped_cone_pos[1], 2) <
                        dist_association) {  // x**2 < maxdist <=> x < maxdist**2, when maxdist > 0 and x > 0
                        detected_landmark_id = coneid;
                        dist_association = std::pow(abs_conepos[0] - maped_cone_pos[0], 2) +
                                           std::pow(abs_conepos[1] - maped_cone_pos[1], 2);
                    }
                }
                // if(do_frontend){add cone only when nesseray; add vp_constraint}else{always add cone, add vp_constraint}
                if (!do_frontend || dist_association >= 1) {
                    // no existing cone near detection -> add new landmark vertices
                    //printf("add landmark id=%i, cls=%i\n", vertex_id, color);
                    VertexPointXY *landmark = new VertexPointXY;
                    landmark->setId(vertex_id);
                    conevertex_metainfo[vertex_id] = {0, color};  // (negative) information=0 cause it gets set when the vp_constraint is added

                    detected_landmark_id = vertex_id;
                    landmark->setEstimate(abs_conepos);
                    optimizer.addVertex(landmark);
                    vertex_id++;
                }
            }
            // all orange cones detections are the same two orange cones, no matter the distance.
            if (color == 2) {
                detected_landmark_id = blueside_orangecone_id;
            }
            if (color == 3) {
                detected_landmark_id = yellowside_orangecone_id;
            }

            // add vp constraints
            if(do_negativeinformation){
                std::get<0>(conevertex_metainfo[detected_landmark_id]) += 2 * (1 - 0.1 * dist_heading(0));   // double value, cause from every (even redetected cones) 1-0.1*dist gets subtracted
            }else{
                std::get<0>(conevertex_metainfo[detected_landmark_id]) += 1 - 0.1 * dist_heading(0);
            }
            Eigen::Matrix2d vp_information;
            vp_information <<
            1 / (0.1 + std::pow(0.0537 * dist_heading(0), 2)), 0,
            0, 1 / (0.1 + std::pow(0.02813 * dist_heading(0), 2));  // add 0.1 to avoid div by 0
            //vp_information << 1, 0, 0, 1;
            EdgeSE2PointXY *vpobs_constraint = new EdgeSE2PointXY;
            vpobs_constraint->vertices()[0] = optimizer.vertex(current_carpose_vertex_id);
            vpobs_constraint->vertices()[1] = optimizer.vertex(detected_landmark_id);
            m_position rel_mpos_unrotated = distazimuth_to_meter(dist_heading(0), dist_heading(1));  // meter position of landmark, in carpose reference frame
            //assert rel_mpos_unrotated == carpose_se2.inverse() * abs_conepos;
            vpobs_constraint->setMeasurement(rel_mpos_unrotated);
            vpobs_constraint->setInformation(vp_information);
            vpobs_constraint->setParameterId(0, sensorOffset->id());  //https://github.com/RainerKuemmerle/g2o/issues/379
            optimizer.addEdge(vpobs_constraint);
        }//end SLAM front-end

        //negative information: remove dist(car, cone) from every cone that should be visible
        if (do_negativeinformation) {
            std::vector<int> vertexIdToRemove;
            for (auto &it: conevertex_metainfo) {
                int coneid = it.first;
                VertexPointXY *maped_cone = dynamic_cast<VertexPointXY *>(optimizer.vertex(coneid));
                auto maped_cone_pos = maped_cone->estimate();
                distheading tmp = meter_pose_to_distazimuth(maped_cone_pos, carpose);
                meter dist = std::get<0>(tmp);
                // adapt values of vision range and field of view when both cameras are used.
                radiants heading = angle_dist(std::get<1>(tmp), std::get<2>(carpose) -
                                                                0.371008495);  // left cam is rotated 0.37 radiants from car heading
                if (dist < 10 && heading < 0.538131955) {
                    // the cone coneid should be visible from current carpose
                    std::get<0>(conevertex_metainfo[coneid]) -= 1.0 - 0.1 * dist;
                    if(std::get<0>(conevertex_metainfo[coneid]) < -0.1){
                        printf("remove Vertex %i with position (%f, %f) from graph causen negative information %f, color=%i\n", coneid, maped_cone_pos(0), maped_cone_pos(1), std::get<0>(conevertex_metainfo[coneid]), std::get<1>(conevertex_metainfo[coneid]));
                        vertexIdToRemove.push_back(coneid);
                    }
                }
            }
            for (int coneid: vertexIdToRemove) {
                if (coneid > 2) {
                    optimizer.removeVertex(optimizer.vertex(coneid), true);
                    conevertex_metainfo.erase(coneid);
                }
            }
            vertexIdToRemove.clear();
        }
        //upkeep
        last_carpose_vertex_id = current_carpose_vertex_id;
        last_carpose_se2 = carpose_se2;

        /*
         * How to iterate over all vertices of a type:
        printf("optimizer.vertexids = [");
        for(auto & vertex: optimizer.vertices()){
            VertexPointXY* vertex_pointxy = dynamic_cast<VertexPointXY*>(std::get<1>(vertex));
            if(vertex_pointxy != nullptr){
                printf("(\"cone\", %i, %i), ", std::get<0>(vertex), std::get<1>(vertex)->id());
            }else{
                printf("(\"carpose\", %i, %i), ", std::get<0>(vertex), std::get<1>(vertex)->id());
            }

        }
        printf("]\n");
        */

        // optimise

        //optimizer.initializeOptimization();
        //optimizer.optimize(2);

    }//for(camL3_frnr)

    /*********************************************************************************
     * optimization
     ********************************************************************************/

    // dump initial state to the disk
    //optimizer.save("tutorial_before.g2o");

    // prepare and run the optimization
    optimizer.save((name+"_before.g2o").c_str());
    optimizer.setVerbose(true);
    cerr << "Optimizing" << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(7);
    cerr << "done." << endl;

    optimizer.save((name+"_optimised.g2o").c_str());

    // after loop closure -> car-pose changes -> re-evalute data association
    if(do_clustering) {
        //merge any two conevertex that are same color and within 1m to one another
        std::vector<VertexPointXY> conevertexe;
        for (auto &vertex: optimizer.vertices()) {
            VertexPointXY *vertex_pointxy = dynamic_cast<VertexPointXY *>(std::get<1>(vertex));
            if (vertex_pointxy != nullptr) {
                // assert std::get<0>(vertex) == std::get<1>(vertex)->id()
                conevertexe.push_back(*vertex_pointxy);
            }
        }
        std::vector<std::tuple<double, double, std::vector<int>>> to_merge;  // merge all vectors in to_merge[i] with each other
        for (int i = 0; i < conevertexe.size(); i++) {
            int iid = conevertexe[i].id();
            // if i is in no cluster
            bool i_in_cluster = false;
            for (std::tuple<double, double, std::vector<int>> &entry: to_merge) {
                std::vector<int> cluster = std::get<2>(entry);
                i_in_cluster |= std::find(cluster.begin(), cluster.end(), iid) != cluster.end();
            }
            if (!i_in_cluster) {
                // get all cones within 1m of conevertexe[i]
                std::vector<int> cluster;

                double cluster_weight = 1;//std::get<0>(conevertex_metainfo[iid]);
                int color = std::get<1>(conevertex_metainfo[iid]);
                double posn = conevertexe[i].estimate()(0);
                double pose = conevertexe[i].estimate()(1);

                for (int ii = i + 1; ii < conevertexe.size(); ii++) {
                    int ii_id = conevertexe[ii].id();
                    bool ii_in_cluster = false;
                    for (std::tuple<double, double, std::vector<int>> &entry: to_merge) {
                        std::vector<int> icluster = std::get<2>(entry);
                        ii_in_cluster |= std::find(icluster.begin(), icluster.end(), ii_id) != icluster.end();
                    }
                    if (!ii_in_cluster) {
                        if (color == std::get<1>(conevertex_metainfo[ii_id])) {
                            auto vii_pos = conevertexe[ii].estimate();
                            if (std::pow(posn - vii_pos(0), 2) + std::pow(pose - vii_pos(1), 2) < 1) {
                                double vii_w = 1.0;//std::get<0>(conevertex_metainfo[ii]);
                                cluster.push_back(ii_id);
                                posn = (posn * cluster_weight + vii_pos(0) * vii_w) / (cluster_weight + vii_w);
                                pose = (pose * cluster_weight + vii_pos(1) * vii_w) / (cluster_weight + vii_w);
                                cluster_weight += vii_w;
                            }
                        }
                    }
                }

                if (!cluster.empty()) {
                    cluster.push_back(iid);
                    std::tuple<double, double, std::vector<int>> new_entry = {posn, pose, cluster};
                    to_merge.emplace_back(new_entry);
                }
            }
        }

        for (auto & i : to_merge) {
            print_vector(std::get<2>(i));
        }

        for (auto & i : to_merge) {
            std::vector<int> cluster = std::get<2>(i);
            if (cluster.size() > 1) {
                int resid = cluster[0];
                for (int ii = 1; ii < cluster.size(); ii++) {
                    optimizer.mergeVertices(optimizer.vertex(resid), optimizer.vertex(cluster[ii]), true);
                    std::get<0>(conevertex_metainfo[resid]) += std::get<0>(conevertex_metainfo[cluster[ii]]);
                }
                Eigen::Vector2d estimate;
                estimate << std::get<0>(i), std::get<1>(i);
                VertexPointXY *vertex_pointxy = dynamic_cast<VertexPointXY *>(optimizer.vertex(resid));
                vertex_pointxy->setEstimate(estimate);
                vertex_pointxy = dynamic_cast<VertexPointXY *>(optimizer.vertex(resid));
            }
        }


    }
    optimizer.save((name + "_after.g2o").c_str());
    printf("save optimised graph(do_frontend=%i, do_clustering=%i, do_negativinformation=%i) to %s\n", do_frontend, do_clustering, do_negativeinformation, (name + "_after.g2o").c_str());
    // freeing the graph memory
    optimizer.clear();

    // save conevertex_metainfo (negative_information, color)
    ofstream myfile;
    myfile.open ((name+"conevertex_metainfo.txt").c_str());
    for(auto & it : conevertex_metainfo) {
        int coneid = it.first;
        myfile << std::to_string(coneid) << "," << std::to_string(std::get<0>(conevertex_metainfo[coneid])) << "," << std::to_string(std::get<1>(conevertex_metainfo[coneid])) << "\n";
    }
    myfile.close();
    return 0;
}
