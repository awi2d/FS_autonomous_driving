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
#include "vertex_se2.h"

#include <sys/stat.h>
#include <algorithm>

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;


typedef double heading;
typedef double meter;
typedef double second;
typedef double radiants;
typedef int droneFrnr;
typedef std::tuple<double, double> pxpos;
typedef std::tuple<int, double, double, double, double> boundingbox;  // (cls, posw, posh, sizew, sizeh)
typedef std::tuple<meter, heading> distheading;
typedef std::vector<pxpos> cone_keypoints; // always has length 7.
typedef std::tuple<meter, meter, heading> pose;
typedef Eigen::Vector2d m_position;  // (meter_north, meter_east)

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
std::vector<boundingbox> get_boundingbox(const std::string& cam, unsigned int framenr)
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

std::map<std::tuple<unsigned int, unsigned int>, std::vector<pxpos>> keypoints_cache;
std::vector<pxpos> get_cone_keypoints(const std::string& cam, unsigned int framenr, unsigned int cone){
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
    std::tuple<unsigned int, unsigned int> key = {framenr, cone};
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

m_position distazimuth_to_meter(meter dist, radiants heading){
    m_position res;
    res << cos(heading)*dist, sin(heading)*dist;
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
            pose v0 = std::get<1>(data[i]);
            pose v1 = std::get<1>(data[i-1]);
            //(w1*y[i-1]+w0*y[i])/sum
            pose res = {(std::get<0>(v0)*w0+std::get<0>(v1)*w1)/sum, (std::get<1>(v0)*w0+std::get<1>(v1)*w1)/sum, (std::get<2>(v0)*w0+std::get<2>(v1)*w1)/sum};
            return res;

        }
    }
    printf("ERROR: unreachable code in get_at_time(data[0].time=%i, data[-1].time=%i, time=%f)", std::get<0>(data[0]), std::get<0>(data[data.size()-1]), time);
    return std::get<1>(data[0]);
}

int main() {
    cerr << "Hallo Welt!" << endl;
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
    //double vp_dist_error = 0.1*dist meter; -> information(0, 0) = 1/(0.1*dist*0.1*dist) = 100/dist
    Eigen::Matrix3d covariance;
    covariance.fill(0.);
    covariance(0, 0) = 0.632*0.632;  // translational noise squared
    covariance(1, 1) = 0.632*0.632;  // translational noise squared
    covariance(2, 2) = 0.0156*0.0156;  // rotational noise squared
    Eigen::Matrix3d gnss_information = covariance.inverse();

    // iterate over sensordata
    std::vector<std::tuple<int, pose>> carposes = get_car_poses();
    //pose old_carpose = get_at_time(carposes, (1280/20.0+29.56)*25.0);
    // init variables for for(camL3_frnr)
    int vertex_id = 1;
    int current_carpose_vertex_id = 1;
    int last_carpose_vertex_id = 0;
    SE2 carpose_se2 = SE2(0, 0, 0);
    SE2 last_carpose_se2 = SE2(0, 0, 0);
    std::vector<std::tuple<Eigen::Vector2d, m_position>> vp_det;  // std::get<0>(vp_det[i]) = dist, heading, std::get<1>(gp_det[i]) = abs mpos of cone
    std::vector<int> conevertex_ids;
    // add initial 0-pose
    current_carpose_vertex_id = vertex_id;
    // add new carpose vertice
    VertexSE2* zero_vertex = new VertexSE2;
    zero_vertex->setId(0);
    SE2 zero_se2 = SE2(0, 0, 0);
    zero_vertex->setEstimate(zero_se2);  // type(initguess) = g2o::SE2
    optimizer.addVertex(zero_vertex);

    /*********************************************************************************
     * creating the optimization problem
     ********************************************************************************/

    for(unsigned int camL3_frnr=1280; camL3_frnr<2643; camL3_frnr++){
        //printf("\nfrnr=%i\n", camL3_frnr);
        std::vector<boundingbox> bbs = get_boundingbox("camL3", camL3_frnr);
        //camL3_frnr/20 = drone3_frnr/25-29.56 -> drone3_frnr = (camL3_frnr/20+29.56)*25
        pose carpose = get_at_time(carposes, (camL3_frnr/20.0+29.56)*25.0);
        vp_det.clear();
        for(unsigned int bbi=0; bbi < bbs.size(); bbi++){
            cone_keypoints kp = get_cone_keypoints("camL3", camL3_frnr, bbi);
            if(kp.size() == 7){
                distheading tmp = customPnP(kp, bbs[bbi]);
                if(std::get<0>(tmp) < 10){
                    Eigen::Vector2d disthead;
                    disthead << std::get<0>(tmp), std::get<1>(tmp);
                    m_position relpos = distazimuth_to_meter(std::get<0>(tmp), std::get<1>(tmp)+std::get<2>(carpose));
                    m_position abspos;
                    abspos << relpos(0)+std::get<0>(carpose), relpos(1)+std::get<1>(carpose);
                    //printf("camL3_frnr=%i, bbi=%i, relpos=(%f, %f), abspos=(%f, %f)\n", camL3_frnr, bbi, relpos(0), relpos(1), abspos(0), abspos(1));
                    //relpos and abspos are correct
                    vp_det.emplace_back(disthead, abspos);
                }//ignore cone detections 10 or more meter away
                //printf("vp_det[%i] = (%f, %f)\n", bbi, std::get<0>(vp_det), std::get<1>(vp_det));
            }

        }//for(bb)

        // add new carpose vertice
        VertexSE2* carpose_vertex = new VertexSE2;
        carpose_vertex->setId(vertex_id);
        current_carpose_vertex_id = vertex_id;
        SE2 carpose_se2 = SE2(std::get<0>(carpose), std::get<1>(carpose), std::get<2>(carpose));
        //TODO set velocity and heading diff as  odometry measurement.
        carpose_vertex->setEstimate(carpose_se2);
        optimizer.addVertex(carpose_vertex);
        vertex_id++;

        // add odometry constraint
        EdgeSE2* odometry_constraing = new EdgeSE2;
        odometry_constraing->vertices()[0] = optimizer.vertex(last_carpose_vertex_id);
        odometry_constraing->vertices()[1] = optimizer.vertex(current_carpose_vertex_id);
        //void EdgeSE2::computeError() {_error = (_inverseMeasurement * (v1->estimate().inverse() * v2->estimate())).toVector;}
        odometry_constraing->setMeasurement(last_carpose_se2.inverse()*carpose_se2);  // measurement ?= SE2(meterdist in direction of travel, meterdist perpendicular to travel, change in angle, gl.36
        odometry_constraing->setInformation(gnss_information);
        optimizer.addEdge(odometry_constraing);

        // add gnss constraint
        EdgeSE2* gnss_constraing = new EdgeSE2;
        gnss_constraing->vertices()[0] = optimizer.vertex(0);
        gnss_constraing->vertices()[1] = optimizer.vertex(current_carpose_vertex_id);
        gnss_constraing->setMeasurement(carpose_se2);  // prev.truePose.inverse() * p.truePose, truePose of type SE2
        gnss_constraing->setInformation(gnss_information);
        optimizer.addEdge(gnss_constraing);

        // slam_frontend
        for(int i=0; i<vp_det.size(); i++){
            Eigen::Vector2d dist_heading = std::get<0>(vp_det[i]);
            m_position abs_conepos = std::get<1>(vp_det[i]);
            int detected_landmark_id = -1;

            // type(last_vp_absmpos) == std::vector<std::tuple<int, meter, meter>>
            for(int conevertex_id : conevertex_ids){
                VertexPointXY* maped_cone = dynamic_cast<VertexPointXY*>(optimizer.vertex(conevertex_id));
                auto maped_cone_pos = maped_cone->estimate();
                //if(dist(abs_conepos, last_vp_absmpos[ii]) < 1){
                if(std::sqrt(std::pow(abs_conepos[0]-maped_cone_pos[0], 2) + std::pow(abs_conepos[1]-maped_cone_pos[1], 2))<1){
                    detected_landmark_id = conevertex_id;
                    break;
                }
            }
            if(detected_landmark_id==-1){
                // add new landmark vertices
                VertexPointXY* landmark = new VertexPointXY;
                landmark->setId(vertex_id);
                detected_landmark_id = vertex_id;
                conevertex_ids.emplace_back(detected_landmark_id);
                landmark->setEstimate(abs_conepos);
                optimizer.addVertex(landmark);
                vertex_id++;
            }

            // add vp constraints
            Eigen::Matrix2d vp_information;
            //vp_information << 100/(0.1+dist_heading(0)), 0, 0, 100/(0.1+dist_heading(0)); // add 0.1 to dist to avoid div by 0. //TODO Cholesky failure, writing debug.txt (Hessian loadable by Octave)
            vp_information << 1, 0, 0, 1;
            EdgeSE2PointXY* vpobs_constraint = new EdgeSE2PointXY;
            vpobs_constraint->vertices()[0] = optimizer.vertex(current_carpose_vertex_id);
            vpobs_constraint->vertices()[1] = optimizer.vertex(detected_landmark_id);
            // error = ( v1−>estimate().inverse() * l2−>estimate()) − measurement;
            //conepos_estimate = carpose->estimate() * _measurement
            m_position rel_mpos_unrotated = distazimuth_to_meter(dist_heading(0), dist_heading(1));  // meter position of landmark, in carpose reference frame
            //assert rel_mpos_unrotated == carpose_se2.inverse() * abs_conepos;
            vpobs_constraint->setMeasurement(rel_mpos_unrotated);
            vpobs_constraint->setInformation(vp_information);
            vpobs_constraint->setParameterId(0, sensorOffset->id());  //https://github.com/RainerKuemmerle/g2o/issues/379
            optimizer.addEdge(vpobs_constraint);
        }//end SLAM front-end

        last_carpose_vertex_id = current_carpose_vertex_id;
        last_carpose_se2 = carpose_se2;
    }//for(camL3_frnr)

    /*********************************************************************************
     * optimization
     ********************************************************************************/

    // dump initial state to the disk
    optimizer.save("tutorial_before.g2o");

    // prepare and run the optimization
    // fix the first robot pose to account for gauge freedom
    VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);
    optimizer.setVerbose(true);

    bool has_gauge_freedom = optimizer.gaugeFreedom();
    if (has_gauge_freedom) {
        printf("has gauge freedom");
        auto* gauge_node = optimizer.findGauge();
        printf("gauge_node.id = %i\n", gauge_node->id());
        gauge_node->setFixed(true);
    }

    cerr << "Optimizing" << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cerr << "done." << endl;

    optimizer.save("tutorial_after.g2o");

    // freeing the graph memory
    optimizer.clear();

    return 0;
}
