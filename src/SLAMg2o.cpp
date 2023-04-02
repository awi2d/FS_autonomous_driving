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

#include "SLAMg2o.h"

#include "g2o_core_api.h"
#include "g2o_stuff_api.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "custom_g2otypes.h"

#include <sys/stat.h>
#include <algorithm>
#include "util.h"

#define B2S(x) (x ? "True" : "Flse")

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

m_position distazimuth_to_meter(meter dist, radiants heading){
    m_position res;
    res << cos(heading)*dist, sin(heading)*dist;
    return res;
}
distheading meter_pose_to_distazimuth(const m_position& pos0, meter pos1_north, meter pos1_east){
    //p1+distazimuth_to_meter(meter_pose_to_distazimuth(p0, p1))
    double mv0 = pos0(0)-pos1_north;
    double mv1 = pos0(1)-pos1_east;
    distheading res {sqrt(mv0*mv0+mv1*mv1), atan2(mv1, mv0)};
    return res;
}



SLAM::SLAM(bool clustering, bool frontend, bool negativeinformation){
    printf("init SLAM class\n");
    this->base_path = "C:/Users/Idefix/PycharmProjects/tmpProject/vp_labels/";
    this->do_clustering = clustering;
    this->do_dataAssociation = frontend;
    this->do_negativeinformation = negativeinformation;
    this->name = "slam_";
    std::string seperator = "_";
    this->name += B2S(this->do_clustering) + seperator + B2S(this->do_dataAssociation) + seperator + B2S(this->do_negativeinformation) + seperator;

    // allocating the optimizer
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    this->optimizer.setAlgorithm(solver);
    g2o::ParameterSE2Offset* sensorOffset = new g2o::ParameterSE2Offset;
    sensorOffset->setOffset();
    this->sensorOffsetid = 0;
    sensorOffset->setId(this->sensorOffsetid);

    this->optimizer.addParameter(sensorOffset);
    this->optimizer.setVerbose(true);

    //information matrices
    this->gnss_information <<
        1.0/(0.632*0.632), 0, 0, //0.632*0.632;  // translational noise squared
        0, 1.0/(0.632*0.632), 0, //0.632*0.632;  // translational noise squared
        0, 0, 1.0/(0.0156*0.0156);// rotational noise squared
    this->odometry_information <<
        1/(0.35*0.35), 0, 0,
        0, 1/(0.01*0.01), 0,  // I am quite sure that the car does not go sideways
        0, 0, 1/(0.3818*0.3818);
    odometry_information *= 10;  // no reasening behind this, the car trajectory just doesnt look smooth enough

    //auxilary variables
    this->vertex_id = 2;  // 2 vertexes (the two orange cones) are added in int, the first call should therefore start with vertex_id 2
    this->current_carpose_vertex_id = 3;
    this->last_carpose_vertex_id = 0;

    // vertexes that always exist

    // add orange cone landmarks
    this->borangec = new g2o::VertexPointXY();
    this->blueside_orangecone_id = 1;
    this->borangec->setId(this->blueside_orangecone_id);
    m_position bs_oc_pos;
    bs_oc_pos << 16, 3;  // true pos = 15.2027, 1.49728
    this->borangec->setEstimate(bs_oc_pos);
    this->optimizer.addVertex(borangec);

    this->vertex_id++;

    this->yorangec = new g2o::VertexPointXY();
    this->yellowside_orangecone_id = 2;
    this->yorangec->setId(this->yellowside_orangecone_id);
    m_position ys_oc_pos;
    ys_oc_pos << 13, -1;  // true pos = 13.0089, -0.584126
    this->yorangec->setEstimate(ys_oc_pos);
    this->optimizer.addVertex(yorangec);
    this->vertex_id++;

}

void SLAM::add_vpdetections(double velocity, double yawrate, std::vector<std::tuple<int, distheading>> vp_det) {
    //call for each new frame
    g2o::SE2 odometry = g2o::SE2(velocity/20, 0, yawrate/20); // 20 = fps

    // get initional estimate
    g2o::VertexSE2* last_carpose = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(this->current_carpose_vertex_id));
    g2o::SE2 last_carpose_estimate = g2o::SE2(0, 0, 0);
    if(last_carpose != nullptr){
        last_carpose_estimate = last_carpose->estimate(); //g2o::SE2(last_carpose->estimate()[0], last_carpose->estimate()[1], last_carpose->estimate()[2]); // Process finished with exit code -1073741819 (0xC0000005)
    }else{
        printf("SLAM->current_carpose_vertex_id = %i is invalid. (no g2o::VertexSE2*)\n", this->current_carpose_vertex_id);
        //TODO set theta to initial orientation of car.
    }
    g2o::SE2 init_estimate = last_carpose_estimate * odometry; // thats the estimated position when moving with velocity and yawrate for 1/20 seconds, starting at last carpose. I hope
    // add new carpose for each new frame
    g2o::VertexSE2 *carpose_vertex = new g2o::VertexSE2();
    carpose_vertex->setId(this->vertex_id);
    this->current_carpose_vertex_id = this->vertex_id;
    printf("SLAM::call init_estiamte = (%f, %f, %f), number of detections =%zu\n", init_estimate[0], init_estimate[1], init_estimate[2], vp_det.size());
    carpose_vertex->setEstimate(init_estimate);
    this->optimizer.addVertex(carpose_vertex);
    this->vertex_id++;

    // add odometry constraint
    if(this->last_carpose_vertex_id > 0){// dont add odometry to first pose.
        g2o::EdgeSE2 *odometry_constraing = new g2o::EdgeSE2;
        odometry_constraing->vertices()[0] = this->optimizer.vertex(this->last_carpose_vertex_id);
        odometry_constraing->vertices()[1] = this->optimizer.vertex(this->current_carpose_vertex_id);
        //void EdgeSE2::computeError() {_error = (_inverseMeasurement * (v1->estimate().inverse() * v2->estimate())).toVector;}
        odometry_constraing->setMeasurement(odometry);  // measurement ?= SE2(meterdist in direction of travel, meterdist perpendicular to travel, change in angle, gl.36
        odometry_constraing->setInformation(odometry_information);
        this->optimizer.addEdge(odometry_constraing);
    }
    // slam_frontend
    for (auto &i: vp_det) {
        //i = (cls, (dist, heading));
        int color = std::get<0>(i);
        distheading dist_heading = std::get<1>(i);
        int detected_landmark_id = -1;  // id of the cone that is nearest to abs_conepos
        if (color == 0 || color == 1) {
            double dist_association = 1.0;  // dist between vertex(detected_landmark_id) and abs_conepos
            // blue or yellow cone -> match with map
            m_position relpos = distazimuth_to_meter(std::get<0>(dist_heading), std::get<1>(dist_heading) + init_estimate[2]);
            m_position abs_conepos;
            abs_conepos << relpos(0) + init_estimate[0], relpos(1) + init_estimate[1];
            if(this->do_dataAssociation){
                for (auto &it: conevertex_metainfo) {
                    int coneid = it.first;
                    g2o::VertexPointXY *maped_cone = dynamic_cast<g2o::VertexPointXY *>(optimizer.vertex(coneid));
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

            }
            // if(do_dataAssociation){add cone only when necesseray}else{always add cone}; add vp_constraint
            if (dist_association >= 1) {
                // no existing cone near detection -> add new landmark vertices
                //printf("add landmark id=%i, cls=%i\n", vertex_id, color);
                g2o::VertexPointXY* landmark = new g2o::VertexPointXY();
                landmark->setId(vertex_id);
                conevertex_metainfo[vertex_id] = {0, color};  // (negative) information=0 cause it gets set when the vp_constraint is added

                detected_landmark_id = vertex_id;
                //printf("landmark->setEstiamte((%f, %f))\n", abs_conepos(0), abs_conepos(1));
                landmark->setEstimate(abs_conepos);
                optimizer.addVertex(landmark);
                vertex_id++;
            }else{
                printf("SLAM::add_vpdetections: associate cone detection with cone %i\n", detected_landmark_id);
            }

        }
        // all orange cones detections are the same two orange cones, no matter the distance.
        if (color == 2) {
            detected_landmark_id = this->blueside_orangecone_id;
        }
        if (color == 3) {
            detected_landmark_id = this->yellowside_orangecone_id;
        }

        // add vp constraints
        if(do_negativeinformation){
            std::get<0>(conevertex_metainfo[detected_landmark_id]) += 2 * (1 - 0.1 * std::get<0>(dist_heading));   // double value, cause from every (even redetected cones) 1-0.1*dist gets subtracted
        }
        Eigen::Matrix2d vp_information;
        vp_information <<
            1 / (0.1 + std::pow(0.0537 * std::get<0>(dist_heading), 2)), 0,
            0, 1 / (0.1 + std::pow(0.02813 * std::get<0>(dist_heading), 2));  // add 0.1 to avoid div by 0
        //vp_information << 1, 0, 0, 1;
        g2o::EdgeSE2PointXY *vpobs_constraint = new g2o::EdgeSE2PointXY();
        vpobs_constraint->vertices()[0] = this->optimizer.vertex(this->current_carpose_vertex_id);
        vpobs_constraint->vertices()[1] = this->optimizer.vertex(detected_landmark_id);
        m_position rel_mpos_unrotated = distazimuth_to_meter(std::get<0>(dist_heading), std::get<1>(dist_heading));  // meter position of landmark, in carpose reference frame
        //assert rel_mpos_unrotated == carpose_se2.inverse() * abs_conepos;
        vpobs_constraint->setMeasurement(rel_mpos_unrotated);
        vpobs_constraint->setInformation(vp_information);
        vpobs_constraint->setParameterId(0, this->sensorOffsetid);  //https://github.com/RainerKuemmerle/g2o/issues/379
        optimizer.addEdge(vpobs_constraint);
    }//end SLAM front-end

    //negative information: remove dist(car, cone) from every cone that should be visible
    if (do_negativeinformation) {
        std::vector<int> vertexIdToRemove;
        for (auto &it: conevertex_metainfo) {
            int coneid = it.first;
            g2o::VertexPointXY* maped_cone = dynamic_cast<g2o::VertexPointXY *>(optimizer.vertex(coneid));
            auto maped_cone_pos = maped_cone->estimate();
            distheading tmp = meter_pose_to_distazimuth(maped_cone_pos, init_estimate[0], init_estimate[1]);
            meter dist = std::get<0>(tmp);
            // adapt values of vision range and field of view when both cameras are used.
            radiants heading = angle_dist(std::get<1>(tmp), init_estimate[2] - 0.371008495);  // left cam is rotated 0.37 radiants from car heading
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
}


void SLAM::add_GNSS_mes(pose_ext gnss_measurement){
    //pose_ext gnss_measurement = {std::get<0>(mpos), std::get<1>(mpos), current_speed, current_heading, 0};//lat, long, speed, heading, yawrate

    //add prior constraint. why the fuck is this not working?
    g2o::EdgeSE2Prior* gnss_constraint = new g2o::EdgeSE2Prior;
    gnss_constraint->vertices()[0] = this->optimizer.vertex(this->current_carpose_vertex_id);
    g2o::SE2 tmp = g2o::SE2(std::get<0>(gnss_measurement), std::get<1>(gnss_measurement), to_range(std::get<3>(gnss_measurement)));
    printf("SLAM::addGNSS gnss_measurement = (%f, %f, %f)\n", tmp[0], tmp[1], tmp[2]);
    gnss_constraint->setMeasurement(tmp);  // error = tmp.inverse() * v1->estimate(); -> error is minimal if tmp == v1->estimate;
    gnss_constraint->setInformation(this->gnss_information);
    unsigned long long int prior_edges_count = this->optimizer.edges().size();
    this->optimizer.addEdge(gnss_constraint);
    printf("prior_edges_count = %llu, post_edges_count = %llu\n", prior_edges_count, this->optimizer.edges().size());//number of edges didnt change
}

void SLAM::optimise(){
    printf("SLAM::optimise\n");
    optimizer.initializeOptimization();
    optimizer.optimize(2);
    printf("done\n");
}

void SLAM::full_optimise() {

    printf("full Optimisng\n");
    optimizer.initializeOptimization();
    printf("init done\n");
    optimizer.optimize(20);
    printf("optimization done\n");

    if(this->do_clustering){
        // after loop closure -> car-pose changes -> re-evalute data association
        //merge any two conevertex that are same color and within 1m to one another
        std::vector<g2o::VertexPointXY> conevertexe;
        for (auto &vertex: optimizer.vertices()) {
            g2o::VertexPointXY *vertex_pointxy = dynamic_cast<g2o::VertexPointXY *>(std::get<1>(vertex));
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

        //for (auto & i : to_merge) {
            //print_vector(std::get<2>(i));
        //}

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
                g2o::VertexPointXY *vertex_pointxy = dynamic_cast<g2o::VertexPointXY *>(optimizer.vertex(resid));
                vertex_pointxy->setEstimate(estimate);
                vertex_pointxy = dynamic_cast<g2o::VertexPointXY *>(optimizer.vertex(resid));
            }
        }
        printf("clustering done.\n");
    }

}


//optimizer.clear();// freeing the graph memory should be called at some point, maybe?
void SLAM::save_graph(const std::string& namesuffix) {
    std::string savename = name + "_" +namesuffix+".g2o";
    printf("save graph(do_dataAssociation=%i, do_clustering=%i, do_negativinformation=%i) with %llu vertices and %llu edges to %s\n", do_dataAssociation, do_clustering, do_negativeinformation, this->optimizer.vertices().size(), this->optimizer.edges().size(), savename.c_str());

    optimizer.save(savename.c_str());

    printf("write metainfo\n");
    // save conevertex_metainfo (negative_information, color)
    std::ofstream myfile;
    myfile.open ((name+"_"+namesuffix+"_conevertex_metainfo.txt").c_str());
    for(auto & it : conevertex_metainfo) {
        int coneid = it.first;
        myfile << std::to_string(coneid) << "," << std::to_string(std::get<0>(conevertex_metainfo[coneid])) << "," << std::to_string(std::get<1>(conevertex_metainfo[coneid])) << "\n";
    }
    myfile.close();
}

void SLAM::print(){
    //print information about optimiser:
    int vertex_se2_count =0; int vertex_xy_count=0; int edge_se2_count=0; int edge_se2_xy_count=0; int edge_se2_prior_count = 0;
    for (auto &vertex: optimizer.vertices()) {
        g2o::VertexPointXY *vertex_pointxy = dynamic_cast<g2o::VertexPointXY *>(std::get<1>(vertex));
        if (vertex_pointxy != nullptr) {
            vertex_xy_count++;
        }
        g2o::VertexSE2 *vertex_se2 = dynamic_cast<g2o::VertexSE2 *>(std::get<1>(vertex));
        if (vertex_se2 != nullptr) {
            vertex_se2_count++;
        }
    }
    for(auto &edge: optimizer.edges()){
        g2o::EdgeSE2* edge_se2 = dynamic_cast<g2o::EdgeSE2*>(edge);
        if(edge_se2 != nullptr){
            edge_se2_count += 1;
        }
        g2o::EdgeSE2PointXY* edge_se2_xy = dynamic_cast<g2o::EdgeSE2PointXY*>(edge);
        if(edge_se2_xy != nullptr){
            edge_se2_xy_count += 1;
        }
        g2o::EdgeSE2Prior* edge_se2_prior = dynamic_cast<g2o::EdgeSE2Prior*>(edge);
        if(edge_se2_prior != nullptr){
            edge_se2_prior_count += 1;
        }
    }
    printf("SLAM->optimiser has\n");
    printf("  %llu vertices (SE2: %i, XY: %i),\n", this->optimizer.vertices().size(), vertex_se2_count, vertex_xy_count);
    printf("  %llu edges (SE2: %i, SE2_prior: %i, SE2_XY: %i)\n", this->optimizer.edges().size(), edge_se2_count, edge_se2_prior_count, edge_se2_xy_count);
}
