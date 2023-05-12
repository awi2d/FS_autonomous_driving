#include <cmath>
#include <iostream>

#include "SLAMg2o.h"

#include "types.h"  // has to be included because otherwise saving doesnt work.
#include "custom_g2otypes.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"

#include <algorithm>
#include "util.h"
#include "cppTypes.h"
#define B2S(x) (x ? "True" : "Flse")

custom_g2otypes::Vertex_carpose* SLAM::get_vertexcarpose(int id){
    return dynamic_cast<custom_g2otypes::Vertex_carpose*>(this->optimizer.vertex(id));
}

custom_g2otypes::Vertex_conepos* SLAM::get_vertexconepos(int id){
    return dynamic_cast<custom_g2otypes::Vertex_conepos*>(this->optimizer.vertex(id));
}


SLAM::SLAM(bool clustering, bool frontend, bool negativeinformation) : do_clustering(clustering), do_dataAssociation(frontend), do_negativeinformation(negativeinformation){
    printf("init SLAM class\n");
    this->name = "slam_";
    std::string seperator = "_";
    this->name += B2S(this->do_clustering) + seperator + B2S(this->do_dataAssociation) + seperator + B2S(this->do_negativeinformation) + seperator;

    // allocating the optimizer
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    this->optimizer.setAlgorithm(solver);
    g2o::ParameterSE2Offset* sensorOffset = new g2o::ParameterSE2Offset();
    sensorOffset->setOffset(g2o::SE2(0, 0, 0));
    sensorOffset->setOffset();
    sensorOffset->setId(this->sensorOffsetid);

    this->optimizer.addParameter(sensorOffset);
    this->optimizer.setVerbose(false); // prints "iteration= 0     chi2= 175.459230        time= 0.0690684         cumTime= 0.0690684      edges= 4912     schur= 0" for each optimisation

    //information matrices
    //some of these values are calculated from the error statistics of the GNSS during the testrun. Most are just set to whatever worked first, so tuning them may improve results.
    this->gnss_information <<
        1.0/0.686, 0, 0, 0, 0, 0, //error of gnss_posn - true_posn
        0, 1.0/0.686, 0, 0, 0, 0, //error of gnss_pose - true_pose
        0, 0, 1.0/0.00823, 0, 0, 0, //error of gnss_heading - true_heading
        0, 0, 0, 1.0/0.0181, 0, 0, // error of gnss_speedoverground - true_velocity
        0, 0, 0, 0, 1.0/(0.01*0.01), 0, // error in true_vy-0  // no real data for vy available, so this is just a guess
        0, 0, 0, 0, 0, 1/0.0257; // error in true_yawrate-(gnss_heading[t]-gnss_heading[t-1])/(gnss_time[t]-gnss_time[t-1])
    //TODO switch between gnss_information and gnss_information2 based on what fits better, like in https://github.com/agpratik/max-mixture
    //I suspect the franenumber after witch the gnss location is accurate will not be known in advance for real scenarios.
    this->gnss_information2 <<  // same as gnss_information, but with much larger position errors (for frames before 1700, whenn the position error of GNSS is much larger)
        1.0/7, 0, 0, 0, 0, 0, //error of gnss_posn - true_posn
        0, 1.0/7, 0, 0, 0, 0, //error of gnss_pose - true_pose
        0, 0, 1.0/0.00823, 0, 0, 0, //error of gnss_heading - true_heading
        0, 0, 0, 1.0/0.0181, 0, 0, // error of gnss_speedoverground - true_velocity
        0, 0, 0, 0, 1.0/(0.01*0.01), 0, // error in true_vy-0  // no real data for vy available, so this is just a guess
        0, 0, 0, 0, 0, 1/0.0257; // error in true_yawrate-(gnss_heading[t]-gnss_heading[t-1])/(gnss_time[t]-gnss_time[t-1])
    this->gnsspos_information <<
        this->gnss_information2(0, 0), 0, 0, 0, 0,
        0, this->gnss_information2(1, 1), 0, 0, 0,
        0, 0, 1/(0.01*0.01), 0, 0, // no heading -> car is not moving -> velocity and yawrate are certanly 0.
        0, 0, 0, 1/(0.01*0.01), 0,
        0, 0, 0, 0, 1/(0.01*0.01);
    this->odometry_information <<
        1.0/0.001, 0, 0, 0, 0, 0, // error in estimated_posn (from previus carpose) - true_posn. Should be 0, as velocity and yawrate should change until the positions match.
        0, 1.0/0.001, 0, 0, 0, 0, // error in estimated_pose (from previus carpose) - true_pose
        0, 0, 1.0/0.0001, 0, 0, 0, // error in estimated_heading (from previus carpose) - true_heading
        0, 0, 0, 1.0/0.0136, 0, 0, // error in (previus_vx-current_vx)=0
        0, 0, 0, 0, 1.0/0.0001, 0, // error in (previus_vy-current_vy)=0  // no real data for vy available, so this is just a guess
        0, 0, 0, 0, 0, 1/0.0857; // error in (previus_yawrate-current_yawrate)=0
    //auxilary variables
    this->vertex_id = 2;  // 2 vertexes (the two orange cones) are added in constructer, the first call should therefore start with vertex_id 2
    this->current_carpose_vertex_id = 3;
    this->last_carpose_vertex_id = 0;

    // vertexes that always exist

    // add orange cone landmarks
    this->borangec = new custom_g2otypes::Vertex_conepos(Color::orange_bs);
    //this->blueside_orangecone_id = 0;
    this->borangec->setId(this->blueside_orangecone_id);
    m_position bs_oc_pos;
    bs_oc_pos << 0, 0;
    this->borangec->setEstimate(bs_oc_pos);
    this->optimizer.addVertex(borangec);

    this->yorangec = new custom_g2otypes::Vertex_conepos(Color::orange_ys);
    //this->yellowside_orangecone_id = 1;
    this->yorangec->setId(this->yellowside_orangecone_id);
    m_position ys_oc_pos;
    ys_oc_pos << 0, 2;
    this->yorangec->setEstimate(ys_oc_pos);
    this->optimizer.addVertex(yorangec);

    this->last_time = 0;  // even if the first call to add_vpdetections has a time much larger than that, it should not matter. (during the first call, last_carpose_vertex is null, so no odometry edge gets added and dt isnt used.

}

void SLAM::add_vpdetections(second time, int camL3_frnr, std::vector<std::tuple<Color, distheading>> vp_det) {
    //call for each new frame. Add newly detected cones as landmarks. Add new carpose vertex (with odometry edge to last carpose vertex). Add visual pipeline edges between new carpose and all detected cones.

    second dt = time-this->last_time;//maybe replace this line with second dt = 0.05;//video has 20 fps
    //printf("dt=%f\n", dt);

    // add new carpose for each new frame
    custom_g2otypes::Vertex_carpose *carpose_vertex = new custom_g2otypes::Vertex_carpose(camL3_frnr);
    carpose_vertex->setId(this->vertex_id);
    this->current_carpose_vertex_id = this->vertex_id;
    std::tuple<g2o::SE2, g2o::SE2> init_estimate = {g2o::SE2(0, 0, 0), g2o::SE2(0, 0, 0)};
    carpose_vertex->setEstimate(init_estimate);
    this->optimizer.addVertex(carpose_vertex);
    this->vertex_id++;


    custom_g2otypes::Vertex_carpose* last_carpose = dynamic_cast<custom_g2otypes::Vertex_carpose*>(optimizer.vertex(this->last_carpose_vertex_id));

    if(last_carpose != nullptr){
        // calculate and set init estimate
        std::tuple<g2o::SE2, g2o::SE2> last_carpose_estimate = last_carpose->estimate(); //g2o::SE2(last_carpose->estimate()[0], last_carpose->estimate()[1], last_carpose->estimate()[2]); // Process finished with exit code -1073741819 (0xC0000005)
        g2o::SE2 last_carpose_odometry = std::get<1>(last_carpose_estimate);
        last_carpose_odometry = g2o::SE2(last_carpose_odometry[0]*dt, last_carpose_odometry[1]*dt, last_carpose_odometry[2]*dt);
        init_estimate = {std::get<0>(last_carpose_estimate) * last_carpose_odometry, std::get<1>(last_carpose_estimate)}; // thats the estimated position when moving with velocity and yawrate for 1/20 seconds, starting at last carpose.
        //printf("SLAM::call init_estiamte = {(%f, %f, %f), (%f, %f, %f)}, number of detections =%zu\n", std::get<0>(init_estimate)[0], std::get<0>(init_estimate)[1], std::get<0>(init_estimate)[2], std::get<1>(init_estimate)[0], std::get<1>(init_estimate)[1], std::get<1>(init_estimate)[2], vp_det.size());
        carpose_vertex->setEstimate(init_estimate);

        // add odometry constraint
        custom_g2otypes::Edge_odometry *odometry_constraing = new custom_g2otypes::Edge_odometry;
        odometry_constraing->vertices()[0] = this->optimizer.vertex(this->last_carpose_vertex_id);
        odometry_constraing->vertices()[1] = this->optimizer.vertex(this->current_carpose_vertex_id);
        //void EdgeSE2::computeError() {_error = (_inverseMeasurement * (v1->estimate().inverse() * v2->estimate())).toVector;}
        odometry_constraing->setMeasurement(dt);
        odometry_constraing->setInformation(odometry_information);
        this->optimizer.addEdge(odometry_constraing);
    }else{
        printf("SLAM->current_carpose_vertex_id = %i is invalid. (no custom_g2otypes::Vertex_carpose*)\n", this->current_carpose_vertex_id);
    }
    if(true) {
        // slam_frontend.
        for (auto &i: vp_det) {
            //i = (cls, (dist, heading));
            Color color = std::get<0>(i);
            distheading dist_heading = std::get<1>(i);
            int detected_landmark_id = -1;  // id of the cone that is nearest to abs_conepos
            if (color == Color::blue || color == Color::yellow) {
                double dist_association = 1.0;  // dist between vertex(detected_landmark_id) and abs_conepos
                // blue or yellow cone -> match with map
                m_position relpos = distazimuth_to_meter(std::get<0>(dist_heading),std::get<1>(dist_heading) + std::get<0>(init_estimate)[2]);
                m_position abs_conepos;
                abs_conepos << relpos(0) + std::get<0>(init_estimate)[0], relpos(1) + std::get<0>(init_estimate)[1];
                if (this->do_dataAssociation) {  //do_dataAssociation: Match detections in current frame to already mapped cones. !do_dataAssociation: add new vertex for every detection. (and merge them later via clustering)
                    // this data association currently uses the gated nearest neighbour approach (is detection is matched to the mapped cone with the shortest distance to the detection)
                    // that can very likely be greatly improved. E.g. read "Data Association in Stochastic Mapping Using the Joint Compatibility Test" from "José Neira and Juan D. Tardós"
                    // how good this data association is can be tested when I finally finish annotating the video recordings with bounding boxes with poi ids.
                    for (int coneid = 0; coneid < this->optimizer.vertices().size(); coneid++) {
                        custom_g2otypes::Vertex_conepos *maped_cone = this->get_vertexconepos(coneid);
                        if (maped_cone != nullptr) {
                            auto maped_cone_pos = maped_cone->estimate();
                            // if(colors are equal && dist(abs_conepos, maped_cone_pos) < dist_association){
                            if (color == this->get_vertexconepos(coneid)->color &&
                                std::pow(abs_conepos[0] - maped_cone_pos[0], 2) +
                                std::pow(abs_conepos[1] - maped_cone_pos[1], 2) <
                                dist_association) {  // x**2 < maxdist <=> x < maxdist**2, when maxdist > 0 and x > 0
                                detected_landmark_id = coneid;
                                dist_association = std::pow(abs_conepos[0] - maped_cone_pos[0], 2) +
                                                   std::pow(abs_conepos[1] - maped_cone_pos[1], 2);
                            }
                        }
                    }

                }
                if (dist_association >= 1) {
                    // no existing cone nearer than 1 meter to the detection -> add new landmark to map
                    //printf("add landmark id=%i, cls=%i\n", vertex_id, color);
                    custom_g2otypes::Vertex_conepos *landmark = new custom_g2otypes::Vertex_conepos();
                    landmark->setId(vertex_id);

                    detected_landmark_id = vertex_id;
                    //printf("landmark->setEstiamte((%f, %f))\n", abs_conepos(0), abs_conepos(1));
                    landmark->setEstimate(abs_conepos);
                    optimizer.addVertex(landmark);
                    vertex_id++;
                } else {
                    printf("SLAM::add_vpdetections: associate cone detection with cone %i\n", detected_landmark_id);
                }

            }else{ // cone is orange
                // all orange cones detections are the same two orange cones, no matter the distance. So the data association is skipped.
                if (color == Color::orange_bs) {
                    detected_landmark_id = this->blueside_orangecone_id;
                }else{
                    if (color == Color::orange_ys) {
                        detected_landmark_id = this->yellowside_orangecone_id;
                    }else{
                        printf("ERROR: invalid color: %i\n", color);
                    }
                }

            }

            // add vp constraints
            this->get_vertexconepos(detected_landmark_id)->negativeInformation += 2 * (1 - 0.1 * std::get<0>(dist_heading));   // double value, cause from every (even redetected cones) 1-0.1*dist gets subtracted
            Eigen::Matrix2d vp_information;
            vp_information <<
                1 / (0.1 + std::pow(0.0537 * std::get<0>(dist_heading), 2)), 0,
                0, 1 / (0.1 + std::pow(0.02813 * std::get<0>(dist_heading), 2));  // add 0.1 to avoid div by 0
            custom_g2otypes::Edge_visdet* vpobs_constraint = new custom_g2otypes::Edge_visdet();
            if(this->get_vertexcarpose(this->current_carpose_vertex_id) == nullptr){
                printf("ERROR for vpobs_constraint: current_carpose_vertex_id is invalid: %i\n", this->current_carpose_vertex_id);
            }
            if(this->get_vertexconepos(detected_landmark_id) == nullptr){
                printf("ERROR for vpobs_constraint: detected_landmark_id=%i is invalid.\n", detected_landmark_id);
            }
            vpobs_constraint->vertices()[0] = this->optimizer.vertex(this->current_carpose_vertex_id);;
            vpobs_constraint->vertices()[1] = this->optimizer.vertex(detected_landmark_id);
            m_position rel_mpos_unrotated = distazimuth_to_meter(std::get<0>(dist_heading), std::get<1>(dist_heading));
            //assert rel_mpos_unrotated == carpose_se2.inverse() * abs_conepos;
            vpobs_constraint->setMeasurement(rel_mpos_unrotated);
            vpobs_constraint->setInformation(vp_information);
            vpobs_constraint->setParameterId(0,this->sensorOffsetid);  //https://github.com/RainerKuemmerle/g2o/issues/379
            optimizer.addEdge(vpobs_constraint);
        }//end SLAM front-end

        //negative information: if a cone should currently be visible, but ist: remove it.
        if (do_negativeinformation) {
            std::vector<int> vertexIdToRemove;
            for (int coneid = 0; coneid < this->optimizer.vertices().size(); coneid++) {
                custom_g2otypes::Vertex_conepos *maped_cone = this->get_vertexconepos(coneid);
                dynamic_cast<custom_g2otypes::Vertex_conepos *>(optimizer.vertex(coneid));
                if (maped_cone != nullptr) {
                    auto maped_cone_pos = maped_cone->estimate();
                    distheading tmp = meter_pose_to_distazimuth(maped_cone_pos, std::get<0>(init_estimate)[0],
                                                                std::get<0>(init_estimate)[1]);
                    meter dist = std::get<0>(tmp);
                    // adapt values of vision range and field of view when both cameras are used.
                    radiants heading = angle_dist(std::get<1>(tmp), std::get<0>(init_estimate)[2] -
                                                                    0.371008495);  // left cam is rotated 0.37 radiants from car heading
                    if (dist < 10 && heading < 0.538131955) {
                        // the cone coneid should be visible from current carpose
                        this->get_vertexconepos(coneid)->negativeInformation -= 1.0 - 0.1 * dist;
                        if (this->get_vertexconepos(coneid)->negativeInformation < -0.1) {
                            printf("remove Vertex %i with position (%f, %f) from graph causen negative information %f, color=%i\n",
                                   coneid, maped_cone_pos(0), maped_cone_pos(1), maped_cone->negativeInformation,
                                   maped_cone->color);
                            vertexIdToRemove.push_back(coneid);
                        }
                    }
                }

            }
            for (int coneid: vertexIdToRemove) {
                if (coneid > 2) {
                    optimizer.removeVertex(optimizer.vertex(coneid), true);
                }
            }
            vertexIdToRemove.clear();
        }
    }
    //upkeep
    last_carpose_vertex_id = current_carpose_vertex_id;
    this->last_time = time;
}


void SLAM::add_GNSS_mes(pose_ext gnss_measurement, int camL3_frnr){
    // add GNSS measurement to current carpose. (does not add any new vertices to map)
    //pose_ext gnss_measurement = {std::get<0>(mpos), std::get<1>(mpos), current_speed, current_heading, 0};//lat, long, speed, heading, yawrate

    g2o::SE2 pose = g2o::SE2(std::get<0>(gnss_measurement), std::get<1>(gnss_measurement), to_range(std::get<3>(gnss_measurement))); //posn, pose, heading
    g2o::SE2 odo = g2o::SE2(std::get<2>(gnss_measurement), 0, std::get<4>(gnss_measurement)); //vx, vy, yawrate
    printf("SLAM::addGNSS gnss_measurement = (%f, %f, %f), (%f, %f, %f)\n", pose[0], pose[1], pose[2], odo[0], odo[1], odo[2]);
    custom_g2otypes::Vertex_carpose *current_carpose = this->get_vertexcarpose(this->current_carpose_vertex_id);
    if(current_carpose != nullptr){
        current_carpose->setEstimate({pose, odo});

        //add prior constraint
        custom_g2otypes::Edge_gnss_measurement *gnss_constraint = new custom_g2otypes::Edge_gnss_measurement();
        gnss_constraint->vertices()[0] = this->optimizer.vertex(this->current_carpose_vertex_id);
        gnss_constraint->setMeasurement({pose, odo});
        if(camL3_frnr < 1700){
            gnss_constraint->setInformation(this->gnss_information2);
        }else{
            gnss_constraint->setInformation(this->gnss_information);
        }
        this->optimizer.addEdge(gnss_constraint);
    }
}

void SLAM::add_GNSSpos_mes(std::tuple<meter, meter> mpos, int camL3_frnr){
    //same as add_GNSS_mes, but the GNSS measurement contains no information about heading -> car hasnt moved since start of GNSS.
    custom_g2otypes::Vertex_carpose *current_carpose = this->get_vertexcarpose(this->current_carpose_vertex_id);
    if(current_carpose != nullptr){
        g2o::SE2 pose = g2o::SE2(std::get<0>(mpos), std::get<1>(mpos), std::get<0>(current_carpose->estimate())[2]);
        current_carpose->setEstimate({pose, std::get<1>(current_carpose->estimate())});

        //add prior constraint
        custom_g2otypes::Edge_gnss_measurement_noheading* gnss_constraint = new custom_g2otypes::Edge_gnss_measurement_noheading();
        gnss_constraint->vertices()[0] = current_carpose;
        gnss_constraint->setMeasurement(mpos);
        if(camL3_frnr > 1700){
            gnss_constraint->setInformation(this->gnsspos_information);
            this->optimizer.addEdge(gnss_constraint);
        }else{
            //gnss_constraint->setInformation(this->gnsspos_information*0.01);
            //this->optimizer.addEdge(gnss_constraint);
        }
    }
}

void SLAM::optimise(){
    printf("SLAM::optimise\n");
    optimizer.initializeOptimization();
    //verify that graph is able to be optimised (maybe remove for better performance in final version?)
    optimizer.verifyInformationMatrices(true);
    printf("optimzer has gauge freedom: %i (should be 0)\n", optimizer.gaugeFreedom());
    optimizer.optimize(2);
    printf("done\n");
}

void SLAM::full_optimise() {
    this->optimizer.setVerbose(true);

    printf("full Optimisng\n");
    optimizer.initializeOptimization();
    optimizer.verifyInformationMatrices(true);
    printf("init done\n");
    optimizer.optimize(20);
    printf("optimization done\n");

    if(this->do_clustering){
        // after loop closure -> car-pose changes -> re-evalute data association
        //merge any two conevertex that are same color and within 1m to one another
        std::vector<custom_g2otypes::Vertex_conepos> conevertexe;
        for (auto &vertex: optimizer.vertices()) {
            custom_g2otypes::Vertex_conepos *vertex_pointxy = dynamic_cast<custom_g2otypes::Vertex_conepos *>(std::get<1>(vertex));
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
                Color color = this->get_vertexconepos(iid)->color;
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
                        if (color == this->get_vertexconepos(ii_id)->color) {
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
            std::vector<int> cluster = std::get<2>(i);
            if (cluster.size() > 1) {
                int resid = cluster[0];
                for (int ii = 1; ii < cluster.size(); ii++) {
                    custom_g2otypes::Vertex_conepos* merged_vertex = this->get_vertexconepos(resid);
                    merged_vertex->negativeInformation += this->get_vertexconepos(cluster[ii])->negativeInformation;
                    optimizer.mergeVertices(optimizer.vertex(resid), optimizer.vertex(cluster[ii]), true);
                }
                Eigen::Vector2d estimate;
                estimate << std::get<0>(i), std::get<1>(i);
                custom_g2otypes::Vertex_conepos* vertex_pointxy = this->get_vertexconepos(resid);
                vertex_pointxy->setEstimate(estimate);
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

}

void SLAM::print(){
    //print information about optimiser:
    int vertex_se2_count =0; int vertex_xy_count=0; int edge_se2_count=0; int edge_se2_xy_count=0; int edge_se2_prior_count = 0; int edge_se2_prior2_count = 0;
    for (auto &vertex: optimizer.vertices()) {
        custom_g2otypes::Vertex_conepos *vertex_pointxy = dynamic_cast<custom_g2otypes::Vertex_conepos *>(std::get<1>(vertex));
        if (vertex_pointxy != nullptr) {
            vertex_xy_count++;
        }
        custom_g2otypes::Vertex_carpose *vertex_se2 = dynamic_cast<custom_g2otypes::Vertex_carpose *>(std::get<1>(vertex));
        if (vertex_se2 != nullptr) {
            vertex_se2_count++;
        }
    }
    for(auto &edge: optimizer.edges()){
        custom_g2otypes::Edge_odometry* edge_se2 = dynamic_cast<custom_g2otypes::Edge_odometry*>(edge);
        if(edge_se2 != nullptr){
            edge_se2_count += 1;
        }
        custom_g2otypes::Edge_visdet* edge_se2_xy = dynamic_cast<custom_g2otypes::Edge_visdet*>(edge);
        if(edge_se2_xy != nullptr){
            edge_se2_xy_count += 1;
        }
        custom_g2otypes::Edge_gnss_measurement* edge_se2_prior = dynamic_cast<custom_g2otypes::Edge_gnss_measurement*>(edge);
        if(edge_se2_prior != nullptr){
            edge_se2_prior_count += 1;
        }
        custom_g2otypes::Edge_gnss_measurement_noheading* edge_se2_prior2 = dynamic_cast<custom_g2otypes::Edge_gnss_measurement_noheading*>(edge);
        if(edge_se2_prior2 != nullptr){
            edge_se2_prior2_count++;
        }
    }
    printf("SLAM->optimiser has\n");
    printf("  %llu vertices (carpose: %i, conepos: %i),\n", this->optimizer.vertices().size(), vertex_se2_count, vertex_xy_count);
    printf("  %llu edges (odometry: %i, gnss: %i, gnssPos: %i, visdet: %i)\n", this->optimizer.edges().size(), edge_se2_count, edge_se2_prior_count, edge_se2_prior2_count, edge_se2_xy_count);
}
