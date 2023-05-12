#ifndef STATE_ESTIMATOR_SLAMG2O_H
#define STATE_ESTIMATOR_SLAMG2O_H

#include <vertex_se2.h>
#include <sparse_optimizer.h>
#include <block_solver.h>
#include <eigen/linear_solver_eigen.h>
#include <vertex_point_xy.h>
#include "string"
#include "map"
#include "util.h"
#include "custom_g2otypes.h"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

class SLAM{
private:
    std::string name;// for saving
    second last_time;
    g2o::SparseOptimizer optimizer;
    Eigen::Matrix<double, 6, 6> gnss_information;  // 1/error in gnss measurement [position north [m], position east [m], heading [radiants], vx [m/s], vy [m/s], yawrate [radiants/s]]
    Eigen::Matrix<double, 6, 6> gnss_information2;  // same as gnss_information, but higher error for position measurement.
    Eigen::Matrix<double, 6, 6> odometry_information;  // 1/error for odometry "masurements" (currently no real mesurements, only uses information from carstates-vertices
    Eigen::Matrix<double, 5, 5> gnsspos_information;// used for gnss measurement before moving when heading is not available
    custom_g2otypes::Vertex_conepos* borangec;  // Orange cone on the blue(left) side of the track
    custom_g2otypes::Vertex_conepos* yorangec;  // Orange cone on the yellwo(right) side of the track
    int vertex_id;  // id that next vertex to add to the optimiser should have. (just an incrementing counter to avoid giving the same id twice)
    int current_carpose_vertex_id;  // id of the carpose_vertex where the visual pipeline detections should be added to.
    int last_carpose_vertex_id;
    const int blueside_orangecone_id = 0;
    const int yellowside_orangecone_id = 1;
    const int sensorOffsetid = 0; // sensor offset is 0 in all directions, but g2o requires it to be set.
    const bool do_dataAssociation;  // disable features for easier debugging
    const bool do_negativeinformation;
    const bool do_clustering;
    custom_g2otypes::Vertex_carpose* get_vertexcarpose(int id);
    custom_g2otypes::Vertex_conepos* get_vertexconepos(int id);
public:
    SLAM(bool clustering, bool frontend, bool negativeinformation);
    void add_vpdetections(second time, int camL3_frnr, std::vector<std::tuple<Color, distheading>> vp_det);  // vp_det are cone positions from visual pipeline, relative to car. TODO add odometry measurment (acceleration and yawrate))
    void add_GNSS_mes(pose_ext gnss_measurement, int camL3_frnr);
    void add_GNSSpos_mes(std::tuple<meter, meter> mpos, int camL3_frnr);
    void optimise();
    void full_optimise();
    void save_graph(const std::string& name_suffix);
    void print();

};

#endif //STATE_ESTIMATOR_SLAMG2O_H
