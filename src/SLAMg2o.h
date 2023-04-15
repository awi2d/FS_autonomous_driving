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
    std::string base_path;
    std::string name;
    second last_time;
    g2o::SparseOptimizer optimizer;
    Eigen::Matrix<double, 6, 6> gnss_information;
    Eigen::Matrix<double, 6, 6> odometry_information;
    Eigen::Matrix<double, 5, 5> gnsspos_information;
    custom_g2otypes::Vertex_conepos* borangec;
    custom_g2otypes::Vertex_conepos* yorangec;
    int vertex_id;
    int current_carpose_vertex_id;
    int last_carpose_vertex_id;
    int blueside_orangecone_id;
    int yellowside_orangecone_id;
    int sensorOffsetid;
    bool do_dataAssociation;
    bool do_negativeinformation;
    custom_g2otypes::Vertex_carpose* get_vertexcarpose(int id);
    custom_g2otypes::Vertex_conepos* get_vertexconepos(int id);
public:
    SLAM(bool clustering, bool frontend, bool negativeinformation);
    void add_vpdetections(second time, int camL3_frnr, std::vector<std::tuple<int, distheading>> vp_det);  // vp_det are cone positions from visual pipeline, relative to car. TODO add odometry measurment (acceleration and yawrate))
    void add_GNSS_mes(pose_ext gnss_measurement, int camL3_frnr);
    void add_GNSSpos_mes(std::tuple<meter, meter> mpos, int camL3_frnr);
    void optimise();
    void full_optimise();
    void save_graph(const std::string& name_suffix);
    void print();
    bool do_clustering;
};

#endif //STATE_ESTIMATOR_SLAMG2O_H
