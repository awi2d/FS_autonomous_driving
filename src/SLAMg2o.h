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

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

class SLAM{
private:
    std::string base_path;
    std::string name;
    g2o::SparseOptimizer optimizer;
    Eigen::Matrix3d gnss_information;
    Eigen::Matrix3d odometry_information;
    std::map<int, vertex_metainfo> conevertex_metainfo;  // conevertex_metainfo[conevertex_id] = (negative information, cls)
    g2o::VertexSE2* zero_vertex;
    g2o::VertexPointXY* borangec;
    g2o::VertexPointXY* yorangec;
    int vertex_id;
    int current_carpose_vertex_id;
    int last_carpose_vertex_id;
    int blueside_orangecone_id;
    int yellowside_orangecone_id;
    int sensorOffsetid;
    bool do_dataAssociation;
    bool do_negativeinformation;
public:
    SLAM(bool clustering, bool frontend, bool negativeinformation);
    void add_vpdetections(double velocity, double yawrate, std::vector<std::tuple<int, distheading>> vp_det);  // vp_det are cone positions from visual pipeline, relative to car. TODO add odometry measurment (acceleration and yawrate))
    void add_GNSS_mes(pose_ext gnss_measurement);
    void optimise();
    void full_optimise();
    void save_graph(const std::string& name_suffix);
    void print();
    bool do_clustering;
};

#endif //STATE_ESTIMATOR_SLAMG2O_H
