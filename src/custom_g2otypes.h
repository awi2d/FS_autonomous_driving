#ifndef STATE_ESTIMATOR_CUSTOM_G2OTYPES_H
#define STATE_ESTIMATOR_CUSTOM_G2OTYPES_H

#include "Eigen/Dense"
#include "g2o_core_api.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"

namespace custom_g2otypes {
    class Vertex_carpose : public g2o::BaseVertex<3, g2o::SE2> {
        //TODO set type of _estimate to something containing velocity and yawrate.
    public:
        int camL3_frnr = 0;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // what does this line do? but its in g2o/examples/tutarial_slam2d/vertexpoint_xy.h
        Vertex_carpose();

        virtual void setToOriginImpl() { _estimate = g2o::SE2(); }

        virtual void oplusImpl(const double *update) {
            g2o::SE2 up(update[0], update[1], update[2]);
            _estimate *= up;
        }

        virtual bool read(std::istream &is);

        virtual bool write(std::ostream &os) const;
    };

    class Edge_odometry : public g2o::BaseBinaryEdge<3, g2o::SE2, Vertex_carpose, g2o::VertexPointXY> {
        //change measurement from g2o::SE2 to seconds, and read speed and yawrate from v1 in computeError
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Edge_odometry();

        void computeError() {
            const Vertex_carpose *v1 = static_cast<const Vertex_carpose *>(_vertices[0]);
            const Vertex_carpose *v2 = static_cast<const Vertex_carpose *>(_vertices[1]);
            g2o::SE2 delta = _inverseMeasurement * (v1->estimate().inverse() * v2->estimate());
            _error = delta.toVector();
        }

        void setMeasurement(const g2o::SE2 &m) {
            _measurement = m;
            _inverseMeasurement = m.inverse();
        }

        virtual bool read(std::istream &is);

        virtual bool write(std::ostream &os) const;

    protected:
        g2o::SE2 _inverseMeasurement;
    };

class Edge_visdet : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, Vertex_carpose, g2o::VertexPointXY> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Edge_visdet();

        void computeError();

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

};

#endif //STATE_ESTIMATOR_CUSTOM_G2OTYPES_H
}//namespace custom_g2otypes