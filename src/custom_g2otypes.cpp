#include "custom_g2otypes.h"

namespace custom_g2otypes {

    Vertex_carpose::Vertex_carpose() {
        this->camL3_frnr = 0;
    }

    bool Vertex_carpose::read(std::istream &is) {
        Eigen::Vector3d p;
        is >> p[0] >> p[1] >> p[2] >> this->camL3_frnr;
        _estimate.fromVector(p);
        return true;
    }

    bool Vertex_carpose::write(std::ostream &os) const {
        Eigen::Vector3d p = estimate().toVector();
        os << p[0] << " " << p[1] << " " << p[2] << " " << this->camL3_frnr;
        return os.good();
    }

    Edge_odometry::Edge_odometry() : g2o::BaseBinaryEdge<3, g2o::SE2, Vertex_carpose, g2o::VertexPointXY>() {}

    bool Edge_odometry::read(std::istream &is) {
        Eigen::Vector3d p;
        is >> p[0] >> p[1] >> p[2];
        _measurement.fromVector(p);
        _inverseMeasurement = measurement().inverse();
        for (int i = 0; i < 3; ++i)
            for (int j = i; j < 3; ++j) {
                is >> information()(i, j);
                if (i != j) information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool Edge_odometry::write(std::ostream &os) const {
        Eigen::Vector3d p = measurement().toVector();
        os << p.x() << " " << p.y() << " " << p.z();
        for (int i = 0; i < 3; ++i)
            for (int j = i; j < 3; ++j) os << " " << information()(i, j);
        return os.good();
    }

    void Edge_visdet::computeError() {
        const Vertex_carpose* l1 = static_cast<const Vertex_carpose*>(_vertices[0]);
        const g2o::VertexPointXY* l2 = static_cast<const g2o::VertexPointXY*>(_vertices[1]);
        _error = (l1->estimate() * _measurement) - l2->estimate();
    }

    bool Edge_visdet::read(std::istream& is) {
        is >> _measurement[0] >> _measurement[1];
        is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
        information()(1, 0) = information()(0, 1);
        return true;
    }

    bool Edge_visdet::write(std::ostream& os) const {
        os << measurement()[0] << " " << measurement()[1] << " ";
        os << information()(0, 0) << " " << information()(0, 1) << " "
           << information()(1, 1);
        return os.good();
    }
} // namespace custom_g2otypes