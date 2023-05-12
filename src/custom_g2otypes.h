#ifndef STATE_ESTIMATOR_CUSTOM_G2OTYPES_H
#define STATE_ESTIMATOR_CUSTOM_G2OTYPES_H

#include <base_unary_edge.h>
#include "Eigen/Dense"
#include "g2o_core_api.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "cppTypes.h"

typedef std::tuple<g2o::SE2, g2o::SE2> type_carpose;
namespace custom_g2otypes {
    /*
     * Vertex_carpose: pose of the car, (position_north:meter, position_east:meter, heading:radiants):g2o::SE2
     * Vertex_conepos: position of one cone (position_north:meter, position_east:meter):Eigen::Vector2d
     * Edge_odometry: edge between successive Vertex_carpose
     * Edge_visdet: edge between Vertex_carpose and Vertex_conepos
     * Edge_gnss_measurement: unary edge on Vertex_carpose
     */

class Vertex_carpose : public g2o::BaseVertex<6, type_carpose> {
    public:
        int camL3_frnr = 0;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // what does this line do? but its in g2o/examples/tutarial_slam2d/vertexpoint_xy.h
        Vertex_carpose();
        Vertex_carpose(int camL3_frnr);

        virtual void setToOriginImpl() { _estimate = {g2o::SE2(), g2o::SE2()}; }

        virtual void oplusImpl(const double *update) {
            g2o::SE2 up(update[0], update[1], update[2]);
            std::get<0>(_estimate) *= up;
            g2o::SE2 up2(update[3], update[4], update[5]);
            std::get<1>(_estimate) *= up2;
        }

        virtual bool read(std::istream &is);

        virtual bool write(std::ostream &os) const;
    };

    class Vertex_conepos : public g2o::BaseVertex<2, Eigen::Vector2d> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Color color;
    double negativeInformation;
    Vertex_conepos();
    Vertex_conepos(Color color);

    virtual void setToOriginImpl() { _estimate.setZero(); }

    virtual void oplusImpl(const double* update) {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    };

    //g2o::BaseBinaryEdge<number of elements in _error, type of measurement, type of _vertices[0], type of _vertices[1]>
    class Edge_odometry : public g2o::BaseBinaryEdge<6, double, Vertex_carpose, Vertex_carpose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Edge_odometry();

        void computeError();

        void setMeasurement(double time) {
            _measurement = time;
        }

        bool read(std::istream &is);

        bool write(std::ostream &os) const;
    };

class Edge_visdet : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, Vertex_carpose, Vertex_conepos> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Edge_visdet();

        void computeError();

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

};


class Edge_gnss_measurement: public g2o::BaseUnaryEdge<6, type_carpose, Vertex_carpose> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Edge_gnss_measurement();

    void computeError();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void setMeasurement(const type_carpose &m) {
        _measurement = m;
        _inverseMeasurement = {std::get<0>(m).inverse(), std::get<1>(m).inverse()};
    }

protected:
    type_carpose _inverseMeasurement;
};

class Edge_gnss_measurement_noheading: public g2o::BaseUnaryEdge<5, std::tuple<double, double>, Vertex_carpose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Edge_gnss_measurement_noheading();

        void computeError();

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        void setMeasurement(const std::tuple<double, double> &m) {
            //std::tuple<meter, meter> position of car. (odometry is 0, heading is unknown)
            _measurement = m;
        }
    };

}//namespace custom_g2otypes

#endif //STATE_ESTIMATOR_CUSTOM_G2OTYPES_H