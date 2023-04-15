#include "custom_g2otypes.h"

namespace custom_g2otypes {

//Vertex_carpose
    Vertex_carpose::Vertex_carpose() {
        this->camL3_frnr = 0;
    }
    Vertex_carpose::Vertex_carpose(int frnr) {
        this->camL3_frnr = frnr;
    }
    bool Vertex_carpose::read(std::istream &is) {
        Eigen::Vector3d pos;
        Eigen::Vector3d odo;
        is >> pos[0] >> pos[1] >> pos[2] >> odo[0] >> odo[1] >> odo[2] >> this->camL3_frnr;
        _estimate = {g2o::SE2(pos), g2o::SE2(odo)};
        return true;
    }

    bool Vertex_carpose::write(std::ostream &os) const {
        Eigen::Vector3d pos = std::get<0>(estimate()).toVector();
        Eigen::Vector3d odo = std::get<1>(estimate()).toVector();
        os << pos[0] << " " << pos[1] << " " << pos[2] << " " << odo[0] << " " << odo[1] << " " << odo[2] << " " << this->camL3_frnr;
        return os.good();
    }
//Vertex_conepos
    Vertex_conepos::Vertex_conepos(){
        this->color = 0;
        this->negativeInformation = 0;
    }
    Vertex_conepos::Vertex_conepos(int color){
        this->color = color;
        this->negativeInformation = 0;
    }
    bool Vertex_conepos::read(std::istream& is) {
        is >> _estimate[0] >> _estimate[1] >> this->color >> this->negativeInformation;
        return true;
    }
    bool Vertex_conepos::write(std::ostream& os) const {
        os << this->_estimate[0] << " " << this->_estimate[1] << " " << this->color << " " << this->negativeInformation;
        return os.good();
    }
//Edge_odometry
    Edge_odometry::Edge_odometry() : g2o::BaseBinaryEdge<6, double, Vertex_carpose, Vertex_carpose>() {}

    void Edge_odometry::computeError() {
        const Vertex_carpose *v1 = static_cast<const Vertex_carpose *>(_vertices[0]);
        const Vertex_carpose *v2 = static_cast<const Vertex_carpose *>(_vertices[1]);
        g2o::SE2 v1_posest = std::get<0>(v1->estimate());
        g2o::SE2 v2_posest = std::get<0>(v2->estimate());
        g2o::SE2 v1_odometry = std::get<1>(v1->estimate());
        g2o::SE2 v2_odometry = std::get<1>(v2->estimate());
        g2o::SE2 odo = g2o::SE2(v1_odometry[0]*_measurement, v1_odometry[1]*_measurement, v1_odometry[2]*_measurement);  // distance in meter between positions is speed [m/s]*seconds
        // error between v1_posest*v1_posdist and v2_posest
        g2o::SE2 position_delta = (odo.inverse() * (v1_posest.inverse() * v2_posest)).toVector(); // zero if the difference between posest is equal to odo.
        //car model: yawrate and velocity are constant
        g2o::SE2 odo_delta = (v2_odometry.inverse() * v1_odometry).toVector();
        _error << position_delta[0], position_delta[1], position_delta[2], odo_delta[0], odo_delta[1], odo_delta[2];
    }

    bool Edge_odometry::read(std::istream &is) {
        is >> this->_measurement;
        for (int i = 0; i < 6; ++i)
            for (int j = i; j < 6; ++j) {
                is >> information()(i, j);
                if (i != j) information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool Edge_odometry::write(std::ostream &os) const {
        os << this->_measurement;
        for (int i = 0; i < 6; ++i)
            for (int j = i; j < 6; ++j) os << " " << information()(i, j);
        return os.good();
    }

//Edge_visdet
    Edge_visdet::Edge_visdet() : g2o::BaseBinaryEdge<2, Eigen::Vector2d, Vertex_carpose, Vertex_conepos>() {

    }
    void Edge_visdet::computeError() {
        const Vertex_carpose* l1 = static_cast<const Vertex_carpose*>(_vertices[0]);
        const g2o::VertexPointXY* l2 = static_cast<const g2o::VertexPointXY*>(_vertices[1]);
        //_error = (_sensorCache->w2n() * l2->estimate()) - _measurement;
        g2o::SE2 carpose = std::get<0>(l1->estimate());
        Eigen::Vector2d conepos = l2->estimate();
        //printf("l1->estimate() = %i, (%f, %f, %f), l2->estimate = (%f, %f), _mes = (%f, %f)\n", l1->camL3_frnr, carpose[0], carpose[1], carpose[2], conepos[0], conepos[1], _measurement[0], _measurement[1]);
        Eigen::Vector2d delta = (std::get<0>(l1->estimate()) * _measurement) - l2->estimate(); // maybe?
        _error << delta[0], delta[1];
    }

    bool Edge_visdet::read(std::istream& is) {
        is >> _measurement[0] >> _measurement[1];
        is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
        information()(1, 0) = information()(0, 1);
        return true;
    }

    bool Edge_visdet::write(std::ostream& os) const {
        os << measurement()[0] << " " << measurement()[1] << " ";
        os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1);
        return os.good();
    }

//Edge_gnss_measurement
    Edge_gnss_measurement::Edge_gnss_measurement() : g2o::BaseUnaryEdge<6, type_carpose , Vertex_carpose>(){

    }
    void Edge_gnss_measurement::computeError() {
        const Vertex_carpose* l1 = static_cast<const Vertex_carpose*>(_vertices[0]);
        g2o::SE2 position_delta = (std::get<0>(this->_inverseMeasurement) * std::get<0>(l1->estimate())).toVector();  // _error is 0 if this->_measurement == l1->estimate
        g2o::SE2 odo_delta = (std::get<1>(this->_inverseMeasurement) * std::get<1>(l1->estimate())).toVector();
        _error << position_delta[0], position_delta[1], position_delta[2], odo_delta[0], odo_delta[1], odo_delta[2];
    }
    bool Edge_gnss_measurement::read(std::istream &is) {
        Eigen::Vector3d pos;
        Eigen::Vector3d odo;
        is >> pos[0] >> pos[1] >> pos[2] >> odo[0] >> odo[1] >> odo[2];
        g2o::SE2 mes0 = g2o::SE2(pos);
        g2o::SE2 mes1 = g2o::SE2(odo);
        this->_measurement = {mes0, mes1};
        this->_inverseMeasurement = {mes0.inverse(), mes1.inverse()};
        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                is >> information()(i, j);
                if (i != j) information()(j, i) = information()(i, j);
            }
        }
        return true;
    }

    bool Edge_gnss_measurement::write(std::ostream &os) const {
        Eigen::Vector3d pos = std::get<0>(measurement()).toVector();
        Eigen::Vector3d odo = std::get<1>(measurement()).toVector();
        os << pos(0) << " " << pos(1) << " " << pos(2) << " " << odo(0) << " " << odo(1) << " " << odo(2);
        for (int i = 0; i < 6; ++i)
            for (int j = i; j < 6; ++j) os << " " << information()(i, j);
        return os.good();
    }

    //Edge_gnss_measurement_noheading
    Edge_gnss_measurement_noheading::Edge_gnss_measurement_noheading() : g2o::BaseUnaryEdge<5, std::tuple<double, double>, Vertex_carpose>(){
    }

    void Edge_gnss_measurement_noheading::computeError() {
        const Vertex_carpose* l1 = static_cast<const Vertex_carpose*>(_vertices[0]);
        g2o::SE2 pose = g2o::SE2(std::get<0>(this->_measurement), std::get<1>(this->_measurement), 0);
        g2o::SE2 pos_delta = (pose.inverse()*std::get<0>(l1->estimate())).toVector();
        g2o::SE2 odo = g2o::SE2(0, 0, 0);  // if no heading was recorded, yawrate and speed are 0.
        g2o::SE2 odo_delta = odo.inverse() * std::get<1>(l1->estimate());
        _error << pos_delta[0], pos_delta[1], odo_delta[0], odo_delta[1], odo_delta[2];
    }

    bool Edge_gnss_measurement_noheading::read(std::istream &is) {
        Eigen::Vector2d pos;
        is >> std::get<0>(this->_measurement) >> std::get<1>(this->_measurement);
        is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
        information()(1, 0) = information()(0, 1);
        return true;
    }

    bool Edge_gnss_measurement_noheading::write(std::ostream &os) const {
        os << std::get<0>(this->_measurement) << " " << std::get<1>(this->_measurement) << " " << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1);
        return os.good();
    }
} // namespace custom_g2otypes