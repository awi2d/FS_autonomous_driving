#ifndef VELOCITY_ESTIMATOR_STATEESTIMATOR_H
#define VELOCITY_ESTIMATOR_STATEESTIMATOR_H

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 10, 10> Matrix10d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 5, 10> Matrix5x10d;
typedef Eigen::Matrix<double, 4, 10> Matrix4x10d;
typedef Eigen::Matrix<double, 3, 10> Matrix3x10d;
typedef Eigen::Matrix<double, 2, 10> Matrix2x10d;
typedef Eigen::Matrix<double, 10, 5> Matrix10x5d;
typedef Eigen::Matrix<double, 10, 4> Matrix10x4d;
typedef Eigen::Matrix<double, 10, 3> Matrix10x3d;
typedef Eigen::Matrix<double, 10, 2> Matrix10x2d;

static const double pi = 3.14159265358979323846;
static const Eigen::Matrix<double, 10, 10> eye10 = Eigen::MatrixXd::Identity(10,10);

namespace sI //stateIndex
{
    //KF->state(sI::X) = the current state prediction for value X
    static const int X = 0;  //[m] distance between starting position and current position in north/south direction.
    static const int Y = 1;  //[m] distance between starting position and current position in west/est direction.
    static const int Yaw = 2;//[°] rotation of car in global frame
    static const int vx = 3; //[m/2] velocity of car in forward direction
    static const int vy = 4; //[m/2] velocity of car in sideways direction
    static const int yawrate = 5;//[°/s] derivatative of Yaw, turning speed of car
    static const int ax = 6; //[m/s2] acceleration forwards
    static const int ay = 7; //[m/s2] acceleration sideways
    static const int srl = 8;//[1] slip ratio of rear left wheel
    static const int srr = 9;//[1] slip ratio of rear right wheel
}

inline Eigen::Matrix<double, 1, 10> getOnes(int index){
    // returns a zero Vector of length 10 with a 1 at position index
    Eigen::Matrix<double, 1, 10> r = Eigen::MatrixXd::Zero(1, 10);
    r(0, index) = 1;
    return r;
}

class StateEstimator {
private:
    Vector10d state; // state = [X, Y, Yaw, vx, vy, yawrate, ax, ay, srl, srr]. use sI::name to get indixes.
    Matrix10d p; // covariance of state.
    Eigen::Vector2d gps_startpos = Eigen::MatrixXd::Zero(2, 1); // the starting position of the car. used for state(X) and state(Y)
    bool gps_startpos_notset = true;
    double time = 0;

    const Matrix10d q = 0.05*eye10;  // covariance of state prediction https://de.wikipedia.org/wiki/Kalman-Filter#Gleichungen
    const Matrix5x10d c_gps = (Eigen::MatrixXd(5, 10) << getOnes(sI::X), getOnes(sI::Y), getOnes(sI::Yaw), getOnes(sI::vx), getOnes(sI::yawrate)).finished();
    const Matrix10x5d c_gpst = c_gps.transpose();
    const Matrix5d r_gps = Eigen::MatrixXd::Identity(5,5);
    const Matrix3x10d c_imu = (Eigen::MatrixXd(3,10) << getOnes(sI::ax), getOnes(sI::ay), getOnes(sI::yawrate)).finished();
    const Matrix10x3d c_imut = c_imu.transpose();
    const Eigen::Matrix3d r_imu = Eigen::MatrixXd::Identity(3,3);
    const Matrix4x10d c_rh = (Eigen::MatrixXd(4, 10) << getOnes(sI::vx), getOnes(sI::srl), getOnes(sI::srr), getOnes(sI::yawrate)).finished();
    const Matrix10x4d c_rht = c_rh.transpose();
    const Matrix4d r_rh = Eigen::MatrixXd::Identity(4,4);
    const Matrix2x10d c_u = (Eigen::MatrixXd(2, 10) << getOnes(sI::yawrate), getOnes(sI::ax)).finished();  // this means that the first element of the u mesurment predicts yawrate and the second element ax.
    const Matrix10x2d c_ut = c_u.transpose();
    const Eigen::Matrix2d r_u = (Eigen::MatrixXd(2,2) << 1, 0, 100, 0).finished(); // ax prediction by u bad
    const Matrix2x10d c_ps = (Eigen::MatrixXd(2, 10) << getOnes(sI::vy), getOnes(sI::yawrate)).finished();
    const Matrix10x2d c_pst = c_ps.transpose();

    Vector5d gps_val;  // used to check if gps measurement has changed (<=> new gps measurement) in call.
    Eigen::Vector3d imu_val;
    Eigen::Vector2d rh_val;
    Eigen::Vector2d u_val;

    Vector5d gps_tmp;  // used to store intermediate result when applying gps measurement
    Eigen::Vector4d rh_tmp;
    Eigen::Vector2d u_tmp;

    const double radius = 0.245; //radius der Hinterräder, so dass (ausgabe der Umrichter)*radius = vx
    const double spurweite = 1; //[m] abstand der beiden Hinterräder zueinander
    const double eigenlenkgradient = 0;
    const double radstand = 2; //[m] abstand der beiden Achsen zueinander

    const bool use_gps;  // disable use of gps-mesurment for testing
    const bool use_imu;
    const bool use_rh;
    const bool use_u;
    const bool use_pseudo;
    const bool use_pred;

    std::ofstream logfile;  //#ifdef debugMode
public:
    StateEstimator(bool use_hold, bool use_noise, bool usePred, bool useGps, bool useImu, bool useRh, bool useU, bool usePseudo);
    bool call(double currentTime, const Vector5d& gps, const Vector3d& imu, const Eigen::Vector2d& rh, const Eigen::Vector2d& u, const Vector10d& true_x);
    void doSystemPrediction(double dt);
    void mesurment_pseudo();
    void mesurment_gps(const Vector5d& gps_mes);
    void mesurment_imu(const Vector3d& imu_mes);
    void mesurment_rh(const Eigen::Vector2d& rh_mes);
    void mesurment_u(const Eigen::Vector2d& u_mes);
    void print_state() const;
    ~StateEstimator(){
        this->logfile.close();
        // do Eigen::Matrix have to be deleted?
    }
};


#endif //VELOCITY_ESTIMATOR_STATEESTIMATOR_H
