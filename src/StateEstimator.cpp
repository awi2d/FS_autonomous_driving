#define debugMode true
#include <Eigen/Dense>
#if debugMode
#include <chrono>  //mesuring time
#include <fstream>  // reading file
#include <iomanip>
#include <thread>
#endif

#include "StateEstimator.h"


#if debugMode
#define B2S(x) (x ? "True" : "False")
//VectorX2tab turns a Vector10d into a string, like ", ".join(v:Vector10d) would do
#define Vector22tab(v) string_format("%f, %f, ", v(0), v(1))
#define Vector32tab(v) string_format("%f, %f, %f, ", v(0), v(1), v(2))
#define Vector42tab(v) string_format("%f, %f, %f, %f, ", v(0), v(1), v(2), v(3))
#define Vector52tab(v) string_format("%f, %f, %f, %f, %f, ", v(0), v(1), v(2), v(3), v(4))
#define Vector102tab(v) string_format("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, ", v(0), v(1), v(2), v(3), v(4), v(5), v(6), v(7), v(8), v(9))
#endif

//debug/test methods
#ifdef debugMode

//string_format copied from https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}
#endif

//start state_estimator

StateEstimator::StateEstimator(bool usePred, bool useGps, bool useImu, bool useRh, bool useU, bool usePseudo) :
state(Eigen::MatrixXd::Zero(10, 1)), p(Eigen::MatrixXd::Constant(10, 10, 1)), use_gps{useGps}, use_imu{useImu}, use_rh{useRh}, use_u{useU}, use_pseudo{usePseudo}, use_pred{usePred}{
    //state = Eigen::MatrixXd::Zero(10, 1);
    //p = Eigen::MatrixXd::Constant(10, 10, 1);
    this->gps_val = Eigen::MatrixXd::Constant(5, 1, 0);
    this->imu_val = Eigen::MatrixXd::Constant(3, 1, 0);
    this->rh_val = Eigen::MatrixXd::Constant(2, 1, 0);
    this->u_val = Eigen::MatrixXd::Constant(2, 1, 0);
    this->gps_tmp << 0, 0, 0, 0, 0;
    this->rh_tmp << 0, 0, 0, 0;
    this->u_tmp << 0, 0;

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream tmpfilename;
    tmpfilename << "../logs/kf_log" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    printf("KF.debugoutput_file = %s \n", tmpfilename.str().c_str());
    this->logfile.open(tmpfilename.str());
    if(!this->logfile.is_open()){
        printf("opening logfile failed, abort run.");
        std::exit(1);
    }
#ifdef debugMode
    this->logfile << "logs from velocity_estimator.kf run at " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << string_format(", use_pred=%s, use_gps=%s, use_imu=%s, use_rh=%s, use_u=%s, use_pseudo=%s.", B2S(this->use_pred), B2S(this->use_gps), B2S(this->use_imu), B2S(this->use_rh), B2S(this->use_u), B2S(this->use_pseudo)) <<"\n";
    this->logfile << "time, gps_c, imu_c, rh_ch, u_chd, ";
    this->logfile << "state_X, state_Y, state_Yaw, state_vx, state_vy, state_yawrate, state_ax, state_ay, state_srl, state_srr, ";
    this->logfile << "true_X, true_Y, true_Yaw, true_vx, true_vy, true_yawrate, true_ax, true_ay, true_srl, true_srr, ";
    this->logfile << "gps_pred_X, gps_pred_Y, gps_pred_Yaw, gps_pred_vx, gps_pred_yawrate, ";
    this->logfile << "imu_pred_ax, imu_pred_ay, imu_pred_yawrate, ";
    this->logfile << "rh_pred_vx, rh_pred_srl, rh_pred_srr, rh_pred_yawrate, ";
    this->logfile << "u_pred_yawrate, u_pred_ax\n";
#endif
}

    // to be called if it is unknown with sensor gave a new value. It will test for changes and call the correct mesurment_ function.
    // If instead a new value of gps/imu/rh/u is given, call the correct mesurment_ function directly
    // returns if the state is belivable
    bool StateEstimator::call(double currentTime, const Vector5d& gps, const Vector3d& imu, const Eigen::Vector2d& rh, const Eigen::Vector2d& u, const Vector10d& true_x){
        //std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
        // TODO 0 = no value
        bool gps_changed = !gps.isZero(0);
        bool imu_changed = !imu.isZero(0);
        bool rh_changed = !rh.isZero(0);
        bool u_changed = !u.isZero(0);
        if(gps_changed || imu_changed || rh_changed || u_changed){
            double dt = currentTime - this->time;
            this->time = currentTime;
            this->doSystemPrediction(dt);
            if(gps_changed){
                this->mesurment_gps(gps);
                this->gps_val = gps;
            }
            if(imu_changed){
                this->mesurment_imu(imu);
                this->imu_val = imu;
            }
            if(rh_changed){
                this->mesurment_rh(rh);
                this->rh_val = rh;
            }
            if(u_changed){
                this->mesurment_u(u);
                this->u_val = u;
            }
#ifdef debugMode
            this->logfile << string_format("%f, %s, %s, %s, %s, ", currentTime, B2S(gps_changed), B2S(imu_changed), B2S(rh_changed), B2S(u_changed));
            this->logfile << Vector102tab(this->state);
            this->logfile << Vector102tab(true_x);
            this->logfile << Vector52tab(this->gps_tmp);
            this->logfile << Vector32tab(imu);
            this->logfile << Vector42tab(this->rh_tmp);
            this->logfile << Vector22tab(this->u_tmp);
            this->logfile << std::endl;
#endif
        }
        //returns if any state variables are out of reasonable bounds <=> something has gone wrong.
        return (abs(this->state(sI::X)) > 1400 || abs(this->state(sI::Y)) > 1400 || abs(this->state(sI::Yaw)) > 60 || abs(this->state(sI::vx)) > 140 || abs(this->state(sI::vy)) > 5 || abs(this->state(sI::yawrate)) > 8 || abs(this->state(sI::ax)) > 300 || abs(this->state(sI::ay)) > 60);
    }

    void StateEstimator::doSystemPrediction(double dt){
        // pridicts how the state should be, given that the current content of this->state is dt seconds old
        Matrix10d A = Eigen::MatrixXd::Zero(10, 10);
        //A(x, y) = f mit s = (1+A)*s => s(x) += s(y)*f
        //dX = cos(yaw)*vx-sin(yaw)*vy
        A(sI::X, sI::vx) = cos(this->state(sI::Yaw));
        A(sI::X, sI::vy) = -sin(this->state(sI::Yaw));
        //dY = sin(yaw)*vx+cos(yaw)*vy
        A(sI::Y, sI::vx) = sin(this->state(sI::Yaw));
        A(sI::Y, sI::vy) = cos(this->state(sI::Yaw));
        //dyaw = yawrate
        A(sI::Yaw, sI::yawrate) = 1;
        // dvx = 0.5*yawrate*vy+0.5*vy*yawrate+ax = yawrate*vy+ax
        A(sI::vx, sI::yawrate) = 0.5 * this->state(sI::vy);
        A(sI::vx, sI::vy) = 0.5 * this->state(sI::yawrate);
        A(sI::vx, sI::ax) = 1;
        //dvy = -0.5*yawrate*vx-0.5*vx*yawrate+ay = -yawrate*vx+ay //TODO check, for current data Yaw = vx*yawrate+ay holds instead (Spalte falsch beschriftet überprüfen)
        A(sI::vy, sI::yawrate) = -0.5 * this->state(sI::vx);
        A(sI::vy, sI::vx) = -0.5 * this->state(sI::yawrate);
        A(sI::vy, sI::ay) = 1;
        //dax, day = 0

        if(this->use_pred){
            A = eye10+A*dt;
        }else{
            A = eye10;
        }

        //dyawrate, dsrl, srr = ? (approx 0)
        Vector10d B = Eigen::MatrixXd::Zero(10, 1);
        this->state = A * this->state + dt * B;
        this->p = A * this->p * A.transpose() + this->q;
        this->mesurment_pseudo();
    }

    void StateEstimator::mesurment_pseudo(){
        //uses that yawrate = 1.45*vy
        Eigen::Matrix<double, 2, 1> pseudo_messung;
        pseudo_messung <<
                       this->state(sI::yawrate) * 0.693, // vy
                this->state(sI::vy) * 1.44; // yawrate
        Eigen::Matrix2d r_pseudo;
        r_pseudo <<
                 this->p(6, 6)*1.2, this->p(5, 6)*1.0,
                this->p(6, 5)*1.0, this->p(6, 6)*1.2;
        if(!this->use_pseudo){
            return;
        }
        Matrix10x2d K = this->p*this->c_pst*(this->c_ps*this->p*this->c_pst+r_pseudo).inverse();
        this->state = this->state+K*(pseudo_messung-this->c_ps*this->state);
        //this->p = (eye10-K*this->c_ps)*this->p; //TODO maybe not optimal
        //TODO maybe add other constraints
        //yawrate = v*lenkwinkel/(Radstand+Eigenlenkgradient*v²)
        //Yaw = vx*yawrate+ay
    }

    void StateEstimator::mesurment_gps(const Vector5d& gps_mes){
        // TODO gps doesnt contain yawrate measurment
        //gps_mes = [lattitude, longitude, speed_over_ground, heading, yawrate]
        if(gps_startpos_notset){
            gps_startpos << gps_mes(0), gps_mes(1);
            gps_startpos_notset = false;
            printf("kf.mesurment_gps: set gps_startpos to %f, %f\n", gps_startpos(0), gps_startpos(1));
        }
        double yaw_mes = gps_mes(4);
        while(yaw_mes+pi < this->state(sI::Yaw)){
            yaw_mes = yaw_mes+2*pi;
        }
        while(yaw_mes > this->state(sI::Yaw) + pi){
            yaw_mes = yaw_mes-2*pi;
        }
        //gps_tmp = [X, Y, Yaw, v, yawrate]
        this->gps_tmp <<
                (gps_mes(0) - this->gps_startpos(0))*111.32, // 6378.13567 = erdumfang in meter/erdumfang in grad radiants //https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
                (gps_mes(1) - this->gps_startpos(1))*cos(this->state(sI::X))*111.32,  // maybe replace this->state(xI::X) with gps_mes(0) if they are always syncron
                gps_mes(3), // heading
                gps_mes(2), // speed over ground
                gps_mes(4);// yawrate
        if(!this->use_gps){
            return;
        }
        Matrix10x5d K = this->p*this->c_gpst*(this->c_gps*this->p*this->c_gpst+this->r_gps).inverse();  //maybe remove this-> for readability
        this->state = this->state+K*(gps_tmp-this->c_gps*this->state);
        this->p = (eye10-K*this->c_gps)*this->p;
    }

    void StateEstimator::mesurment_imu(const Vector3d& imu_mes){
        //imu_mes = [ax, ay, yawrate]
        if(!this->use_imu){
            return;
        }
        Matrix10x3d K = this->p*this->c_imut*(this->c_imu*this->p*this->c_imut+this->r_imu).inverse();
        this->state = this->state+K*(imu_mes-this->c_imu*this->state);
        this->p = (eye10-K*this->c_imu)*this->p;
    }

    void StateEstimator::mesurment_rh(const Eigen::Vector2d& rh_mes){
        //rh_mes = [turning_speed_rear_left_wheel, turning_speed_rear_right_wheel]
        // slip is in range [-1, 1], with {(-1, blockierendes Rad), ((-1, 0), gebremstes Rad) (0, frei rollendes Rad), ((0, 1), angetriebenes Rad), (1, durchdrehendes Rad)}
        this->rh_tmp <<
                     //0.5*rh_mes(0)*this->radius/(this->state(sI::srl) + 1) + 0.5 * rh_mes(1) * this->radius / (this->state(sI::srr) + 1), // vx
                     (0.5*rh_mes(0)+0.5*rh_mes(1))*this->radius, //vx
                std::max(-1.0, std::min(1.0, rh_mes(0)*this->radius/(this->state(sI::vx) - this->spurweite * 0.5) - 1)), // srl
                std::max(-1.0, std::min(1.0, rh_mes(1)*this->radius/(this->state(sI::vx) + this->spurweite * 0.5) - 1)), // srr
                (rh_mes(1)-rh_mes(0))*0.235;
        if(!this->use_rh){
            return;
        }
        Matrix10x4d k = this->p*this->c_rht*(this->c_rh*this->p*this->c_rht+this->r_rh).inverse();
        this->state = this->state+k*(rh_tmp-this->c_rh*this->state);
        this->p = (eye10-k*this->c_rh)*this->p;
    }

    void StateEstimator::mesurment_u(const Eigen::Vector2d& u_mes){
        //u_mes = [Trq_Drive, wheelangel]
        //TODO makes vx and yawrate predictions much worse
        this->u_tmp <<
                    u_mes(1), //yawrate
                u_mes(0)*0.07906001-4.9776714; // ax = trq*0.08-5 not realy good match
        if(!this->use_u){
            return;
        }
        Matrix10x2d K = this->p*this->c_ut*(this->c_u*this->p*this->c_ut+this->r_u).inverse();
        this->state = this->state+K*(u_tmp-this->c_u*this->state);
        this->p = (eye10-K*this->c_u)*this->p;
    }

#ifdef debugMode
    void StateEstimator::print_state() const {
        printf("kf->state = [\n\tX = %f\n\tY = %f\n\tYaw = %f\n\tvx = %f\n\tvy = %f\n\tyawrate = %f\n\tax = %f\n\tay = %f\n\tsrl = %f\n\tsrr = %f\n]\n", this->state(sI::X), this->state(sI::Y), this->state(sI::Yaw), this->state(sI::vx), this->state(sI::vy), this->state(sI::yawrate), this->state(sI::ax), this->state(sI::ay), this->state(sI::srl), this->state(sI::srr));
    }
#endif
