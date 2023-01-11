/*
 * velocity_estimator.main:
 * class KF:
 *  construcotr: no arguments.
 *   (currently it has arguments to disable the use of specific mesurments/prediction, but thats for debug purposes and will be removed in final version)
 *
 *  call should be used when all last mesurments can be accesed, but it is unknown which mesurments updated since last tick.
 *  if it is known witch mesurment updated, mesurment_Mess should be called directly (with Mess element of {gps, imu, rh, u}.
 *
 *  bool call(double currentTime, const Vector5d& gps, const Vector3d& imu, const Eigen::Vector2d& rh, const Eigen::Vector2d& u, const Vector10d& true_x)
 *   params:
 *    currentTime: the time the new mesurment was taken, in sekonds. Should be greater or equal than the greatest currentTime given to any KF funktion before.
 *    calls doSystemPrediction and mesurment_Mess(currentTime, Mess), if Mess has changed from last call, for Mess in {gps, imu, rh, u}
 *
 *  private void doSystemPrediction(double dt)
 *    should not be called directly.
 *    dt: time since last call to doSystemPrediction in seconds. (should be greater or equal to 0)
 *    sets the state to the best estimate, given the state from dt seconds ago.
 *
 *  void mesurment_gps(const Vector5d& gps_mes)
 *    gps_mes = [lattitude, longitude, speed_over_ground, heading, yawrate]
 *     assuming heading = 0 := north, heading = 90 := east
 *
 *  void mesurment_imu(const Vector3d& imu_mes){
 *    imu_mes = [ax, ay, yawrate]
 *
 *  void mesurment_rh(const Eigen::Vector2d& rh_mes){
 *    rh_mes = [turning_speed_rear_left_wheel, turning_speed_rear_right_wheel]
 *     so that turning_speed_rear_X_wheel * KF.radius = vx of car (assuming zero slip)
 *
 *   void mesurment_u(const Eigen::Vector2d& u_mes){
 *     u_mes = [Trq_Drive, wheelangel]
 *      assuming Trq_Drive is the Torque of the motors and wheelangle is the angle of the steering wheel. TODO add constant, so that wheelangle*constant = yawrate
 */

/*
//split method from https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
std::vector<std::string> split(const std::string& str, const std::string& regex_str)
{
    std::regex regexz(regex_str);
    std::vector<std::string> list(std::sregex_token_iterator(str.begin(), str.end(), regexz, -1), std::sregex_token_iterator());
    return list;
}

struct Data{
    double time;
    Vector5d gps; //gps = [lattitude, longitude, speed_over_ground, heading, yawrate]
    Eigen::Vector3d imu; // ax, ay, yawrate
    Eigen::Vector2d rh; //wheelspeed_RL, wheelspeed_RR
    Eigen::Vector2d u; // Trq_Drive ~ ax, wheelangel ~ yawrate
    Vector10d true_state; //same datatype as state: [X, Y, yaw, vx, vy, yawrate, ax, ay, srl, srr]
};
typedef struct Data Data;

Data* read_realdata(std::vector<std::string> line, Data* r){
    double tmpd[line.size()];
    for(int i=0; i<line.size(); i++){
        tmpd[i] = std::stod(line[i]);
    }
    r->time = tmpd[0];
    //TODO
    return r;
}

double lastTime_simdata = 0;
double yawrate = 0;
double yaw = 0;

//TODO rename X_noise with X_lastmes, cause it gets used to store the last mesurment of X, optionaly with noise.
Vector5d gps_noise = Eigen::MatrixXd::Zero(5, 1);
Eigen::Vector3d imu_noise = Eigen::MatrixXd::Zero(3, 1);
Eigen::Vector2d rh_noise = Eigen::MatrixXd::Zero(2, 1);
Eigen::Vector2d u_noise = Eigen::MatrixXd::Zero(2, 1);
Data* read_simdata(std::vector<std::string> line, Data* r, bool use_noise, bool use_hold){
    /sternchen
    Data format in simdata_slamom.txt
    PoI := Point of Interest = ?anhängerkupplung? vom Auto
    Fr0 := Global frame, (0, 0) ist startpunkt vom Auto, x-richtung ist konstant.
    Fr1 := vehicle frame, (0, 0) ist ?fixer Punkt am Auto (?CoG)?, ausrichtung ist vermutlich Ausrichtung vom Auto.
	    (wenn (0, 0) als fixer Auto deffiniert ist müsste die linearen geschwindigkeit des Autos immer 0 sein.)
        (wenn die Richtung fix zum Auto deffiniert ist, müssten die winkelgeschwindigkeiten immer 0 sin.)

    0 time
    1 Car.WheelSpd_RL	[rad/s] of wheel in wheel frame
    2 Car.WheelSpd_RR	[rad/s] of wheel in wheel frame
    3 Car.WheelSpd_FL	[rad/s] of wheel in wheel frame
    4 Car.WheelSpd_FR	[rad/s] of wheel in wheel frame
    5 PT.WRR.Trq_Drive	[N] Supported driving torque at wheel carrier
    6 PT.WRL.Trq_Drive	[N] Supported driving torque at wheel carrier
    7 Vhcl.Pol.v_abs	[m/s] absolut velocity of PoI in Fr1
    8 Vhcl.Pol.ax_1		[m/s²] of PoI in Fr1
    9 Vhcl.Pol.ay_1		[m/s²] of PoI in Fr1
    10 Vhcl.Pol.az_1		[m/s²] of PoI in Fr1
    11 Car.RollAcc		[rad/s²] of Car in 2pi (negativ := right turn)
    12 Car.PtichAcc		[rad/s²] of Car in Fr1
    13 Car.YawAcc		[rad/s²] of Car in Fr1
    14 Vhcl.FL.rz		[rad] rotation angles of carrier at mounted position
    15 Vhcl.FR.rz		[rad] rotation angles of carrier at mounted position
    16 Vhcl.Pol.x		[m] Position for PoI in Fr0
    17 Vhcl.Pol.y		[m] Position for PoI in Fr0
    18 Vhcl.PoI.vx_1		[m/s] velocity vector for PoI in Fr1
    19 Vhcl.PoI.vy_1		[m/s] velocity vector for PoI in Fr1
     sternchen/

    //gps << PoI.x/111195, PoI.y/111195, v, yaw, yawrate
    // time = 0.001*k
    double tmpd[line.size()];
    for(int i=0; i<line.size(); i++){
        tmpd[i] = std::stod(line[i]);
    }
    double dt = tmpd[0]-lastTime_simdata;
    lastTime_simdata = tmpd[0];
    yawrate += dt*tmpd[13];
    yaw += dt*yawrate;
    //printf("readsimdata.yaw = %f, %f, %f, %f\n", dt, tmpd[13], yawrate, yaw);
    //TODO X/Y pos in lattitude/longitude, would be better in m diff to startpos

    // return zero-order-hold of input+noise
    int t = int(tmpd[0]*1000); // t := time since start in ms.
    if(!use_hold || t%249 == 0){ // new gps noise every 0.25 seconds
        r->gps << tmpd[16]/111195, tmpd[17]/111195, tmpd[7], yaw, yawrate; // use r->gps to store measurement cause it has the right dimensions and gets overwritten with gps_noise anyways
        if(use_noise){
            //gps_noise = measurement + noise
            gps_noise.setRandom();
            gps_noise /= 111195;
            gps_noise += r->gps;
        }else{
            //gps_noise = measurement
            gps_noise << r->gps; // gps_noise << r->gps vs. gps_noise = r->gps
        }

    }
    if(!use_hold || t%101 == 0){
        r->imu << tmpd[8], tmpd[9], yawrate;
        if(use_noise){
            imu_noise.setRandom();
            imu_noise /= 10;
            imu_noise += r->imu;
        }else{
            imu_noise << r->imu;
        }
    }
    if(!use_hold || t%99 == 0){
        r->rh << tmpd[1], tmpd[2];
        if(use_noise){
            rh_noise.setRandom();
            rh_noise /= 10;
            rh_noise += r->rh;
        }else{
            rh_noise << r->rh;
        }
    }
    if(!use_hold || t%53 == 0){
        r->u << 0.5*tmpd[5]+0.5*tmpd[6], yawrate;
        if(use_noise){
            u_noise.setRandom();
            u_noise /= 10;
            u_noise += r->u;
        }else{
            u_noise << r->u;
        }
    }
    r->true_state << tmpd[16], tmpd[17], yaw, tmpd[18], tmpd[19], yawrate, tmpd[8], tmpd[9], abs(tmpd[18]) < 0.1 ? 0.0 : (tmpd[1]*0.245-tmpd[18])/tmpd[18], abs(tmpd[18]) < 0.1 ? 0 : (tmpd[2]*0.245-tmpd[18])/tmpd[18];
    r->time = tmpd[0];
    r->gps = gps_noise; // gps_noise << r->gps vs. gps_noise = r->gps
    r->imu << imu_noise;
    r->rh << rh_noise;
    r->u << u_noise;
    return r;
}

void runKF(int enabled){
    //enable = usePseudo, useU, useRh, useImu, useGps, usePred, use_noise, use_hold
    bool use_hold = (1 << 0 & enabled) > 0;
    bool use_noise = (1 << 1 & enabled) > 0;
    KF* kf = new KF(use_hold, use_noise, (1 << 2 & enabled) > 0, (1 << 3 & enabled) > 0, (1 << 4 & enabled) > 0, (1 << 5 & enabled) > 0, (1 << 6 & enabled) > 0, (1 << 7 & enabled) > 0);
    printf("use_hold = %s, use_noise = %s\n", B2S(use_hold), B2S(use_noise));
    std::ifstream file;
    //file.open("../sensordata/real_data_stationary_inside.txt");
    file.open("../sensordata/sim_data_slalom.txt");
    printf("file ../sensordata/sim_data_slalom.txt is open = %s\n", B2S(file.is_open()));
    int li = 0;
    if (file.is_open()) {
        std::string fileinput;
        std::vector<std::string> tmp;
        Data *data = new Data();
        double tmpd[10];
        getline(file, fileinput);
        //printf("column names: %s\n", fileinput.c_str());
        while (getline(file, fileinput)) {
            //printf("line %i in file = %s\n", li, fileinput.c_str());
            tmp = split(fileinput, "\t");
            data = read_simdata(tmp, data, use_noise, use_hold);
            if(kf->call(data->time, data->gps, data->imu, data->rh, data->u, data->true_state)){
                break;
            }
            li++;
        }
        file.close();
        lastTime_simdata = 0;
        yawrate = 0;
        yaw = 0;
        delete data;
    }
    kf->print_state();
}

int run_debug_kf() {
    //enable = usePseudo, useU, useRh, useImu, useGps, usePred, use_noise, use_hold
    //u and pseudo together make ax go wild.
    int i = 0b10111111;
    runKF(i);
    i = 0b01111111;
    runKF(i);
    return 0;
    for(int i=(1<<6)-1; i>=0; i--) {
        printf("%i: %s, %s, %s, %s, %s, %s\n", i, B2S((1 << 0 & i) > 0), B2S((1 << 1 & i) > 0), B2S((1 << 2 & i) > 0), B2S((1 << 3 & i) > 0), B2S((1 << 4 & i) > 0), B2S((1 << 5 & i) > 0));
        runKF(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // use system time with seconds accuracy for filenames -> when running the KF is faster than 1 second, its log file gets the same name as before.
    }
    return 0;
}
*/
