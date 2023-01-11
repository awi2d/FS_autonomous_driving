# FS_autonomous
should contain Files neccesary to autonous drive around an autocross-track, as described in Formula Student Germany. \
##### contains: 
* velocity_estimator:
  - input: all sensor data from the car.
  - output: state of the car, including position and velocity (linear an angle)
* image_detection:
  - input: images from the onboard-cameras.
  - output: position and color of all cones in that image, relative to the car.
* SLAM:
  - input: state of the car and position of cones
  - output: map of the track and position of car in the map.
* [optionaly] controller:
  - input: state of the car, map and position of car in map.
  - output: gaol veloctiy and yawrate.

## velocity_estimator: 

 velocity_estimator.main:
 class KF:
  construcotr: no arguments.
   (currently it has arguments to disable the use of specific mesurments/prediction, but thats for debug purposes and will be removed in final version)

  call should be used when all last mesurments can be accesed, but it is unknown which mesurments updated since last tick.
  if it is known witch mesurment updated, mesurment_Mess should be called directly (with Mess element of {gps, imu, rh, u}.

  bool call(double currentTime, const Vector5d& gps, const Vector3d& imu, const Eigen::Vector2d& rh, const Eigen::Vector2d& u, const Vector10d& true_x)
   params:
    currentTime: the time the new mesurment was taken, in sekonds. Should be greater or equal than the greatest currentTime given to any KF funktion before.
    calls doSystemPrediction and mesurment_Mess(currentTime, Mess), if Mess has changed from last call, for Mess in {gps, imu, rh, u}

  private void doSystemPrediction(double dt)
    should not be called directly.
    dt: time since last call to doSystemPrediction in seconds. (should be greater or equal to 0)
    sets the state to the best estimate, given the state from dt seconds ago.

  void mesurment_gps(const Vector5d& gps_mes)
    gps_mes = [lattitude, longitude, speed_over_ground, heading, yawrate]
     assuming heading = 0 := north, heading = 90 := east

  void mesurment_imu(const Vector3d& imu_mes){
    imu_mes = [ax, ay, yawrate]

  void mesurment_rh(const Eigen::Vector2d& rh_mes){
    rh_mes = [turning_speed_rear_left_wheel, turning_speed_rear_right_wheel]
     so that turning_speed_rear_X_wheel * KF.radius = vx of car (assuming zero slip)

   void mesurment_u(const Eigen::Vector2d& u_mes){
     u_mes = [Trq_Drive, wheelangel]
      assuming Trq_Drive is the Torque of the motors and wheelangle is the angle of the steering wheel. TODO add constant, so that wheelangle*constant = yawrate