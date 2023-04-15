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

## SLAM

 uses g2o to optimize graph. 
 Graph is made up from custom Vertices (variables to be optimised) and Edges (constraints on these edges):
* Vertex_carpose, that contain 
  - optimisable variables: pose (position_north [m], position_east [m], heading [radiants]) and odometry (vx [m/s], vy [m/s], yawrate [radiants/second]). [std::tuple<g2o::SE2, g2o::SE2>]
  - non-optimisable variables: camL3_frnr, for debug displays
* Vertex_conepos, that contains position 
  - optimisable variables: (position_north [m], position_east [m]) [Eigen::Vector2d]
  - non-optimisable variables: color [int]
* Edge_odometry: constrain between two adjacend Vertex_carpose.
  - measurment is time [seconds].
  - constraints the pose to be consistent with the odometry.
  - constraints the odometry to be consistent with the car model: vx, vy, yawrate = constant. (there might be a better carmodel, if the acceleration data from the IMU is used)
* Edge_gnss: unary constraint on one Vertex_carpose.
  - measurement is converted gnss measurement, same type as Vertex_carpose optimisable variables. (yawrate is estimated as current_heading-old_heading/dt)
* Edge_visdet: constraint between vertex_carpose and Vertex_conepos.
  - measurement is distance to cone (in direction of heading and perpendicular to that direction), so the output of the visual pipeline.

 class SLAM:
  add_visdet:
   params:
    vis_det: output of visual pipeline
   description:
    adds new carpose to graph, with edge_odometry to last carpose.
    for each detection in vis_det: (if necessary: adds new conepos), adds edeg_visdet between current carpose and conepos.
  add_gnss:
   adds Edge_gnss to current carpose
  optimise: 
   optimises the graph, and writes the result to file.
These custom types have the advantage over the tutorial_slam2d types of g2o that they incorporate an car model, instead of relying on the speed and yawrate to be measured for each frame.