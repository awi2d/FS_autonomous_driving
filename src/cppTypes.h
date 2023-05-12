#ifndef STATE_ESTIMATOR_CPPTYPES_H
#define STATE_ESTIMATOR_CPPTYPES_H

#include "Eigen/Dense"
#include "vector"
enum class Color {blue, yellow, orange_ys, orange_bs};  // for convinience, the two orange cones are considered different colors depending on if they are on the yellow or blue side
typedef double heading;
typedef double meter;
typedef double second;
typedef double radiants;
typedef double velocity;
typedef double yawrate;
typedef unsigned int droneFrnr;
typedef std::tuple<double, double> pxpos;
typedef std::tuple<Color, double, double, double, double> boundingbox;  // (cls, posw, posh, sizew, sizeh)
typedef std::tuple<meter, heading> distheading;
typedef std::vector<pxpos> cone_keypoints; // always has length 7.
typedef std::tuple<meter, meter, heading> pose;
typedef std::tuple<meter, meter, heading, velocity, yawrate> pose_ext;
typedef Eigen::Vector2d m_position;  // (meter_north, meter_east)
typedef std::tuple<double, Color> vertex_metainfo;  // (negative-information score)
typedef unsigned int type_frnr;
#endif //STATE_ESTIMATOR_CPPTYPES_H
