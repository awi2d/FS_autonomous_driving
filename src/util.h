#ifndef STATE_ESTIMATOR_UTIL_H
#define STATE_ESTIMATOR_UTIL_H

#include "string"
#include "Eigen/Dense"
#include <vector>
#include <fstream>
#include <sys/stat.h>
#include <algorithm>

const double pi = 3.14159265358979323846;

typedef double heading;
typedef double meter;
typedef double second;
typedef double radiants;
typedef double velocity;
typedef double yawrate;
typedef int droneFrnr;
typedef std::tuple<double, double> pxpos;
typedef std::tuple<int, double, double, double, double> boundingbox;  // (cls, posw, posh, sizew, sizeh)
typedef std::tuple<meter, heading> distheading;
typedef std::vector<pxpos> cone_keypoints; // always has length 7.
typedef std::tuple<meter, meter, heading> pose;
typedef std::tuple<meter, meter, heading, velocity, yawrate> pose_ext;
typedef Eigen::Vector2d m_position;  // (meter_north, meter_east)
typedef std::tuple<double, int> vertex_metainfo;  // (negative-information score, class 0:Blue, 1:yellow, 2:orange)
typedef unsigned int type_frnr;

double str2double(const std::string& s);
int str2int(const std::string& s);
std::vector<std::string> split(const std::string& str, char delemiter);
pxpos str2pxpos(const std::string& s);

bool file_exists(const std::string& name);
std::vector<boundingbox> get_boundingbox(const std::string& cam, unsigned int framenr);
std::vector<pxpos> get_cone_keypoints(const std::string& cam, int framenr, int cone);
distheading customPnP(const cone_keypoints& keypoints, boundingbox bb);
pose_ext get_at_time(std::vector<std::tuple<droneFrnr, pose_ext>> data, double time);
std::vector<std::tuple<int, pose_ext>> get_car_poses(std::string filename);
int ssdt2camL(double ssdt);
std::tuple<meter, meter> gps_to_meter(radiants lat, radiants lng, radiants lat_base, radiants lng_base);
radiants to_range(radiants a);
#endif //STATE_ESTIMATOR_UTIL_H
