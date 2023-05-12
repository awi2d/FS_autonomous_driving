#ifndef STATE_ESTIMATOR_UTIL_H
#define STATE_ESTIMATOR_UTIL_H

#include "string"
#include "Eigen/Dense"
#include <vector>
#include <fstream>
#include <sys/stat.h>
#include <algorithm>
#include "cppTypes.h"

const double pi = 3.14159265358979323846;

double str2double(const std::string& s);
int str2int(const std::string& s);
Color int2color(int color);
int color2int(Color color);
std::vector<std::string> split(const std::string& str, char delemiter);
pxpos str2pxpos(const std::string& s);

bool file_exists(const std::string& name);
std::vector<boundingbox> get_boundingbox(const std::string& cam, unsigned int framenr);
std::vector<pxpos> get_cone_keypoints(const std::string& cam, type_frnr framenr, unsigned int cone);
distheading customPnP(const cone_keypoints& keypoints, boundingbox bb);
std::vector<std::tuple<int, pose_ext>> get_car_poses(std::string filename);
int ssdt2camL(double ssdt);
std::tuple<meter, meter> gps_to_meter(radiants lat, radiants lng, radiants lat_base, radiants lng_base);
radiants to_range(radiants a);
radiants angle_dist(radiants a, radiants b);
m_position distazimuth_to_meter(meter dist, radiants heading);
distheading meter_pose_to_distazimuth(const m_position& pos0, meter pos1_north, meter pos1_east);
#endif //STATE_ESTIMATOR_UTIL_H
