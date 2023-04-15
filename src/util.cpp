#include <vector>
#include <sstream>
#include <tuple>
#include <map>

#include "util.h"

double str2double(const std::string& s){
    double tmp = ::atof(s.c_str());
    return tmp;
}
int str2int(const std::string& s){
    int tmp = std::stoi(s);
    return tmp;
}
std::vector<std::string> split(const std::string& str, char delemiter){
    std::vector<std::string> res;
    std::stringstream stst(str);
    std::string tmp;
    while(std::getline(stst, tmp, delemiter)){
        res.push_back(tmp);
    }
    if (!stst && tmp.empty())
    {
        // If there was a trailing ${delemiter} then add an empty element.
        res.emplace_back("");
    }
    return res;
}

pxpos str2pxpos(const std::string& s){
    std::vector<std::string> s_split = split(s, '#');
    pxpos res{str2double(s_split[0]), str2double(s_split[1])};
    return res;
}

//copied from https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exists-using-standard-c-c11-14-17-c
bool file_exists(const std::string& name) {
    struct stat buffer{};
    return (stat (name.c_str(), &buffer) == 0);
}

// read file (copied from https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c):
std::vector<boundingbox> get_boundingbox(const std::string& cam, unsigned int framenr){
    // should have the same behavior as
    // def get_boundingboxes(cam: str, framenr: int) -> [cone_bounding_box]:
    std::string filename = "C:/Users/Idefix/PycharmProjects/tmpProject/vp_labels/"+cam+"_bb/"+cam+"_frame_"+std::to_string(framenr)+".txt";
    std::vector<boundingbox> result;
    if(file_exists(filename)){
        std::ifstream file(filename);

        std::string line;
        std::string cell;
        while(std::getline(file,line)){
            // line = "0 0.5234 0.54241 0.12345 0.12345"
            std::vector<std::string> line_split = split(line, ' ');
            boundingbox res {str2int(line_split[0]), str2double(line_split[1]), str2double(line_split[2]), str2double(line_split[3]), str2double(line_split[4])};
            result.push_back(res);
        }
    }
    return result;  // empty if file doesnt exist
}
std::map<std::tuple<int, int>, std::vector<pxpos>> keypoints_cache;
std::vector<pxpos> get_cone_keypoints(const std::string& cam, int framenr, int cone){
    //printf("get_cone_keypoints(cam=%s, frnr=%i, cone=%i)", cam.c_str(), framenr, cone);
    // should have the same behavior as
    // def get_cone_keypoints(cam, framenr, cone) -> [(normalised_px_w, normalised_px_h)]:
    if(keypoints_cache.empty()){
        std::string filename = "C:/Users/Idefix/PycharmProjects/tmpProject/vp_labels/cone_annotations.csv";  // hardcoeded filenames in the sourcecode are a good idea, right?
        if(file_exists(filename)){
            std::ifstream file(filename);
            std::string line;
            std::string cell;
            std::getline(file,line);//remove first line to avoid name row ("path,top,left_top,left_middle,left_bot,right_top,right_middle,right_bot")
            while(std::getline(file,line)){
                // line = "/cones/camL3_frame_1297.jpg_cone_17.jpg,0.7255555555555555#0.16666666666666666,0.6055555555555555#0.3466666666666667,0.51#0.5488888888888889,0.39111111111111113#0.7433333333333333,0.7822222222222223#0.36777777777777776,0.7911111111111111#0.6077777777777778,0.7811111111111111#0.7711111111111111"
                std::vector<std::string> line_split = split(line, ',');
                std::vector<std::string> line0_split = split(line_split[0], '_'); // line0_split=["/cones/camL3", "frame", "1297.jpg", "cone", "17.jpg"]
                if(line0_split.size() == 5){
                    int key_frnr = str2int(split(line0_split[2], '.')[0]);
                    int key_cone = str2int(split(line0_split[4], '.')[0]);
                    std::vector<pxpos> res {str2pxpos(line_split[1]), str2pxpos(line_split[2]), str2pxpos(line_split[3]), str2pxpos(line_split[4]), str2pxpos(line_split[5]), str2pxpos(line_split[6]), str2pxpos(line_split[7])};
                    std::tuple<int, int> key = {key_frnr, key_cone};
                    //printf("util.get_cone_keypoints: add key=(%i, %i) to keypoints_cache.\n", std::get<0>(key), std::get<1>(key));
                    keypoints_cache[key] = res;
                }
            }
        }else{
            printf("ERROR: file doesnt exists, change path in get_cone_keypoints to correct location of cone_annotations.csv\n");
        }
    }
    std::tuple<int, int> key = {framenr, cone};
    //printf("util.get_cone_keypoints: keypoints_cache[key=(%i, %i)]=%zu\n", std::get<0>(key), std::get<1>(key), keypoints_cache[key].size());
    return keypoints_cache[key];  // empty if file doesnt exist
}

distheading customPnP(const cone_keypoints& keypoints, boundingbox bb){
    int cls; double posw, posh, bb_sizew, bb_sizeh;
    std::tie(cls, posw, posh, bb_sizew, bb_sizeh) = bb;
    int imgsize_h = 1200; int imgsize_w = 1920;
    double obj_Distm[7][7] = {
            {0.00000000000000000, 0.09931057723962547, 0.18581319480106973, 0.2893731266180254, 0.09931057723962547, 0.18581319480106973, 0.2893731266180254},
            {0.09931057723962547, 0.00000000000000000, 0.08783333333333333, 0.18816666666666668, 0.058826410187308546, 0.126375, 0.20548663807186746},
            {0.18581319480106973, 0.08783333333333333, 0.00000000000000000, 0.1, 0.126375, 0.08119296009798978, 0.156},
            {0.28937312661802540, 0.18816666666666668, 0.1, 0.00000000000000000, 0.20548663807186746, 0.156, 0.1159014116265954},
            {0.09931057723962547, 0.05882641018730854, 0.126375, 0.20548663807186746, 0.00000000000000000, 0.08783333333333333, 0.18816666666666668},
            {0.18581319480106973, 0.12637500000000000, 0.08119296009798978, 0.156, 0.08783333333333333, 0.00000000000000000, 0.1},
            {0.28937312661802540, 0.20548663807186746, 0.156, 0.1159014116265954, 0.18816666666666668, 0.1, 0.00000000000000000}
    };  // distances between keypoints on physical cone in m.
    double tmp[21];
    int tmpi=0;
    for(int i=0; i<6; i++){
        for(int j=i+1; j<7; j++){
            double pxdist = std::sqrt(std::pow((std::get<0>(keypoints[i])*imgsize_w*bb_sizew-std::get<0>(keypoints[j])*imgsize_w*bb_sizew), 2)
                                      + std::pow((std::get<1>(keypoints[i])*imgsize_h*bb_sizeh-std::get<1>(keypoints[j])*imgsize_h*bb_sizeh), 2));  // = dist between keypoints i and j in pixel space
            double mpropx = obj_Distm[i][j]/pxdist;
            tmp[tmpi] = mpropx;
            tmpi++;
        }
    }
    std::sort(std::begin(tmp), std::end(tmp));
    double dist = -1.97349351e-01 + 1.82927777e+03*tmp[10];
    double avg_widthpos = 0;
    for(int i=0; i<7; i++){
        avg_widthpos += std::get<0>(keypoints[i])/7;
    }
    avg_widthpos = posw-0.5*bb_sizew+bb_sizew*avg_widthpos;
    double heading = -0.90914045 + 1.07626391*avg_widthpos;
    distheading res {dist, heading};
    return res;
}

pose_ext get_at_time(std::vector<std::tuple<droneFrnr, pose_ext>> data, double time){
    if(time <= std::get<0>(data[0])){
        printf("warning: time %f is before time range of data (%i, %i)\n", time, std::get<0>(data[0]), std::get<0>(data[data.size()-1]));
        return std::get<1>(data[0]);
    }
    if(time >= std::get<0>(data[data.size()-1])){
        printf("warning: time %f is after time range of data (%i, %i)\n", time, std::get<0>(data[0]), std::get<0>(data[data.size()-1]));
        return std::get<1>(data[data.size()-1]);
    }
    for(unsigned int i=1; i<data.size(); i++){
        if(std::get<0>(data[i]) > time){
            if(time == std::get<0>(data[i-1])){
                return std::get<1>(data[i-1]);
            }
            double w0 = abs(time-std::get<0>(data[i-1]));
            double w1 = abs(std::get<0>(data[i])-time);
            double sum = w0+w1;
            w0 = w0/sum;
            w1 = w1/sum;
            pose_ext v0 = std::get<1>(data[i]);
            pose_ext v1 = std::get<1>(data[i-1]);
            //(w1*y[i-1]+w0*y[i])/sum
            pose_ext res = {(std::get<0>(v0)*w0+std::get<0>(v1)*w1), (std::get<1>(v0)*w0+std::get<1>(v1)*w1), (std::get<2>(v0)*w0+std::get<2>(v1)*w1), (std::get<3>(v0)*w0+std::get<3>(v1)*w1), (std::get<4>(v0)*w0+std::get<4>(v1)*w1)};
            return res;

        }
    }
    printf("ERROR: unreachable code in get_at_time(data[0].time=%i, data[-1].time=%i, time=%f)", std::get<0>(data[0]), std::get<0>(data[data.size()-1]), time);
    return std::get<1>(data[0]);
}

std::vector<std::tuple<int, pose_ext>> get_car_poses(std::string filename){

    if(!file_exists(filename)){
        printf("ERROR: file %s does not exist", filename.c_str());
        std::vector<std::tuple<int, pose_ext>> res;
        return res;
    }
    std::ifstream file(filename);
    std::string line;
    std::string cell;
    std::vector<std::tuple<int, pose_ext>> res;
    while(std::getline(file,line)){

        std::vector<std::string> line_split = split(line, ',');
        pose_ext carpose = {str2double(line_split[1]), str2double(line_split[2]), str2double(line_split[3]), str2double(line_split[4]), str2double(line_split[5])};// pos_north, pos_east, heading, speed, yawrate
        std::tuple<int, pose_ext> tmp = {str2int(line_split[0]), carpose};
        res.push_back(tmp);
    }
    return res;
}
int ssdt2camL(double ssdt){
    // camL3_frnr/20 = drone3_frnr/25-29.56
    // ssdt = drone_frnr/25 + 26.4
    // camL3_frnr/20 = ssdt-55.959
    int camL3_frnr = (int) (ssdt*20-1119.2);
    return camL3_frnr;
}


double sqr(double x){
    return x*x;
}

//https://en.wikipedia.org/wiki/Vincenty%27s_formulae
const double a = 6378137.0;  // length of semi-major axis of the ellipsoid (radius at equator);
const double f = 1/298.257223563;  // flattening of the ellipsoid;
const double b = 6356752.314245;  // (1-f)*a length of semi-minor axis of the ellipsoid (radius at the poles);
std::tuple<meter, meter> gps_to_meter(double lat, double lng, double lat_base, double lng_base){
    if(lat < -2*pi || lat > 2*pi || lng < -2*pi || lng > 2*pi){
        printf("WARNING: gps_util._gps_to_distazimuth: gps (%f, %f) should be in radiants, not degree.\n", lat, lng);
        lat *= pi/180;
        lng *= pi/180;
    }
    if(lat_base < -2*pi || lat_base > 2*pi || lng_base < -2*pi || lng_base > 2*pi){
        printf("WARNING: gps_util._gps_to_distazimuth: gps_base (%f, %f) should be in radiants, not degree.\n", lat_base, lng_base);
        lat_base *= pi/180;
        lng_base *= pi/180;
    }

    double u1 = std::atan((1-f)*std::tan(lat_base));
    double u2 = std::atan((1-f)*std::tan(lat));
    double L = lng-lng_base;
    double sin_u1 = std::sin(u1);
    double cos_u1 = std::cos(u1);
    double sin_u2 = std::sin(u2);
    double cos_u2 = std::cos(u2);
    double lamba = L;
    int i = 0;

    double sin_lambda;double cos_lambda;double cos_alpha_sqrd;double sin_sigma;double cos_sigma;double sigma;double cos_2sm;
    while(true){
        i += 1;
        sin_lambda = std::sin(lamba);
        cos_lambda = std::cos(lamba);
        sin_sigma = std::sqrt(sqr(cos_u2*sin_lambda)+sqr(cos_u1*sin_u2-sin_u1*cos_u2*cos_lambda));
        cos_sigma = sin_u1*sin_u2+cos_u1*cos_u2*cos_lambda;
        sigma = std::atan2(sin_sigma, cos_sigma);
        double sin_alpha = cos_u1*cos_u2*sin_lambda/sin_sigma;
        cos_alpha_sqrd = 1-sqr(sin_alpha);
        cos_2sm = cos_sigma - 2*sin_u1*sin_u2/cos_alpha_sqrd;
        double C = f/16*cos_alpha_sqrd*(4+f*(4-3*cos_alpha_sqrd));
        double old_lambda = lamba;
        lamba = L+(1-C)*f*sin_alpha*(sigma+C*sin_sigma*(cos_2sm+C*cos_sigma*(-1+2*sqr(cos_2sm))));
        if(abs(old_lambda-lamba) < 1e-12){
            break;
        }  // approx 0.06mm

        if(i > 10){
            printf("gps_util._gps_to_distazimuth: dist and azimuth between gps {gps} and gps_base {gps_base} could not be established after {i} iterations. reaturn best guess");
            break;
        }

    }

    double u_sqared = cos_alpha_sqrd*(a*a-b*b)/(b*b);
    double A = 1+u_sqared/16384*(4096+u_sqared*(-768+u_sqared*(320-175*u_sqared)));
    double B = u_sqared/1024*(256+u_sqared*(-128+u_sqared*(74-47*u_sqared)));
    double d_sigma = B*sin_sigma*(cos_2sm+0.25*B*(cos_sigma*(-1+2*cos_2sm*cos_2sm)-B/6*cos_2sm*(-3+4*sin_sigma*sin_sigma)*(-3+4*cos_2sm*cos_2sm)));
    double dist = b*A*(sigma-d_sigma);
    double azimuth1 = std::atan2(cos_u2*sin_lambda, cos_u1*sin_u2-sin_u1*cos_u2*cos_lambda);
    //azimuth2 = np.arctan2(cos_u1*sin_lambda, -sin_u1*cos_u2+cos_u1*sin_u2*cos_lambda)  # seems to be almost the same as azimuth1
    //print(f"dist, azimutz from {gps_base} to {gps} = {dist, azimuth1, azimuth2}")
    //convert dist, azimuth1 to meter position
    return {cos(azimuth1)*dist, sin(azimuth1)*dist};
}

radiants angle_dist(radiants a, radiants b){
    return pi - fabs(fmod(fabs(a - b), 2*pi) - pi);
}

m_position distazimuth_to_meter(meter dist, radiants heading){
    m_position res;
    res << cos(heading)*dist, sin(heading)*dist;
    return res;
}
distheading meter_pose_to_distazimuth(const m_position& pos0, meter pos1_north, meter pos1_east){
    //p1+distazimuth_to_meter(meter_pose_to_distazimuth(p0, p1))
    double mv0 = pos0(0)-pos1_north;
    double mv1 = pos0(1)-pos1_east;
    distheading res {sqrt(mv0*mv0+mv1*mv1), atan2(mv1, mv0)};
    return res;
}