#ifndef STATE_ESTIMATOR_TYPES_H
#define STATE_ESTIMATOR_TYPES_H

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "custom_g2otypes.h"

namespace custom_g2otypes{
    //"register" the types to enable read/write
    G2O_REGISTER_TYPE_GROUP(custom_g2otypes);
    G2O_REGISTER_TYPE(VERTEX_CarPose, Vertex_carpose);
    G2O_REGISTER_TYPE(VERTEX_ConePos, Vertex_conepos);
    G2O_REGISTER_TYPE(EDGE_Odometry, Edge_odometry);
    G2O_REGISTER_TYPE(EDGE_GNSS, Edge_gnss_measurement);
    G2O_REGISTER_TYPE(EDGE_GNSSpos, Edge_gnss_measurement_noheading);
    G2O_REGISTER_TYPE(EDGE_VisDet, Edge_visdet);
}

#endif //STATE_ESTIMATOR_TYPES_H
