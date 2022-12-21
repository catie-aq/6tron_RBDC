//
// Created by sedelpeuch on 20/12/22.
//

#ifndef RBDC_ODOMETRY_EPOCK_H
#define RBDC_ODOMETRY_EPOCK_H

#include "RBDC/odometry.h"
#include "ros/console.h"

namespace sixtron {

class OdometryEpock: public Odometry {
public:
    struct Position odometry(struct Position position);
    void compute_odometry();
};

}

#endif // RBDC_ODOMETRY_EPOCK_H
