/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RBDC_ODOMETRY_H
#define RBDC_ODOMETRY_H

namespace sixtron {

struct Position {
    float x;
    float y;
    float theta;
};

class Odometry {
public:
    virtual struct Position odometry(struct Position) = 0;
    virtual void compute_odometry() = 0;
};
} // namespace sixtron

#endif // RBDC_ODOMETRY_H
