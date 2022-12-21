/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RBDC_MOTOR_CONTROL_H
#define RBDC_MOTOR_CONTROL_H

namespace sixtron {
struct Velocity {
    float linear;
    float angular;
};

class MotorControl {
public:
    virtual void compute_vel(struct Velocity) = 0;
};

}

#endif // RBDC_MOTOR_CONTROL_H
