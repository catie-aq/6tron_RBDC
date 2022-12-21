//
// Created by sedelpeuch on 20/12/22.
//

#ifndef RBDC_MOTOR_CONTROL_EPOCK_H
#define RBDC_MOTOR_CONTROL_EPOCK_H

#include "RBDC/motor_control.h"

namespace sixtron {
class MotorControlEpock: public MotorControl {
public:
    void compute_vel(struct Velocity vel);
};
}

#endif // RBDC_MOTOR_CONTROL_EPOCK_H