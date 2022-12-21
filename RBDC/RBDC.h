/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_RBDC_H_
#define CATIE_SIXTRON_RBDC_H_

#include <stdint.h>
#include "RBDC/odometry.h"
#include "RBDC/motor_control.h"

namespace sixtron {

    class RobotBaseDriveControl {

    public:
        RobotBaseDriveControl(Odometry* odometry, MotorControl* motor_control);
        Odometry* odometry;
        MotorControl* motor_control;

        void compute();

    };

} // namespace sixtron

#endif // CATIE_SIXTRON_RBDC_H_
