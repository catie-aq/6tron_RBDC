/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "RBDC/RBDC.h"
#include "RBDC/motor_control_epock.h"
#include "RBDC/odometry_epock.h"
#include <cstdio>
namespace sixtron {

void RobotBaseDriveControl::compute()
{
    printf("Hello, World!");
}
RobotBaseDriveControl::RobotBaseDriveControl(Odometry *odometry, MotorControl *motor_control)
{
    this->odometry = odometry;
    this->motor_control = motor_control;
}
}

int main()
{
    sixtron::OdometryEpock odometry;
    sixtron::MotorControlEpock motor_control;
    sixtron::RobotBaseDriveControl rbdc(&odometry, &motor_control);
    rbdc.compute();
}