/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_RBDC_H_
#define CATIE_SIXTRON_RBDC_H_

#include <math.h>
#include <stdint.h>

#include "mobile_base/mobile_base.h"
#include "odometry/odometry.h"
#include "pid/pid.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#ifndef M_PI_2
#define M_PI_2 (1.57079632679489661923)
#endif

namespace sixtron {

// Defines for direction, useful for 2 wheels robots only ?
#define RBDC_DIR_FORWARD (1)
#define RBDC_DIR_BACKWARD (-1)

#define RBDC_MAX_STATUS 8

typedef enum {
    two_wheels_robot,
    three_wheels_robot,
    four_wheels_robot
} RBDC_format;

typedef enum {
    absolute,
    relative
} RBDC_reference;

typedef enum {
    undefined, // default behavior will be set at RBDC start
    pid_only, // generally for the angle only
    trapezoidal_only, // when the mobile base and the motors have a great response to speed command
    trapezoidal_and_pid, // when the mobile base need a linear speed control after trapeze output
} movement_type;

typedef enum {
    RBDC_standby = 0,
    RBDC_working = 1, // This should never happen, as we cover all the cases at each iteration.
    RBDC_done = 2, // 1.1.1 robot arrive at target
    RBDC_correct_final_angle = 3, // 1.1.2
    RBDC_moving = 4, // 1.2.1
    RBDC_moving_and_correct_angle = 5, // 1.2.2.1
    RBDC_correct_initial_angle = 6, // 1.2.2.2
    RBDC_following_vector = 7,
} RBDC_status;

/*!
 *  \struct trapezoid_profile
 *  RBDC trapezoidal profile
 */
typedef struct trapezoid_profile trapezoid_profile;

struct trapezoid_profile {
    float pivot_gain = 0.0f; // fine-tune the distance needed to decelerate properly
    float precision_gain = 1.0f; // fine-tune the precision when the trapeze must stop
    float previous_output_speed = 0.0f; // trapeze backup value
};

/*!
 *  \struct speed_control_parameters
 *  Velocity controle parameters for the given speed type (linear or angular)
 */
typedef struct speed_control_parameters speed_control_parameters;

struct speed_control_parameters {
    float max_accel = 0.0f; // max acceleration when ramping up in [m/s²].
    float max_decel = 0.0f; // max deceleration when ramping down in [m/s²].
    float max_speed = 0.0f; // max speed (positive or negative) in [m/s].
    trapezoid_profile trapeze;
    movement_type movement = movement_type::undefined;
    float precision = 0.0f;
    // PID *pid = nullptr; // not sure if it is a good idea here
    // PID_args pid_args; // same comment
    PID_params pid_params;
};

/*!
 *  \struct target_position
 *  RBDC target position
 */
typedef struct target_position target_position;

struct target_position {
    position pos;
    RBDC_reference ref = RBDC_reference::absolute; // global plane reference by default

    bool correct_final_theta = true; // will be set to false when no angle is provided
    bool is_a_vector = false; // when true, pos will be read as a "target_speeds" vector
};

/*!
 *  \struct RBDC_param
 *  PID parameters structure
 */
typedef struct RBDC_params RBDC_params;

struct RBDC_params {
    RBDC_format rbdc_format = two_wheels_robot;
    // PID_params pid_param_dv, pid_param_dteta;

    speed_control_parameters linear_control;
    speed_control_parameters angular_control;

    // float final_theta_precision = 0.0f;
    float moving_theta_precision = 0.0f;
    // float target_precision = 0.5f;
    // float dv_precision = 0.0f; // todo: this must disappear
    float dv_reducing_coefficient = 0.80f; // coefficient, between 0.0f and 1.0f; (todo: burn this)
    float dt_seconds = 0.0f; // todo: rename "time_step" or equivalent, remove "dt"?
    bool can_go_backward = true; // ignore when holonomic
};

class RBDC {

public:
    RBDC(Odometry *odometry, MobileBase *mobile_base, RBDC_params rbdc_parameters);

    // Target is a postion (x y theta)
    void setTarget(float x, float y, RBDC_reference reference = RBDC_reference::absolute);
    void setTarget(
            float x, float y, float theta, RBDC_reference reference = RBDC_reference::absolute);
    void setTarget(position target_pos, RBDC_reference reference = RBDC_reference::absolute);

    // Target is a vector (most of BRDC is shunted, and send speeds directly to the mobile base)
    void setVector(float v_linear_x,
            float v_linear_y,
            float v_angular_z,
            RBDC_reference reference = RBDC_reference::relative);
    void setVector(
            target_speeds rbdc_target_speeds, RBDC_reference reference = RBDC_reference::relative);

    // Main setTarget function, the one function to rule them all
    void setTarget(target_position rbdc_target_pos);

    void cancel(); // cancel current target.
    void pause(); // save current goal, wait for next start to continue
    void stop(); // cancel current target and put RBDC in standby mode. Need start to wake up.
    void start(); // get out of standby mode.

    int getRunningDirection();

    void setAbsolutePosition(float x, float y, float theta);
    void setAbsolutePosition(position absolute_pos);

    RBDC_status update();

    target_position getTarget();

private:
    target_speeds _rbdc_cmds;
    bool _standby = false;
    int _running_direction;
    void updateMobileBase();

    Odometry *_odometry;
    MobileBase *_mobile_base;

    RBDC_params _parameters;
    target_position _target_pos;
    position _old_pos;
    target_speeds _target_vector;

    PID _pid_linear, _pid_angular;
    PID_args _args_pid_linear, _args_pid_angular;

    float _arrived_theta = 0.0f;
    bool _dv_zone_reached = false;
    bool _first_move = true;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_RBDC_H_
