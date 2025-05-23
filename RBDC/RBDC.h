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

// Defines for direction, useful for two wheels differential robots only
#define RBDC_DIR_FORWARD (1)
#define RBDC_DIR_BACKWARD (-1)

#define RBDC_MAX_STATUS 8

typedef enum {
    differential_robot, // two wheels robot
    holonomic_robot // three or more wheels robot
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
} speed_movement_type;

typedef enum {
    linear,
    angular
} speed_controller_type;

typedef enum {
    RBDC_standby = 0,
    RBDC_working = 1, // This should never happen, as we cover all the cases at each iteration.
    RBDC_done = 2, // Robot arrive at target
    RBDC_correct_final_angle = 3,
    RBDC_moving = 4,
    RBDC_moving_and_correct_angle = 5,
    RBDC_correct_initial_angle = 6,
    RBDC_following_vector = 7,
} RBDC_status;

/*!
 *  \struct trapezoid_profile
 *  RBDC trapezoidal profile parameters
 */
typedef struct trapezoid_profile trapezoid_profile;

struct trapezoid_profile {
    float pivot_gain = 0.0f; // fine-tune the distance needed to decelerate properly
    float precision_gain = 1.0f; // fine-tune the precision when the trapeze must stop. Between 0-1.
};

/*!
 *  \struct speed_profile
 *  Speed profile structure to define max speed and accelerations
 */
typedef struct speed_profile speed_profile;

struct speed_profile {
    float max_accel = 0.0f; // max acceleration when ramping up in [m/s²].
    float max_decel = 0.0f; // max deceleration when ramping down in [m/s²].
    float max_speed = 0.0f; // max speed (positive or negative) in [m/s].
};

/*!
 *  \struct speed_control_parameters
 *  Velocity control parameters for the given speed type (linear or angular)
 */
typedef struct speed_control_parameters speed_control_parameters;

struct speed_control_parameters {
    speed_profile default_speeds;
    trapezoid_profile trapeze_tuning;
    speed_movement_type movement = speed_movement_type::undefined;
    float precision = 0.0f;
    PID_params pid_params;
};

/*!
 *  \struct speed_controller_instance
 *  Velocity control instance, use privately by the RBDC
 */
typedef struct speed_controller_instance speed_controller_instance;

struct speed_controller_instance {
    speed_control_parameters parameters;
    speed_profile speeds;
    float previous_output_speed = 0.0f; // trapeze output backup
    PID *pid = nullptr;
    PID_args pid_args;
};

/*!
 *  \struct target_position
 *  RBDC target position
 */
typedef struct target_position target_position;

struct target_position {
    position pos;
    RBDC_reference ref = RBDC_reference::absolute; // global plane reference by default
    // todo: let the user specify custom acceleration profile for each target?
    bool correct_final_theta = true; // will be set to false when no angle is provided
    bool is_a_vector = false; // when true, pos will be read as a "target_speeds" vector
};

/*!
 *  \struct RBDC_params
 *  PID parameters structure
 */
typedef struct RBDC_params RBDC_params;

struct RBDC_params {
    RBDC_format rbdc_format = differential_robot;

    speed_control_parameters linear_parameters;
    speed_control_parameters angular_parameters;

    float dt_seconds = 0.0f; // todo: rename "time_step" or equivalent, remove "dt"?
    bool can_go_backward = true; // ignore when holonomic
    bool decelerate_when_standby = true;
};

class RBDC {

public:
    RBDC(Odometry *odometry, MobileBase *mobile_base, const RBDC_params &rbdc_parameters);

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

    // Manually change speed profile if needed
    void setSpeedProfile(
            speed_controller_type controller_type, float max_acc, float max_decel, float max_speed);
    void setSpeedProfile(speed_controller_type controller_type, speed_profile profile);
    void resetSpeedProfile(speed_controller_type controller_type);

    void cancel(); // cancel current target. Be aware that the RBDC will first standby to decelerate
    void pause(); // save current goal, wait for next start to continue
    void stop(); // cancel current target and put RBDC in standby mode. Need start to wake up.
    void start(); // get out of standby mode.

    int getRunningDirection();

    void setAbsolutePosition(float x, float y, float theta);
    void setAbsolutePosition(position absolute_pos);

    RBDC_status update();

    target_position getTarget();

private:
    RBDC_params _parameters;
    bool _standby = false;
    bool _cancel_requested = false;
    void cancel_target();

    Odometry *_odometry;
    MobileBase *_mobile_base;
    speed_controller_instance _linear_controller, _angular_controller;

    target_position _target_pos;
    target_speeds _target_vector;
    position _old_pos;

    target_speeds _rbdc_cmds;
    float _linear_speed_command = 0.0f, _angular_speed_command = 0.0f;
    void updateMobileBase();
    void updateMobileBase(const target_speeds &mobile_base_cmds);
    void startMobileBase();
    void stopMobileBase();

    // the following member are only used for an holonomic robot
    float _polar_angle;

    // the following member are only used for a two wheels differential robot
    int _running_direction = RBDC_DIR_FORWARD;
    float _arrived_theta = 0.0f;
    bool _target_zone_reached = false;
    bool _first_move = true;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_RBDC_H_
