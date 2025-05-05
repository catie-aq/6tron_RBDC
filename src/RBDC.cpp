/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "RBDC/RBDC.h"
#include "common.h" // todo: to be removed (temp trapeze debug)

namespace sixtron {

RBDC::RBDC(Odometry *odometry, MobileBase *mobile_base, RBDC_params rbdc_parameters):
        _odometry(odometry), _mobile_base(mobile_base), _parameters(rbdc_parameters)
{

    // todo: this should be removed
    if (_parameters.dv_reducing_coefficient < 0.0f) {
        _parameters.dv_reducing_coefficient = 0.0f;
    } else if (_parameters.dv_reducing_coefficient > 1.0f) {
        _parameters.dv_reducing_coefficient = 1.0f;
    }

    // todo: this should be removed
    // if (_parameters.dv_precision > _parameters.target_precision) {
    //     _parameters.dv_precision = _parameters.target_precision;
    // }

    // If needed, set default movement behavior for linear and angular speed calculations
    // todo: should both linear and angular default movement be "pid_only"?
    if (_parameters.linear_parameters.movement == speed_movement_type::undefined) {
        _parameters.linear_parameters.movement = speed_movement_type::trapezoidal_only;
    }
    if (_parameters.angular_parameters.movement == speed_movement_type::undefined) {
        _parameters.angular_parameters.movement = speed_movement_type::pid_only;
    }

    // Update control loop instances
    _linear_control.parameters = _parameters.linear_parameters;
    _angular_control.parameters = _parameters.angular_parameters;

    _linear_control.speeds = _linear_control.parameters.default_speeds;
    _angular_control.speeds = _angular_control.parameters.default_speeds;

    // Setup PIDs
    _linear_control.pid
            = new PID(_linear_control.parameters.pid_params, rbdc_parameters.dt_seconds);
    _angular_control.pid
            = new PID(_angular_control.parameters.pid_params, rbdc_parameters.dt_seconds);

    _linear_control.pid->setLimit(sixtron::PID_limit::output_limit_HL,
            _linear_control.parameters.default_speeds.max_speed);
    _angular_control.pid->setLimit(sixtron::PID_limit::output_limit_HL,
            _angular_control.parameters.default_speeds.max_speed);

    // initialization
    _odometry->init();
    _mobile_base->init();
}

// this function give the shortest angle between -180° and +180°
static inline float getDeltaFromTargetTHETA(float target_angle_deg, float current_angle)
{

    float delta = fmodf((target_angle_deg - current_angle), float(2 * M_PI));

    if (delta > float(M_PI)) {
        delta -= float(2 * M_PI);
    } else if (delta < -float(M_PI)) {
        delta += float(2 * M_PI);
    }

    return delta;
}

// This function compute the necessary data to do a correct trapezoid movement
// This algorithm is inspired a lot by Aversive library, written by Microb Technology (Eirbot 2005)
static float apply_trapeze_profile(speed_control_instance *control_instance,
        const float dt_seconds,
        const float remaining_distance,
        float current_speed)
{
    float pivot = 0.0f, speed_increment = 0.0f, output_speed = 0.0f;

    // Cap speed input for stability purpose (and validity of calculations)
    if (current_speed > control_instance->speeds.max_speed) {
        current_speed = control_instance->speeds.max_speed;
    }

    // Pivot Anticipation (useful to counteract the delay induced by the velocity control)
    pivot = (current_speed * control_instance->parameters.trapeze_tuning.pivot_gain);

    // Pivot Compute (the distance when we need to decelerate, depending on the current speed)
    pivot += current_speed * current_speed / (2.0f * control_instance->speeds.max_decel);

    // Check is pivot has been reached or not, define the speed increment accordingly
    if (pivot > remaining_distance) {
        speed_increment = -(control_instance->speeds.max_decel * dt_seconds);
    } else {
        speed_increment = +(control_instance->speeds.max_accel * dt_seconds);
    }

    // Increment the output speed
    output_speed = control_instance->previous_output_speed + speed_increment;

    // Cap output speed
    if (output_speed > control_instance->speeds.max_speed) {
        output_speed = control_instance->speeds.max_speed;
    } else if (output_speed < 0.0f) {
        output_speed = 0.0f;
    }

    // cap to 0 m/s if already in precision
    if (remaining_distance < (control_instance->parameters.precision
                * control_instance->parameters.trapeze_tuning.precision_gain)) {
        output_speed = 0.0f;
    }

    // Save new speed consign for next time
    control_instance->previous_output_speed = output_speed;
    return output_speed;
}

// General function to compute the right speed command depending on movement type.
// This is the equivalent of a modular servo controlled loop.
static float get_speed_command(speed_control_instance *control_instance,
        const float dt_seconds,
        float const actual_speed,
        float const distance_error)
{
    float speed_command = 0.0f;

    // Apply trapeze if set
    if (control_instance->parameters.movement == speed_movement_type::trapezoidal_only
            || control_instance->parameters.movement == speed_movement_type::trapezoidal_and_pid) {
        float negate_speed = 1.0f;
        float negate_distance_error = 1.0f;

        // special case: if input speed is negative. Should only appear for the angle.
        if (actual_speed < 0.0f) {
            negate_speed = -1.0f;
        }

        // same for the distance_error, again, should only appear for the angle.
        if (distance_error < 0.0f) {
            negate_distance_error = -1.0f;
        }

        // Compute the right speed consign compared to the current linear distance error
        speed_command = negate_distance_error
                * apply_trapeze_profile(control_instance,
                        dt_seconds,
                        distance_error * negate_distance_error,
                        actual_speed * negate_speed);
    }

    // CAREFUL: in "pid_only" mode, the actual_speed is not used! Only the distance error!
    // It's a speed output control loop, based on distance error input only.
    // PID ramps cannot be used to define properly the max output acceleration/deceleration !
    // PID ramps are only used to "smooth" the input target.
    if (control_instance->parameters.movement == speed_movement_type::pid_only) {

        control_instance->pid_args.actual = 0.0f;
        control_instance->pid_args.target = distance_error;

        control_instance->pid->compute(&control_instance->pid_args);
        speed_command = control_instance->pid_args.output;
    } else if (control_instance->parameters.movement == speed_movement_type::trapezoidal_and_pid) {

        control_instance->pid_args.actual = actual_speed;
        control_instance->pid_args.target = speed_command; // previously calculated by the trapeze

        control_instance->pid->compute(&control_instance->pid_args);
        speed_command = control_instance->pid_args.output;
    }

    return speed_command;
}

void RBDC::setTarget(float x, float y, RBDC_reference reference)
{
    target_position target;
    target.pos.x = x;
    target.pos.y = y;
    target.correct_final_theta = false;
    target.ref = reference;

    setTarget(target);
}

void RBDC::setTarget(float x, float y, float theta, RBDC_reference reference)
{
    target_position target;
    target.pos.x = x;
    target.pos.y = y;
    target.pos.theta = theta;
    target.ref = reference;

    setTarget(target);
}

void RBDC::setTarget(position target_pos, RBDC_reference reference)
{
    target_position target;
    target.pos = target_pos;
    target.ref = reference;

    setTarget(target);
}

void RBDC::setVector(
        float v_linear_x, float v_linear_y, float v_angular_z, RBDC_reference reference)
{
    target_speeds target;
    target.cmd_lin = v_linear_x;
    target.cmd_tan = v_linear_y;
    target.cmd_rot = v_angular_z;

    setVector(target, reference);
}

void RBDC::setVector(target_speeds rbdc_target_speeds, RBDC_reference reference)
{
    target_position target;
    target.correct_final_theta = false;
    target.is_a_vector = true;
    target.ref = reference; // default should bed relative, for mobile base

    _target_vector.cmd_lin = rbdc_target_speeds.cmd_lin;
    _target_vector.cmd_tan = rbdc_target_speeds.cmd_tan;
    _target_vector.cmd_rot = rbdc_target_speeds.cmd_rot;

    setTarget(target);
}

void RBDC::setTarget(target_position rbdc_target_pos)
{

    // If it is a vector, do nothing else
    if (rbdc_target_pos.is_a_vector) {
        _target_pos = rbdc_target_pos;

        return;
    }

    // Do a fmodf one time, to be sure that input angle is between -360° and +360°
    rbdc_target_pos.pos.theta = fmodf(rbdc_target_pos.pos.theta, float(2 * M_PI));

    if (rbdc_target_pos.ref == RBDC_reference::relative) {

        // Transform relative target to global target from current position
        position target_transform;
        target_transform.x = +float(rbdc_target_pos.pos.x) * cosf(_odometry->getTheta())
                - float(rbdc_target_pos.pos.y) * sinf(_odometry->getTheta()) + _odometry->getX();
        target_transform.y = +float(rbdc_target_pos.pos.x) * sinf(_odometry->getTheta())
                + float(rbdc_target_pos.pos.y) * cosf(_odometry->getTheta()) + _odometry->getY();
        target_transform.theta = +float(rbdc_target_pos.pos.theta) + _odometry->getTheta();
        rbdc_target_pos.pos = target_transform;
    }

    // update target pos
    _target_pos = rbdc_target_pos;
}

RBDC_status RBDC::update()
{
    static uint32_t timestamp = 1627551892437;
    // ====== Get actual odometry ===========
    _odometry->update();

    // ======= Standby check ================

    if (_standby) {

        _linear_control.pid_args.output = 0.0f;
        _angular_control.pid_args.output = 0.0f;

        // reset PIDs
        _linear_control.pid->reset();
        _angular_control.pid->reset();

        _rbdc_cmds.cmd_lin = 0.0f;
        _rbdc_cmds.cmd_tan = 0.0f;
        _rbdc_cmds.cmd_rot = 0.0f;

        updateMobileBase();

        return RBDC_status::RBDC_standby;
    }

    // ======= Vector check ================

    if (_target_pos.is_a_vector) {
        if (_target_pos.ref == RBDC_reference::absolute) {
            // convert the vector to a global ref, instead of robot local base, not sure if this is
            // useful
            _rbdc_cmds.cmd_lin = (_target_vector.cmd_lin * cosf(-_odometry->getTheta()))
                    - (_target_vector.cmd_tan * sinf(-_odometry->getTheta()));
            _rbdc_cmds.cmd_tan = (_target_vector.cmd_lin * sinf(-_odometry->getTheta()))
                    + (_target_vector.cmd_tan * cosf(-_odometry->getTheta()));
            _rbdc_cmds.cmd_rot = _target_vector.cmd_rot;
        } else {
            // in local base, relative reference, just send the vector to the mobile base
            _rbdc_cmds = _target_vector;
        }
        //! CAREFUL: by doing that, all accelerating ramp are shunted. No PID is used.
        updateMobileBase();
        return RBDC_status::RBDC_following_vector; // In this mode, RBDC will always (and only)
                                                   // following a vector.
    }

    // =========== Run RBDC =================
    RBDC_status rbdc_end_status = RBDC_status::RBDC_working;

    // ==== Compute remaining distance ======

    // defines the remaining distance to target in the global referential
    float e_x_global = _target_pos.pos.x - _odometry->getX();
    float e_y_global = _target_pos.pos.y - _odometry->getY();
    float e_theta_global = _target_pos.pos.theta - _odometry->getTheta();

    if (_target_pos.ref == RBDC_reference::absolute) {
        e_theta_global = getDeltaFromTargetTHETA(_target_pos.pos.theta, _odometry->getTheta());
    }

    // for ARM, option "-ffast-math" for floating-point optimizations
    float error_linear
            = sqrtf((e_x_global * e_x_global) + (e_y_global * e_y_global)); // linear error

    // Compute linear and angular current speeds
    float diff_x = _odometry->getX() - _old_pos.x;
    float diff_y = _odometry->getY() - _old_pos.y;
    float diff_theta = _odometry->getTheta() - _old_pos.theta;
    // todo: Linear speed should be a class member, could be used by "cancel" function
    float linear_speed = (sqrtf((diff_x * diff_x) + (diff_y * diff_y)) / _parameters.dt_seconds);
    float angular_speed = (diff_theta) / _parameters.dt_seconds; // careful, this can be negative
    _old_pos.x = _odometry->getX();
    _old_pos.y = _odometry->getY();
    _old_pos.theta = _odometry->getTheta();

    float linear_speed_command = 0.0f, angular_speed_command = 0.0f;

    // TODO: Two wheels robot broken for now (trapeze dev)
    if (_parameters.rbdc_format == two_wheels_robot) {

        // 1 Q : Is robot inside the target zone ?
        if ((error_linear < _linear_control.parameters.precision)
                && (error_linear > -_linear_control.parameters.precision)) {
            // 1.1 A : Yes it is.

            // Check if robot is inside dv zone. Target zone must be greater than dv zone.
            if (!_dv_zone_reached
                    && ((error_linear < _linear_control.parameters.precision)
                            && (error_linear > -_linear_control.parameters.precision))) {
                _dv_zone_reached = true;
                _arrived_theta = _odometry->getTheta(); // save arrive theta the first time we
                                                        // arrived inside dv zone
            }

            // Correct angle ONLY if inside target zone AND dv zone already reached
            if (_dv_zone_reached) {
                // Be sure that dv is shutdown
                _linear_control.pid->reset();
                _linear_control.pid_args.output = 0.0f;
                _first_move = true; // reset first move for next target update

                float delta_angle;
                if (_target_pos.correct_final_theta) {
                    // Compute the final angle
                    delta_angle = e_theta_global;
                } else {
                    // keep the arrived angle has the default one.
                    delta_angle = getDeltaFromTargetTHETA(_arrived_theta, _odometry->getTheta());
                }

                // update pid theta
                _angular_control.pid_args.actual = 0.0f;
                _angular_control.pid_args.target = delta_angle;
                _angular_control.pid->compute(&_angular_control.pid_args);

                // 1.1 Q : Is target angle (or final angle) correct ?
                if (fabs(delta_angle) < _angular_control.parameters.precision) {
                    // 1.1.1 A : Yes it is. The robot base is in target position.
                    rbdc_end_status = RBDC_status::RBDC_done;
                } else {
                    // 1.1.2 A : No it is not.
                    rbdc_end_status = RBDC_status::RBDC_correct_final_angle;
                }
            }

        } else {
            _dv_zone_reached = false;
        }

        if (!_dv_zone_reached) {
            // 1.2 A : No it isn't. The base has to move to the target position.

            // Compute the angle error based on target X/Y
            float target_angle = (atan2f((_target_pos.pos.y - _odometry->getY()),
                    (_target_pos.pos.x - _odometry->getX())));
            float delta_angle = getDeltaFromTargetTHETA(target_angle, _odometry->getTheta());

            _running_direction = RBDC_DIR_FORWARD;
            // Check if it is better to go backward or not. Update delta angle accordingly.
            if (_parameters.can_go_backward) {
                if (delta_angle > float(M_PI_2)) {
                    delta_angle -= float(M_PI);
                    _running_direction = RBDC_DIR_BACKWARD;
                } else if (delta_angle < -float(M_PI_2)) {
                    delta_angle += float(M_PI);
                    _running_direction = RBDC_DIR_BACKWARD;
                }
            }

            // update pid theta
            _angular_control.pid_args.actual = 0.0f;
            _angular_control.pid_args.target = delta_angle;
            _angular_control.pid->compute(&_angular_control.pid_args);

            if (_first_move) {
                // 1.2.2.2 A
                // if it is the first move, use a more accurate position instead of
                // moving_theta_precision
                if ((fabs(delta_angle) < _angular_control.parameters.precision)
                        || _target_pos.is_a_vector) {
                    _first_move = false;
                }
                rbdc_end_status = RBDC_status::RBDC_correct_initial_angle;

                // 1.2 Q : Is the base align with the target position (angle thinking) ?
                // Or shunt this question if target is a vector.
            } else if ((fabs(delta_angle) < _parameters.moving_theta_precision)
                    || _target_pos.is_a_vector) {
                // 1.2.1 A : Yes it is.

                error_linear = _running_direction * error_linear; // Add direction of moving

                // update pid dv
                _linear_control.pid_args.actual = 0.0f;
                _linear_control.pid_args.target = error_linear;
                _linear_control.pid->compute(&_angular_control.pid_args);

                rbdc_end_status = RBDC_status::RBDC_moving;

            } else {
                // 1.2.2 A : No it isn't. Angle must be corrected.
                // 1.2.2.1 A. The Base is probably already moving to the target
                // position. But, the PID Theta can't keep up (maybe output PWM are already at max)
                // So we need to reduce the speed, in order for the angle to be corrected correctly.

                _linear_control.pid_args.output = _linear_control.pid_args.output
                        * _parameters.dv_reducing_coefficient; // reduce by given coefficient,
                                                               // only one time

                rbdc_end_status = RBDC_status::RBDC_moving_and_correct_angle;
            }
        }

        _rbdc_cmds.cmd_lin = _linear_control.pid_args.output;
        _rbdc_cmds.cmd_rot = _angular_control.pid_args.output;
    }

    else if (_parameters.rbdc_format == three_wheels_robot) {

        static float polar_angle;

        // condition to consider target reached
        if ((error_linear < _linear_control.parameters.precision)
                && (e_theta_global < _angular_control.parameters.precision)) {
            rbdc_end_status = RBDC_status::RBDC_done;
        } else {
            rbdc_end_status = RBDC_status::RBDC_moving;
        }

        linear_speed_command = get_speed_command(
                &_linear_control, _parameters.dt_seconds, linear_speed, error_linear);

        angular_speed_command = get_speed_command(
                &_angular_control, _parameters.dt_seconds, angular_speed, e_theta_global);

        // todo: optimized
        polar_angle = atan2f(e_y_global, e_x_global);

        _rbdc_cmds.cmd_lin = linear_speed_command * cosf(polar_angle - _odometry->getTheta());
        _rbdc_cmds.cmd_tan = linear_speed_command * sinf(polar_angle - _odometry->getTheta());
        _rbdc_cmds.cmd_rot = angular_speed_command;
    }

    // terminal_printf(">angular_speed:%d:%f§ms\n>angular_command:%d:%f§ms\n>current_angle:%d:%"
    //                 "f§ms\n>e_theta_global:%d:%f§ms\n",
    //         timestamp,
    //         angular_speed,
    //         timestamp,
    //         angular_speed_command,
    //         timestamp,
    //         _odometry->getTheta() / 0.017453f,
    //         timestamp,
    //         e_theta_global / 0.017453f);

    terminal_printf(">linear_speed:%d:%f§ms\n>linear_command:%d:%f§ms\n",
            timestamp,
            linear_speed,
            timestamp,
            linear_speed_command);

    // ======== Update Motor Base ============
    updateMobileBase();
    timestamp++;
    return rbdc_end_status;
}

// todo: this function should take into account the deceleration!
void RBDC::cancel()
{
    _target_pos.pos.x = _odometry->getX();
    _target_pos.pos.y = _odometry->getY();
    _target_pos.pos.theta = _odometry->getTheta();
    _target_pos.correct_final_theta = true;
    _target_pos.is_a_vector = false;
}

void RBDC::pause()
{
    _standby = true;
}

void RBDC::stop()
{
    if (!_standby) {
        cancel();
        _standby = true;
    }
}

void RBDC::start()
{
    _standby = false;
}

int RBDC::getRunningDirection()
{
    return _running_direction;
}

void RBDC::updateMobileBase()
{
    _mobile_base->setTargetSpeeds(_rbdc_cmds);
    _standby == true ? (_mobile_base->stop()) : (_mobile_base->start());
    _mobile_base->update();
}

void RBDC::setAbsolutePosition(float x, float y, float theta)
{
    position absolute_target;
    absolute_target.x = x;
    absolute_target.y = y;
    absolute_target.theta = theta;
    setAbsolutePosition(absolute_target);
}

void RBDC::setAbsolutePosition(position absolute_pos)
{
    stop();
    _odometry->setPos(absolute_pos);
    cancel();
    start();
}

target_position RBDC::getTarget()
{
    return _target_pos;
}

} // namespace sixtron
