/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "RBDC/RBDC.h"

namespace sixtron {

RBDC::RBDC(Odometry *odometry, MotorBase *motor_base, RBDC_params rbdc_parameters):
        _odometry(odometry),
        _motor_base(motor_base),
        _parameters(rbdc_parameters),
        _pid_dv(rbdc_parameters.pid_param_dv, rbdc_parameters.dt_seconds),
        _pid_dtheta(rbdc_parameters.pid_param_dteta, rbdc_parameters.dt_seconds)
{
    _pid_dv.setLimit(sixtron::PID_limit::output_limit_HL, _parameters.max_output_dv);
    _pid_dtheta.setLimit(sixtron::PID_limit::output_limit_HL, _parameters.max_output_dtheta);

    if (_parameters.dv_reducing_coefficient < 0.0f) {
        _parameters.dv_reducing_coefficient = 0.0f;
    } else if (_parameters.dv_reducing_coefficient > 1.0f) {
        _parameters.dv_reducing_coefficient = 1.0f;
    }

    if (_parameters.dv_precision > _parameters.target_precision) {
        _parameters.dv_precision = _parameters.target_precision;
    }

    _odometry->init();
    _motor_base->init();
}

static inline float getDeltaFromTargetTHETA(float target_angle_deg, float current_angle)
{

    float delta = fmod((target_angle_deg - current_angle), float(2 * M_PI));

    if (delta > float(M_PI)) {
        delta -= float(2 * M_PI);
    } else if (delta < -float(M_PI)) {
        delta += float(2 * M_PI);
    }

    return delta;
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

void RBDC::setVector(float x, float y)
{
    target_position target;
    _target_vector.x = x; //* _parameters.dt_seconds;
    _target_vector.y = y; //* _parameters.dt_seconds;
    target.pos.x = _target_vector.x;
    target.pos.y = _target_vector.y;
    target.correct_final_theta = false;
    target.is_a_vector = true;
    target.ref = RBDC_reference::relative;
    setTarget(target);
}

void RBDC::updateTargetFromVector()
{

    _target_pos.pos.x = _target_vector.x;
    _target_pos.pos.y = _target_vector.y;
    _target_pos.pos.theta = 0.0f;
    setTarget(_target_pos);
}

void RBDC::setTarget(target_position rbdc_target_pos)
{

    if (rbdc_target_pos.ref == RBDC_reference::relative) {

        // Transform relative target to global target from curent position
        sixtron::position target_transform;
        target_transform.x = +float(rbdc_target_pos.pos.x) * cos(_odometry->getTheta())
                - float(rbdc_target_pos.pos.y) * sin(_odometry->getTheta()) + _odometry->getX();
        target_transform.y = +float(rbdc_target_pos.pos.x) * sin(_odometry->getTheta())
                + float(rbdc_target_pos.pos.y) * cos(_odometry->getTheta()) + _odometry->getY();
        target_transform.theta = +float(rbdc_target_pos.pos.theta) + _odometry->getTheta();
        rbdc_target_pos.pos = target_transform;
    }

    // update target pos
    _target_pos = rbdc_target_pos;
}

RBDC_status RBDC::update()
{

    // ====== Get actual odometry ===========
    _odometry->update();

    // ======= Standby check ================

    if (_standby) {

        _args_pid_dv.output = 0.0f;
        _args_pid_dtheta.output = 0.0f;

        // reset PIDs
        _pid_dv.reset();
        _pid_dtheta.reset();

        updateMotorBase();

        return RBDC_status::RBDC_standby;
    }

    // ======= Vector check ================

    if (_target_pos.is_a_vector) {
        // Add the vector tomr oizo current position (in relative reference)
        updateTargetFromVector();
    }

    // =========== Run RBDC =================
    RBDC_status rbdc_end_status = RBDC_status::RBDC_working;
    float e_x = _target_pos.pos.x - _odometry->getX();
    float e_y = _target_pos.pos.y - _odometry->getY();
    float error_dv = sqrtf((e_x * e_x)
            + (e_y * e_y)); // for ARM, option "-ffast-math" for floating-point optimizations

    // 1 Q : Is robot inside the target zone ?
    if ((error_dv < _parameters.target_precision) && (error_dv > -_parameters.target_precision)) {
        // 1.1 A : Yes it is.

        // Check if robot is inside dv zone. Target zone must be greater than dv zone.
        if (!_dv_zone_reached
                && ((error_dv < _parameters.dv_precision)
                        && (error_dv > -_parameters.dv_precision))) {
            _dv_zone_reached = true;
            _arrived_theta = _odometry->getTheta(); // save arrive theta the first time we arrived
                                                    // inside dv zone
        }

        // Correct angle ONLY if inside target zone AND dv zone already reached
        if (_dv_zone_reached) {
            // Be sure that dv is shutdown
            _pid_dv.reset();
            _args_pid_dv.output = 0.0f;
            _first_move = true; // reset first move for next target update

            float delta_angle;
            if (_target_pos.correct_final_theta) {
                // Compute the final angle
                delta_angle = getDeltaFromTargetTHETA(_target_pos.pos.theta, _odometry->getTheta());
            } else {
                // keep the arrived angle has the default one.
                delta_angle = getDeltaFromTargetTHETA(_arrived_theta, _odometry->getTheta());
            }

            // update pid theta
            _args_pid_dtheta.actual = 0.0f;
            _args_pid_dtheta.target = delta_angle;
            _pid_dtheta.compute(&_args_pid_dtheta);

            // 1.1 Q : Is target angle (or final angle) correct ?
            if (fabs(delta_angle) < _parameters.final_theta_precision) {
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
        float target_angle = (atan2f(
                (_target_pos.pos.y - _odometry->getY()), (_target_pos.pos.x - _odometry->getX())));
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
        _args_pid_dtheta.actual = 0.0f;
        _args_pid_dtheta.target = delta_angle;
        _pid_dtheta.compute(&_args_pid_dtheta);

        if (_first_move) {
            // 1.2.2.2 A
            // if it is the first move, use a more accurate position instead of
            // moving_theta_precision
            if ((fabs(delta_angle) < _parameters.final_theta_precision) || _target_pos.is_a_vector) {
                _first_move = false;
            }
            rbdc_end_status = RBDC_status::RBDC_correct_initial_angle;

            // 1.2 Q : Is the base align with the target position (angle thinking) ?
            // Or shunt this question if target is a vector.
        } else if ((fabs(delta_angle) < _parameters.moving_theta_precision)
                || _target_pos.is_a_vector) {
            // 1.2.1 A : Yes it is.

            error_dv = _running_direction * error_dv; // Add direction of moving

            // update pid dv
            _args_pid_dv.actual = 0.0f;
            _args_pid_dv.target = error_dv;
            _pid_dv.compute(&_args_pid_dv);

            rbdc_end_status = RBDC_status::RBDC_moving;

        } else {
            // 1.2.2 A : No it isn't. Angle must be corrected.
            // 1.2.2.1 A. The Base is probably already moving to the target
            // position. But, the PID Theta can't keep up (maybe output PWM are already at max)
            // So we need to reduce the speed, in order for the angle to be corrected correctly.

            _args_pid_dv.output = _args_pid_dv.output
                    * _parameters.dv_reducing_coefficient; // reduce by given coefficient,
                                                           // only one time

            rbdc_end_status = RBDC_status::RBDC_moving_and_correct_angle;
        }
    }

    // ======== Update Motor Base ============
    updateMotorBase();

    return rbdc_end_status;
}

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

void RBDC::updateMotorBase()
{
    target_speeds rbdc_cmds;
    rbdc_cmds.cmd_rot = _args_pid_dtheta.output;
    rbdc_cmds.cmd_lin = _args_pid_dv.output;

    _motor_base->setTargetSpeeds(rbdc_cmds);
    _motor_base->update();
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
