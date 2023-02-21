# RBDC 6TRON Library

## *Robot Base Drive Control*

## Introduction

The RBDC library aim to implement a simple robot behaviour, to drive any motor base to a target position (X, Y, Theta).

## Implementation

The RBDC is written in C++, and not related to a particular framework, even-if it is "MbedOs ready"" using `.lib` files.

It's writing has been improved to keep as a goal the implementation on microcontroller (MCU), like STM32. But it needs somme basic requirements.

## Requirement

The library hitself has few requirement:

* `stdint.h`

* `math.h`

* strongly recommended : a [FPU](https://www.wikiwand.com/en/Floating-point_unit) (Available on most MCU nowadays)

Additionnaly, the RBDC depends of two others external class:

* [**Odometry**](https://github.com/catie-aq/6tron_odometry), to get current robot base infos (like the actual position mesured by sensors)

* [**MotorBase**](https://github.com/catie-aq/6tron_motor_base), to send speeds commands to the base.

Theses two class are defined as **template**, this means that the user has to define its own child classes for its own application. An example of a custom implementation [can be found here](https://github.com/catie-aq/mbed_RBDC-pokibot-example).

## Behaviour

<img src="docs/img/rbdc_activity_diagram.svg?sanitize=true">

## How to use

## TODO

* Holonome implementation (need a third PID ?)

* Considerate an even ligter library written in C, and not using floats. But this will be another repository.  (TIP)
