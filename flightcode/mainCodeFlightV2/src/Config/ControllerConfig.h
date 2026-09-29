#pragma once
#include "../Utilities/Utilities.h"
#include "Arduino.h"



struct ControlCommand
{
    float tau_yaw;
    float tau_pitch;
    float tau_roll;
    float thrust_N;
};




namespace ControllerPID {

    constexpr ParameterConfig<float> frequency_control = {
    .min = 1,
    .max = 500,
    .default_ = 50
    };
    constexpr float position_NED_update_frequency = 20.0;
    constexpr float velocity_NED_update_frequency = 25.0;
    constexpr float attitude_update_frequency = 70.0;
    constexpr float body_rates_update_frequency = 110.0;


    constexpr PID_Gains PID_gains_position_North = {.kp = 1.0,
                                        .ki = 0.1,
                                        .kd = 0.5};

    constexpr PID_Gains PID_gains_position_Down = {.kp = 2.0,
                                        .ki = 0.1,
                                        .kd = 0.05};

    constexpr PID_Gains PID_gains_position_East = {.kp = 1.25,
                                        .ki = 0.0,
                                        .kd = 0.0};


    constexpr PID_Gains PID_gains_velocity_North = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};

    constexpr PID_Gains PID_gains_velocity_East = {.kp = 2.0,
                                        .ki = 0.0,
                                        .kd = 0.0};

    constexpr PID_Gains PID_gains_velocity_Down = {.kp = 3.0,
                                        .ki = 0.4,
                                        .kd = 0.2};


    constexpr PID_Gains PID_gains_attitude_roll = {.kp = 4.0,
                                        .ki = 0.0,
                                        .kd = 0.0};

    constexpr PID_Gains PID_gains_attitude_pitch = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.1};

    constexpr PID_Gains PID_gains_attitude_yaw = {.kp = 2.0,
                                        .ki = 0.21,
                                        .kd = 0.1};


    constexpr PID_Gains PID_gains_body_rates_roll = {.kp = 5.0,
                                        .ki = 0.0,
                                        .kd = 0.0};

    constexpr PID_Gains PID_gains_body_rates_pitch = {.kp = 5.0,
                                        .ki = 0.0,
                                        .kd = 0.0};

    constexpr PID_Gains PID_gains_body_rates_yaw = {.kp = 10.0,
                                        .ki = 0.05,
                                        .kd = 0.0};




}