#pragma once
#include "../Utilities/Utilities.h"
#include "Arduino.h"



struct ControlCommand
{
    float tau_yaw;
    float tau_pitch;
    float tau_roll;
    float thrust;
};




namespace ControllerPID {

    constexpr ParameterConfig<float> frequency_control = {
    .min = 1,
    .max = 500,
    .default_ = 50
    };

    constexpr float position_NED_update_frequency = 20.0f; // Hz
    constexpr float velocity_NED_update_frequency = 50.0f; // Hz
    constexpr float attitude_update_frequency = 100.0f; // Hz
    constexpr float body_rates_update_frequency = 200.0f; // Hz
    



    constexpr PID_Gains PID_gains_position_North = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};

  


    constexpr PID_Gains PID_gains_position_Down = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};

    constexpr PID_Gains PID_gains_position_East = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};


    constexpr PID_Gains PID_gains_velocity_North = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};

    constexpr PID_Gains PID_gains_velocity_East = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5}; 
    constexpr PID_Gains PID_gains_velocity_Down = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};
    constexpr PID_Gains PID_gains_attitude_roll = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};
                                        
    constexpr PID_Gains PID_gains_attitude_pitch = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};                                                                                
    constexpr PID_Gains PID_gains_attitude_yaw = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};
    
    constexpr PID_Gains PID_gains_body_rates_roll = {.kp = 1.0,
                                        .ki = 0.0, 
                                        .kd = 0.5};
    constexpr PID_Gains PID_gains_body_rates_pitch = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};
    constexpr PID_Gains PID_gains_body_rates_yaw = {.kp = 1.0,
                                        .ki = 0.0,
                                        .kd = 0.5};
                                                                                                                           





}