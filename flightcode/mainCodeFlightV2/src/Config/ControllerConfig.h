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