#pragma once
#include "Utilities.h"
#include "Arduino.h"



struct ControlCommand
{
    float tau_yaw;
    float tau_pitch;
    float tau_roll;
    float thrust;
};

struct PID_Gains{
    float kp;
    float ki;
    float kd;
}


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





    // =========== ============= ENF OF PARAMS ========================================                                        
    constexpr PID_Gains PID_gains_position_East = PID_gains_position_North; // symmetric vehicule dynamics

}