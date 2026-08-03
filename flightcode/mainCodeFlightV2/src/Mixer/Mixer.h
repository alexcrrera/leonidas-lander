#pragma once

#ifndef MIXER_H
#define MIXER_H


#include "Utilities.h"
#include "ActuatorsConfig.h"



namespace Mixer
{

    ActuatorCommands controlCmdToActuatorsCmd(const ControlCommand& command){

        // converts torques into servo and ESC commands
        // clamping is handled internally

        
        float yaw_cmd =    clamping(command.tau_yaw,ActuatorsConfig::maxAngleZTVC );
        float pitch_cmd =  clamping(command.pitch_roll,ActuatorsConfig::maxAngleXYTVC );
        float roll_cmd =   clamping(command.tau_roll,ActuatorsConfig::maxAngleXYTVC );
        float thrust_cmd = constrain(command.thrust, ActuatorsConfig::ESC_thrust_min_percentage, ActuatorsConfig::ESC_thrust_max_percentage);

        ActuatorCommand cmd_output = {
            .vaneX1_deg = yaw_cmd + pitch_cmd,
            .vaneX2_deg = - yaw_cmd + pitch_cmd,
            .vaneY1_deg = yaw_cmd + roll_cmd,
            .vaneY2_deg = yaw_cmd + roll_cmd,
            .thrust_percentage = thrust_cmd
        }
        

        return(cmd_output);


    }

}


#endif