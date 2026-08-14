
#include "MotorManager.h"

void MotorManager::begin(){
    // attaches actuators to hardware pins
    ESC.attach(ActuatorsConfig::ESC);
    vane_X1.attach(ActuatorsConfig::Servo_X1);
    vane_X2.attach(ActuatorsConfig::Servo_X2);
    vane_Y1.attach(ActuatorsConfig::Servo_Y1);
    vane_Y2.attach(ActuatorsConfig::Servo_Y2);

}


void MotorManager::update(const ControlCommand& command){
    // converts ControlCommand into ActuatorCommand
    ActuatorsCommand actuator_command =controlCmdToActuatorsCmd(command);
    actuate(actuator_command);
}

void MotorManager::actuate(const ActuatorsCommand& command){
    // converts ActuatorCommand into actual
    if (vanes_enabled) {
        vane_X1.write(command.vaneX1_deg);
        vane_X2.write(command.vaneX2_deg);
        vane_Y1.write(command.vaneY1_deg);
        vane_Y2.write(command.vaneY2_deg);
    }
    if (ESC_enabled) {
        ESC.write(command.thrust_percentage);
    }
    
}


ActuatorsCommand MotorManager::controlCmdToActuatorsCmd(const ControlCommand& command){

        // converts torques into servo and ESC commands
        // clamping is handled internally

        
        float yaw_cmd =    Utilities::clamping(command.tau_yaw,ActuatorsConfig::TVC_YAW_AUTHORITY_BUDGET_deg );
        float pitch_cmd =  Utilities::clamping(command.tau_pitch,ActuatorsConfig::TVC_PITCH_ROLL_AUTHORITY_BUDGET_deg );
        float roll_cmd =   Utilities::clamping(command.tau_roll,ActuatorsConfig::TVC_PITCH_ROLL_AUTHORITY_BUDGET_deg );
        float thrust_cmd = constrain(command.thrust, ActuatorsConfig::ESC_thrust_min_percentage, ActuatorsConfig::ESC_thrust_max_percentage);

        ActuatorsCommand cmd_output = {
            .vaneX1_deg = yaw_cmd + pitch_cmd,
            .vaneX2_deg = - yaw_cmd + pitch_cmd,
            .vaneY1_deg = yaw_cmd + roll_cmd,
            .vaneY2_deg = yaw_cmd + roll_cmd,
            .thrust_percentage = thrust_cmd
        };
        

        return(cmd_output);


    }
