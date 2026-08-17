
#include "MotorManager.h"
#include "../FlightManager/FlightManager.h"
#include "../Config/ActuatorsConfig.h"

#include "../Config/SystemConfig.h"

void MotorManager::begin(FlightManager* fm){

    flight_manager = fm; // flight manager is not used in this implementation, but can be set if needed
    // attaches actuators to hardware pins
    ESC.attach(ActuatorsConfig::ESC);
    vane_X1.attach(ActuatorsConfig::Servo_X1);
    vane_X2.attach(ActuatorsConfig::Servo_X2);
    vane_Y1.attach(ActuatorsConfig::Servo_Y1);
    vane_Y2.attach(ActuatorsConfig::Servo_Y2);


    EDF_handler.begin(ActuatorsConfig::ESC.frequency, this, &MotorManager::actuate_EDF);
    vanes_handler.begin(ActuatorsConfig::Servo_frequency, this, &MotorManager::actuate_vanes);

}



void MotorManager::update(){
    // converts ControlCommand into ActuatorCommand


    if(flight_manager == nullptr){
        // flight manager is not set, cannot proceed
        Serial.println("MotorManager: FlightManager not set, cannot update actuators.");
        return;
    }

    auto command = flight_manager->getController().getControlCmd();
    auto overrideFlags = flight_manager->getFlightGuard().overrideFlags;

    toggleESC(overrideFlags.EDF_enabled);
    toggleVanes(overrideFlags.TVC_enabled);

    current_actuator_command =controlCmdToActuatorsCmd(command);

    EDF_handler.handle(); // calls the update function at the specified frequency
    vanes_handler.handle(); // calls the update function at the specified frequency


  
}


void MotorManager::actuate_EDF(){


    if (ESC_enabled) {
        float thrust_in_percentage = current_actuator_command.thrust_percentage;
        ESC_thrust_percentage =constrain(thrust_in_percentage, ActuatorsConfig::ESC_thrust_min_percentage, ActuatorsConfig::ESC_thrust_max_percentage);
        ESC.write(ESC_thrust_percentage); // writing also handles clamping and trimming internally, but we want the output to be clamped as well when printed
        
    }
    else {
        // If ESC is disabled, set thrust to 0%
        
        
         
        ESC.write(ESC_thrust_percentage );
       
    }
  
    
}


void MotorManager::actuate_vanes(){

    
        if (vanes_enabled) {
        vane_X1.write(current_actuator_command.vaneX1_deg);
        vane_X2.write(current_actuator_command.vaneX2_deg);
        vane_Y1.write(current_actuator_command.vaneY1_deg);
        vane_Y2.write(current_actuator_command.vaneY2_deg);
    }
    else {
        // If vanes are disabled, set them to neutral position (0 degrees)
        vane_X1.write(0.0f);
        vane_X2.write(0.0f);
        vane_Y1.write(0.0f);
        vane_Y2.write(0.0f);
    }

    
}



ActuatorsCommand MotorManager::controlCmdToActuatorsCmd(const ControlCommand& command){

        // converts torques into servo and ESC commands
        // clamping is handled internally

        
        float yaw_cmd =    Utilities::clamping(command.tau_yaw,ActuatorsConfig::TVC_YAW_AUTHORITY_BUDGET_deg );
        float pitch_cmd =  Utilities::clamping(command.tau_pitch,ActuatorsConfig::TVC_PITCH_ROLL_AUTHORITY_BUDGET_deg );
        float roll_cmd =   Utilities::clamping(command.tau_roll,ActuatorsConfig::TVC_PITCH_ROLL_AUTHORITY_BUDGET_deg );
        float thrust_percentage = 100.0* command.thrust_N/ActuatorsConfig::THRUST_EDF_max;
            thrust_percentage = constrain(thrust_percentage, ActuatorsConfig::ESC_thrust_min_percentage,ActuatorsConfig::ESC_thrust_max_percentage);
        
            ActuatorsCommand cmd_output = {
            .vaneX1_deg = yaw_cmd + pitch_cmd,
            .vaneX2_deg = - yaw_cmd + pitch_cmd,
            .vaneY1_deg = yaw_cmd + roll_cmd,
            .vaneY2_deg = yaw_cmd + roll_cmd,
            .thrust_percentage = thrust_percentage
        };
        

        return(cmd_output);


    }
