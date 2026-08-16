#pragma once

#ifndef MOTORMANAGER_H
#define MOTORMANAGER_H

#include "../Config/ActuatorsConfig.h"
#include <Arduino.h>

#include "../ActuatorPWM/ActuatorPWM.h"


#include "../Utilities/Utilities.h"
#include "../Config/ActuatorsConfig.h"
#include "../Config/ControllerConfig.h"

#include <Handler.h>

class FlightManager; // forward declaration of FlightManager class




class MotorManager { 
    public:
        void begin(FlightManager* fm); // attaches all the Servos and ESC (hardware)

        void update(const ControlCommand& command); // actuates the motors and vanes based on the control command
        

        
        
        String EDF_arming_status() const{
            String status = "MOTORS: ";
            status += (ESC_enabled) ? "ARMED" : "DISABLED";
            status += ", VANES: ";
            status += (vanes_enabled) ? "ENABLED" : "DISABLED";
            return(status);
        }

        float getESC_thrust_percentage() const {return ESC_thrust_percentage;};
        
        private:


        void actuate_EDF();
        void actuate_vanes();


            void toggleVanes(bool enable){vanes_enabled = enable;};
        void toggleESC(bool enable){ESC_enabled = enable;};

            Handler EDF_handler; // handler for the EDF motor
            Handler vanes_handler; // handler for the vane X1

            float ESC_thrust_percentage = 0.0f;
            ActuatorPWM ESC;
            ActuatorPWM vane_X1;
            ActuatorPWM vane_X2;
            ActuatorPWM vane_Y1;
            ActuatorPWM vane_Y2;

            bool vanes_enabled = false;

            bool ESC_enabled = false;

            FlightManager* flight_manager = nullptr;

            ActuatorsCommand current_actuator_command; // stores the current actuator command for reference

            ActuatorsCommand controlCmdToActuatorsCmd(const ControlCommand& command);
};


#endif