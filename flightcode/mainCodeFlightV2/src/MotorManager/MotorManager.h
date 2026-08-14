#pragma once

#ifndef MOTORMANAGER_H
#define MOTORMANAGER_H

#include "../Config/ActuatorsConfig.h"
#include <Arduino.h>

#include "../ActuatorPWM/ActuatorPWM.h"


#include "../Utilities/Utilities.h"
#include "../Config/ActuatorsConfig.h"
#include "../Config/ControllerConfig.h"



class MotorManager { 
    public:
        void begin(); // attaches all the Servos and ESC (hardware)

        void update(const ControlCommand& command); // actuates the motors and vanes based on the control command
        void actuate(const ActuatorsCommand& command);

        void toggleVanes(bool enable){vanes_enabled = enable;};
        void toggleESC(bool enable){ESC_enabled = enable;};
        private:
            ActuatorPWM ESC;
            ActuatorPWM vane_X1;
            ActuatorPWM vane_X2;
            ActuatorPWM vane_Y1;
            ActuatorPWM vane_Y2;

            bool vanes_enabled = false;

            bool ESC_enabled = false;


            ActuatorsCommand controlCmdToActuatorsCmd(const ControlCommand& command);
};


#endif