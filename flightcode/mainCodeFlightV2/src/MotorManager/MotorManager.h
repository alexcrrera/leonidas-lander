#pragma once

#ifndef MOTORMANAGER_H
#define MOTORMANAGER_H

#include "ActuatorsConfig.h"
#include <Arduino.h>

#include "ActuatorPWM.h"

#include "Utilities.h"


class MotorManager { 
    public:
        void begin(); // attaches all the Servos and ESC (hardware)

        void actuate(const ActuatorCommand command);

        private:
            ActuatorPWM ESC;
            ActuatorPWM vane_X1;
            ActuatorPWM vane_X2;
            ActuatorPWM vane_Y1;
            ActuatorPWM vane_Y2;
}


#endif