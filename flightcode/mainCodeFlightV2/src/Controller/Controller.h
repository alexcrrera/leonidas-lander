#pragma once

#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <Arduino.h>

#include "../Utilities/Utilities.h"
#include "../PID/PID.h"
#include "../Config/ControllerConfig.h"
#include "../Lander/Lander.h"
#include "../Config/FlightRegimeConfig.h"
#include "../Config/ActuatorsConfig.h"

class FlightManager; // forward declaration of FlightManager class

class Controller{

    // owns all the PID objects
    public: 

        Controller();

        void resetIntegralForAllPID(); // resets all PID's - integral term

        void update(); // updates the control command based on the current state of the lander

        void setPositionSetpoint(const NED_coordinates& position_target);
        void setVelocitySetpoint(const NED_coordinates& velocity_target);
        void setAttitudeSetpoint(const Rotation_Euler_coordinates& attitude_target);
        void setBodyRatesSetpoint(const Rotation_Euler_coordinates& body_rates_target);
        

        void setControlCmd(const ControlCommand& cmd){ctrl_cmd = cmd;} // set control command directly
      
        ControlCommand& getControlCmd(){return(ctrl_cmd);} // get latest control command
        void activate(){active = true;} // activate the controller
        void deactivate(){active = false;} // deactivate the controller
        bool isActive(){return(active);} // check if the controller is active
        
    private:

       

        void updatePIDOutputsLimits();// ensures that the control command is within the limits of the current regime's parameters

        PID_3D PID_position; // group of 3 PID's for position control
        PID_3D PID_velocity; // group of 3 PID's for velocity control
        PID_3D PID_attitude;
        PID_3D PID_body_rates;

       

        FlightRegimeData current_regime = FlightRegimeConfig::GROUND; // default is GROUNDed
        bool active = false; // indicates if the controller is active or not

        ControlCommand ctrl_cmd;

        FlightManager* flight_manager; // pointer to the FlightManager class to access the lander state




};
#endif