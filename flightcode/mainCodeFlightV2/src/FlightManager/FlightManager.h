#ifndef FLIGHT_MANAGER_H
#define FLIGHT_MANAGER_H

#include <Arduino.h>
#include "../Lander/Lander.h"
#include "../Mission/Mission.h"
#include "../MotorManager/MotorManager.h"
#include "../Controller/Controller.h"



#include "../TelemetryManager/TelemetryManager.h"

#include "FlightManager_structs_and_more.h"
// ============================================================
// Flight states
// ============================================================



// ============================================================
// Flight Manager
// ============================================================

class FlightManager {
public:
    FlightManager();

    void begin();
    void update();

    // --------------------------------------------------------
    // Commands
    // --------------------------------------------------------

  
    // getters for main components
    Lander& getLander(){return lander;}
    Controller& getController(){return controller;}
    Mission& getMission(){return mission;}
    TelemetryManager& getTelemetryManager(){return telemetry_manager;}
    MotorManager& getMotorManager(){return motor_manager;}

    // --------------------------------------------------------
    // Status
    // --------------------------------------------------------

 

private:
    Lander lander; // state of the vehicule
    Controller controller; // outputs corrections
    Mission mission; // sets setpoints
    MotorManager motor_manager; // owns EDF and servo vanes
    TelemetryManager telemetry_manager; // manages telemetry output


    FlightStatus status;




};

#endif