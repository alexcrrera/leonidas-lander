#ifndef FLIGHT_MANAGER_H
#define FLIGHT_MANAGER_H

#include <Arduino.h>
#include "../Lander/Lander.h"
#include "Mission.h"
#include "MotorManager.h"
// ============================================================
// Flight states
// ============================================================

enum class FlightState : uint8_t {
    Boot,
    Standby,
    Armed,
    Takeoff,
    Flight,
    Landing,
    Landed,
    Abort
};

// ============================================================
// Flight manager status
// ============================================================

struct FlightStatus {
    FlightState state = FlightState::Boot;
    FlightState previousState = FlightState::Boot;

    bool armed = false;
    bool abortRequested = false;

    uint32_t stateEntryTime = 0;
    uint32_t flightStartTime = 0;
    uint32_t timestamp = 0;
};

// ============================================================
// Flight Manager
// ============================================================

class FlightManager {
public:
    FlightManager(Lander& lander);

    void begin();
    void update();

    // --------------------------------------------------------
    // Commands
    // --------------------------------------------------------

    void requestArm();
    void requestDisarm();
    void requestTakeoff();
    bool requestLanding();
    bool requestAbort();

    // --------------------------------------------------------
    // Status
    // --------------------------------------------------------

    FlightState getState() const;
    const FlightStatus& getStatus() const;

    uint32_t getTimeInState() const;
    uint32_t getFlightTime() const;

    bool isArmed() const;
    bool isFlying() const;
    bool isAborted() const;

    void printStatus(Stream& serialPort = Serial) const;

private:
    Lander& lander; // state of the vehicule
    Controller controller; // outputs corrections
    Mission mission; // sets setpoints
    MotorManager motor_manager; // owns EDF and servo vanes


    FlightStatus status;


    // --------------------------------------------------------
    // Command requests
    // --------------------------------------------------------

    bool armRequested = false;
    bool disarmRequested = false;
    bool takeoffRequested = false;
    bool landingRequested = false;

    // --------------------------------------------------------
    // State machine
    // --------------------------------------------------------

    void updateStateMachine();

    void transitionTo(FlightState newState);
    void onStateEntry(FlightState newState);

    void handleBoot();
    void handleStandby();
    void handleArmed();
    void handleTakeoff();
    void handleFlight();
    void handleLanding();
    void handleLanded();
    void handleAbort();

    // --------------------------------------------------------
    // Safety
    // --------------------------------------------------------

    bool readyForStandby() const;
    bool readyToArm() const;

    void checkFailsafes();

    


    // --------------------------------------------------------
    // Utilities
    // --------------------------------------------------------

    const char* stateToString(FlightState state) const;
};

#endif