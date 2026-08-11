#include "FlightManager.h"


// ============================================================
// Constructor
// ============================================================

FlightManager::FlightManager()
    : 
      telemetry_manager(this)
{
}


// ============================================================
// Initialization
// ============================================================

void FlightManager::begin()
{
    status.state = FlightState::Boot;
    status.previousState = FlightState::Boot;

    status.armed = false;
    status.abortRequested = false;

    status.stateEntryTime = millis();
    status.flightStartTime = 0;
    status.timestamp = millis();

    armRequested = false;
    disarmRequested = false;
    takeoffRequested = false;
    landingRequested = false;

    lander.begin();

    telemetry_manager.begin();

    

   
}


// ============================================================
// Main update
// ============================================================

void FlightManager::update()
{
    const uint32_t now = millis();


    // --------------------------------------------------------
    // Update sensors and estimated lander state first
    // --------------------------------------------------------
    
    lander.update();

    


    // --------------------------------------------------------
    // Update telemetry
    // --------------------------------------------------------

    telemetry_manager.update(now);


    // --------------------------------------------------------
    // Evaluate safety conditions
    // --------------------------------------------------------

    checkFailsafes();


    // --------------------------------------------------------
    // Update flight state machine
    // --------------------------------------------------------

    updateStateMachine();


    // --------------------------------------------------------
    // Update timestamp
    // --------------------------------------------------------

    status.timestamp = now;
}


// ============================================================
// State machine
// ============================================================

void FlightManager::updateStateMachine()
{
    switch (status.state)
    {
        case FlightState::Boot:
            handleBoot();
            break;

        case FlightState::Standby:
            handleStandby();
            break;

        case FlightState::Armed:
            handleArmed();
            break;

        case FlightState::Takeoff:
            handleTakeoff();
            break;

        case FlightState::Flight:
            handleFlight();
            break;

        case FlightState::Landing:
            handleLanding();
            break;

        case FlightState::Landed:
            handleLanded();
            break;

        case FlightState::Abort:
            handleAbort();
            break;
    }
}


// ============================================================
// State transition
// ============================================================

void FlightManager::transitionTo(FlightState newState)
{
    if (newState == status.state)
    {
        return;
    }


    status.previousState = status.state;
    status.state = newState;
    status.stateEntryTime = millis();


    onStateEntry(newState);
}


// ============================================================
// State entry actions
// ============================================================

void FlightManager::onStateEntry(FlightState newState)
{
    switch (newState)
    {
        case FlightState::Boot:

            status.armed = false;

            break;


        case FlightState::Standby:

            status.armed = false;

            break;


        case FlightState::Armed:

            status.armed = true;

            break;


        case FlightState::Takeoff:

            status.armed = true;
            status.flightStartTime = millis();

            break;


        case FlightState::Flight:

            status.armed = true;

            break;


        case FlightState::Landing:

            status.armed = true;

            break;


        case FlightState::Landed:

            status.armed = false;

            break;


        case FlightState::Abort:

            status.armed = false;

            break;
    }
}


// ============================================================
// BOOT
// ============================================================

void FlightManager::handleBoot()
{
    if (readyForStandby())
    {
        transitionTo(FlightState::Standby);
    }
}


// ============================================================
// STANDBY
// ============================================================

void FlightManager::handleStandby()
{
    if (!armRequested)
    {
        return;
    }


    armRequested = false;


    if (readyToArm())
    {
        transitionTo(FlightState::Armed);
    }
}


// ============================================================
// ARMED
// ============================================================

void FlightManager::handleArmed()
{
    if (disarmRequested)
    {
        disarmRequested = false;

        transitionTo(FlightState::Standby);

        return;
    }


    if (takeoffRequested)
    {
        takeoffRequested = false;

        transitionTo(FlightState::Takeoff);
    }
}


// ============================================================
// TAKEOFF
// ============================================================

void FlightManager::handleTakeoff()
{
    /*
     * Takeoff control will be implemented later.
     *
     * This state will eventually:
     *
     * - establish thrust
     * - detect liftoff
     * - ramp altitude setpoint
     * - limit vertical velocity
     * - transition to Flight when takeoff is complete
     */
}


// ============================================================
// FLIGHT
// ============================================================

void FlightManager::handleFlight()
{
    if (landingRequested)
    {
        landingRequested = false;

        transitionTo(FlightState::Landing);
    }
}


// ============================================================
// LANDING
// ============================================================

void FlightManager::handleLanding()
{
    /*
     * Landing control will be implemented later.
     *
     * This state will eventually:
     *
     * - command controlled descent
     * - limit descent velocity
     * - detect ground proximity
     * - detect touchdown
     * - transition to Landed
     */
}


// ============================================================
// LANDED
// ============================================================

void FlightManager::handleLanded()
{
    /*
     * For now Landed is a terminal state.
     *
     * Later we can decide whether the system should:
     *
     * - automatically return to Standby
     * - require an explicit reset
     * - permit re-arming
     */
}


// ============================================================
// ABORT
// ============================================================

void FlightManager::handleAbort()
{
    /*
     * Abort behavior will depend on flight condition.
     *
     * Do not implement "motors off" blindly here because
     * an airborne abort may require controlled recovery.
     */
}


// ============================================================
// Safety checks
// ============================================================

void FlightManager::checkFailsafes()
{
    if (status.abortRequested)
    {
        if (status.state != FlightState::Abort)
        {
            transitionTo(FlightState::Abort);
        }

        return;
    }


    /*
     * Automatic failsafes will be added here later.
     *
     * Examples:
     *
     * - attitude solution lost
     * - altitude solution lost
     * - stale navigation solution
     * - excessive attitude
     * - communication loss
     * - motor failure
     * - low battery
     */
}


// ============================================================
// Readiness checks
// ============================================================

bool FlightManager::readyForStandby() const
{
    /*
     * At this stage require attitude and altitude.
     *
     * Position and velocity are intentionally not required yet
     * because RTK / velocity estimation have not been integrated.
     */

    if (!lander.hasAttitudeSolution())
    {
        return false;
    }


    if (!lander.hasAltitudeSolution())
    {
        return false;
    }


    return true;
}


bool FlightManager::readyToArm() const
{
    if (!lander.hasAttitudeSolution())
    {
        return false;
    }


    if (!lander.hasAltitudeSolution())
    {
        return false;
    }


    return true;
}


// ============================================================
// Commands
// ============================================================

bool FlightManager::requestArm()
{
    armRequested = true;

    return true;
}


bool FlightManager::requestDisarm()
{
    disarmRequested = true;

    return true;
}


bool FlightManager::requestTakeoff()
{
    takeoffRequested = true;

    return true;
}


bool FlightManager::requestLanding()
{
    landingRequested = true;

    return true;
}


bool FlightManager::requestAbort()
{
    status.abortRequested = true;

    return true;
}


// ============================================================
// Status access
// ============================================================

FlightState FlightManager::getState() const
{
    return status.state;
}


const FlightStatus& FlightManager::getStatus() const
{
    return status;
}


uint32_t FlightManager::getTimeInState() const
{
    return millis() - status.stateEntryTime;
}


uint32_t FlightManager::getFlightTime() const
{
    if (status.flightStartTime == 0)
    {
        return 0;
    }


    return millis() - status.flightStartTime;
}


bool FlightManager::isArmed() const
{
    return status.armed;
}


bool FlightManager::isFlying() const
{
    return (
        status.state == FlightState::Takeoff ||
        status.state == FlightState::Flight ||
        status.state == FlightState::Landing
    );
}


bool FlightManager::isAborted() const
{
    return status.state == FlightState::Abort;
}


// ============================================================
// Lander access
// ============================================================


// ============================================================
// State string
// ============================================================

const char* FlightManager::stateToString(FlightState state) const
{
    switch (state)
    {
        case FlightState::Boot:
            return "BOOT";

        case FlightState::Standby:
            return "STANDBY";

        case FlightState::Armed:
            return "ARMED";

        case FlightState::Takeoff:
            return "TAKEOFF";

        case FlightState::Flight:
            return "FLIGHT";

        case FlightState::Landing:
            return "LANDING";

        case FlightState::Landed:
            return "LANDED";

        case FlightState::Abort:
            return "ABORT";

        default:
            return "UNKNOWN";
    }
}


// ============================================================
// Debug output
// ============================================================
