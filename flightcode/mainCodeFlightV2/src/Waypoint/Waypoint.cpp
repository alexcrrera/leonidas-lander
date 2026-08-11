#include "Waypoint.h"

Waypoint::Waypoint() : type(WaypointType::NAVIGATION), 
state(WaypointState::INACTIVE), target{{0.0f, 0.0f, 0.0f}, 0.0f}, epsH(EpsConfig::epsH.default_), epsV(EpsConfig::epsV.default_), epsYaw(EpsConfig::epsYaw.default_), holdTimeMs(MissionConfig::holdTimeMs.default_), holdStartTimeMs(0), entryTimeMs(0), reached(false)
{
}
Waypoint::Waypoint(
    WaypointType type,
    const Vector3& positionNED,
    float yawDeg,
    uint32_t holdTimeMs,
    EpsilonGroup epsilon_group
)
    :
    type(type),
    state(WaypointState::INACTIVE),
    target{positionNED, yawDeg},
    epsH(epsilon_group.epsH),
    epsV(epsilon_group.epsV),
    epsYaw(epsilon_group.epsYaw),
    holdTimeMs(holdTimeMs),
    holdStartTimeMs(0),
    entryTimeMs(0),
    reached(false)
{
}
void Waypoint::activate(){
    if (isActive()){
        return; // only allows to activate if inactive...
    }
    entryTimeMs = millis();
    state = WaypointState::APPROACHING;
}


const WaypointTarget& Waypoint::getTarget() const{
    return(target);
}

WaypointType Waypoint::getType() const{
    return(type);
}


WaypointState Waypoint::getState() const{

    return(state);
}



void Waypoint::update(  const Vector3& currentPositionNED,float yawDeg){

    if(!isActive()){
        return; // do nothing since wp. not activated
    }

    bool positionReached = Utilities::isWithinEps_NED(
        epsH,
        epsV,
        currentPositionNED,
        target.positionNED);
    
    bool yawReached = Utilities::isWithinEps_Yaw(
        epsYaw,yawDeg,target.yawDeg);

    reached = positionReached && yawReached;

    switch (state)
    {

    case WaypointState::APPROACHING:
        if(reached){
            holdStartTimeMs = millis();
            state = WaypointState::HOLDING;
        }
        break;

    
    case WaypointState::HOLDING:

        if(!reached){ // need to reset hold time since we left the bounding box
            holdStartTimeMs = 0;
            state = WaypointState::APPROACHING;
            break;

        }

        if(millis()- holdStartTimeMs >= holdTimeMs){
            state = WaypointState::COMPLETED;

        }
        break;



    
    default:
        break;
    }
}


bool Waypoint::isActive(){
    return(state != WaypointState::INACTIVE && state != WaypointState::COMPLETED);
}
