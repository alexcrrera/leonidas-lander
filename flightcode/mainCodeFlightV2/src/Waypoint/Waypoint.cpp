#include "Waypoint.h"



Waypoint::Waypoint(
        WaypointType type,
        const Vector3& positionNED,
        float yawDeg,
        float epsH,
        float epsV,
        float epsYaw,
        uint32_t holdTimeMs
    )
        :

        type(type),
        state(WaypointState::INACTIVE),
        target{WaypointTarget(positionNED,yawDeg)},
        epsH(epsH),
        epsV(epsV),
        epsYaw(epsYaw),
        holdTimeMs(holdTimeMs),
        entryTimeMs(0),
        holdStartTimeMs(0),
        reached(false)
        {

        }

void Waypoint::activate(){
    if (isActive()){
        return; // only allows to activate if inactive...
    }
    entryTime = millis();
    state = Waypoint::APPROACHING
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

    if(!isActive){
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
            state = WaypointState::COMPLETED

        }
        break;



    
    default:
        break;
    }
}


bool Waypoint::isActive(){
    return(state != WaypointState::INACTIVE && state != WaypointState::COMPLETED);
}
