#include "Waypoint.h"
#include "../Utilities/Utilities.h"

//#include "../Config/MissionConfig.h"

Waypoint::Waypoint() : 
    target{{0.0f, 0.0f, 0.0f}, 0.0f},
    epsilon_group(EpsilonGroup {}),
    holdTimeMs(MissionConfig::holdTimeMs.default_),
    holdStartTimeMs(0), 
    entryTimeMs(0)
{
}

Waypoint::Waypoint(
    const NED_coordinates& positionNED,
    float yaw_deg,
    uint32_t holdTimeMs,
    EpsilonGroup epsilon_group
)
    :
    
    target{positionNED, yaw_deg},
    epsilon_group(epsilon_group),
    holdTimeMs(holdTimeMs),
    holdStartTimeMs(0),
    entryTimeMs(0)
   
{
}




const WaypointTarget& Waypoint::getTarget() const{
    return(target);
}


void Waypoint::updatePosition(const NED_coordinates& positionNED, float yaw_deg){
    target.positionNED = positionNED;
    target.yaw_deg = yaw_deg;
}




String Waypoint::getDataAsString() const{
    String data = "Waypoint Data:\n";
    data += "Position NED: N: " + String(target.positionNED.North_SI) + ", E: " + String(target.positionNED.East_SI) + ", D: " + String(target.positionNED.Down_SI) + "\n";
    data += "Yaw: " + String(target.yaw_deg) + "\n";
    data += "Hold Time: " + String(holdTimeMs) + " ms\n";
    data += "Epsilon Group: epsH: " + String(epsilon_group.epsH) + ", epsV: " + String(epsilon_group.epsV) + ", epsYaw: " + String(epsilon_group.epsYaw) + "\n";
    return(data);
}


bool Waypoint::isReached(const NED_coordinates& currentPositionNED, float currentYaw_deg) const {
    return Utilities::isWithinEps_NED(epsilon_group.epsH, epsilon_group.epsV, currentPositionNED, target.positionNED) &&
           Utilities::isWithinEps_Yaw(epsilon_group.epsYaw, currentYaw_deg, target.yaw_deg);
}