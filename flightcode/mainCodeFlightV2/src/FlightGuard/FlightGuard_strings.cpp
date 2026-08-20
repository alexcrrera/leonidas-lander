#include "FlightGuard.h"


String FlightGuard::motorEnabledStatus() const{
    String status = (overrideFlags.EDF_enabled) ? "[ON]" : "[OFF]";    
    return(status);
}



String FlightGuard::motorSafetyStatus() const{
    String status = (overrideFlags.EDF_armed) ? "[ARMED]" : "[DISARMED]";    
    return(status);
}



String FlightGuard::getFlightGuardStatus() const{
    String status = "FlightGuard Status:\n";
    status += "EDF Enabled: " + String(overrideFlags.EDF_enabled) + "\n";
    status += "EDF Armed: " + String(overrideFlags.EDF_armed) + "\n";
    status += "TVC Enabled: " + String(overrideFlags.TVC_enabled) + "\n";
    status += "Vertical Overspeed OK: " + String(overrideFlags.vertical_overspeed_ok) + "\n";
    status += "Horizontal Overspeed OK: " + String(overrideFlags.horizontal_overspeed_ok) + "\n";
    status += "Altitude Limit OK: " + String(overrideFlags.altitude_limit_ok) + "\n";
    status += "NED Horizontal Boundary OK: " + String(overrideFlags.NED_horizontal_boundary_ok) + "\n";
    status += "NED Vertical Boundary OK: " + String(overrideFlags.NED_vertical_boundary_ok) + "\n";
    status += "Roll/Pitch OK: " + String(overrideFlags.roll_pitch_ok) + "\n";
    status += "Yaw OK: " + String(overrideFlags.yaw_ok) + "\n";
    return status;
}