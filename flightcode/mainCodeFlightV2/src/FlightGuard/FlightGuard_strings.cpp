#include "FlightGuard.h"


String FlightGuard::motorEnabledStatus() const{
    String status = (overrideFlags.EDF_enabled) ? "[ON]" : "[OFF]";    
    return(status);
}



String FlightGuard::motorSafetyStatus() const{
    String status = (overrideFlags.EDF_armed) ? "[ARMED]" : "[DISARMED]";    
    return(status);
}