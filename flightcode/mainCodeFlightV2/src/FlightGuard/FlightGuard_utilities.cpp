#include "../FlightGuard/FlightGuard.h"
#include "../Config/SafetyBounds.h"
#include "../FlightGuard/FlightGuard_utilities.h"



bool FlightGuardUtilities::isRollPitchWithinBounds(float pitch_deg, float roll_deg) {
    auto max_val_deg = SafetyBounds::pitch_roll_abort_angle_deg;
    return Utilities::isSimpleBouded(pitch_deg, max_val_deg) && Utilities::isSimpleBouded(roll_deg, max_val_deg);
}


bool FlightGuardUtilities::isYawWithinBounds(float yaw_deg) {
    auto max_val_deg = SafetyBounds::yaw_abort_angle_deg;
    return Utilities::isSimpleBouded(yaw_deg, max_val_deg);
}

bool FlightGuardUtilities::isVerticalVelocityWithinBounds(float speed) {
    auto max_speed = SafetyBounds::max_vertical_velocity_ms;
    return Utilities::isSimpleBouded(speed, max_speed);
}

bool FlightGuardUtilities::isHorizontalVelocityWithinBounds(float velocity_N, float velocity_E) {
    auto max_speed = SafetyBounds::max_horizontal_velocity_ms;
    return Utilities::isSimpleBouded(velocity_N, max_speed) && Utilities::isSimpleBouded(velocity_E, max_speed);
}




void FlightGuardUtilities::resetOverrideFlags(FlightGuardOverrideFlags& flags){

    flags.vertical_overspeed_ok = true;
    flags.horizontal_overspeed_ok = true;
    flags.altitude_limit_ok = true;
    flags.NED_horizontal_boundary_ok= true;
    flags.NED_vertical_boundary_ok = true;
    flags.roll_pitch_ok = true;
    flags.yaw_ok = true;
}