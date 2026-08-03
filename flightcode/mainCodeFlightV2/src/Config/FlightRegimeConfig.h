
#include "Utilities.h"
#include "Arduino.h"
#include "EpsConfig.h"
#include 
/*
FILE CONTAINING PARAMETERS FOR TAKE OFF AND LANDING - PARAMATERS ARE VERIFIED (BOUNDS)
*/
struct FlightRegimeData
{
    float max_vertical_velocity_ms;
    float max_horizontal_velocity_ms;

    float max_vertical_acceleration_ms2;
    float max_horizontal_acceleration_ms2

    float max_yaw_velocity_degs;
    float max_pitch_roll_velocity_degs;

    float max_yaw_acceleration_degs2;
    float max_pitch_roll_acceleration_degs2;

    uint32_t holdTimeMs;
    
    EpsilonGroup epsilon_group;

}
 


 namespace FlightRegimeConfig {

    // values for configuring take off and landing and more.
    // These are not safety limiters - see SafetyBounds.h


    
    constexpr FlightRegimeData TAKEOFF{
        .max_vertical_velocity_ms = 0.2,
        .max_horizontal_velocity_ms = 0.1,

        .max_vertical_acceleration_ms2 = 1,
        .max_horizontal_acceleration_ms2 = 0.1,

        .max_yaw_velocity_degs = 15.0,
        .max_pitch_roll_velocity_degs = 10.0,

        .max_yaw_acceleration_degs2 = 40.0,
        .max_pitch_roll_acceleration_degs2 = 20.0,

        .holdTimeMs = 500 , // ms

        .epsilon_group = {
            .epsH = 0.25, // large tolerance as we do not care about horizontal position
            .epsV = 0.15,
            .epsYaw = 5, // alignment is not a priority
        }
    }


    

    constexpr FlightRegimeData PRE_LANDING{
        .max_vertical_velocity_ms = 0.2,
        .max_horizontal_velocity_ms = 0.1,

        .max_vertical_acceleration_ms2 = 1,
        .max_horizontal_acceleration_ms2 = 0.1,

        .max_yaw_velocity_degs = 20.0,
        .max_pitch_roll_velocity_degs = 10.0,

        .max_yaw_acceleration_degs2 = 40.0,
        .max_pitch_roll_acceleration_degs2 = 20.0,

        .holdTimeMs =   1 // ms

        .epsilon_group = {
            .epsH = 0.25, // large tolerance as we do not care about horizontal position
            .epsV = 0.2,
            .epsYaw = 5, // alignment is not a priority
        } 
    }

    constexpr FlightRegimeData LANDING{
        .max_vertical_velocity_ms = 0.3,
        .max_horizontal_velocity_ms = 0.1,

        .max_vertical_acceleration_ms2 = 1,
        .max_horizontal_acceleration_ms2 = 0.1,

        .max_yaw_velocity_degs = 20.0,
        .max_pitch_roll_velocity_degs = 10.0,

        .max_yaw_acceleration_degs2 = 40.0,
        .max_pitch_roll_acceleration_degs2 = 20.0,

        .holdTimeMs =   1 // ms

        .epsilon_group = {
            .epsH = 0.35, // large tolerance as we do not care about horizontal position
            .epsV = 0.15,
            .epsYaw = 15, // alignment is not a priority
        }
    }


        bool verifyFlightRegimeData(FlightRegimeData regime){
        
        // verifies that all regime data is bounded

        if(regime.max_vertical_velocity_ms >SafetyBounds::max_vertical_velocity_ms ){
            return(false);}

        if(regime.max_horizontal_velocity_ms > SafetyBounds::max_horizontal_velocity_ms){
            return(false);}

        if(regime.max_vertical_acceleration_ms2 > SafetyBounds::max_vertical_acceleration_ms2){
            return(false);}

        if(regime.max_vertical_acceleration_ms2 > SafetyBounds::max_horizontal_acceleration_ms2){
            return(false);}

        if(regime.max_yaw_velocity_degs > SafetyBounds::max_yaw_velocity_degs){
            return(false);}

        if(regime.max_pitch_roll_velocity_degs > SafetyBounds::max_pitch_roll_velocity_degs){
            return(false);}

        if(regime.max_yaw_acceleration_degs2 > SafetyBounds::max_yaw_acceleration_degs2){
            return(false);}

        if(regime.max_pitch_roll_acceleration_degs2 > SafetyBounds::max_pitch_roll_acceleration_degs2){
            return(false);}


        if(regime.max_yaw_acceleration_degs2 > SafetyBounds::max_pitch_roll_acceleration_degs2){
            return(false);}

        if(regime.pitch_roll_abort_angle > SafetyBounds::max_pitch_roll_acceleration_degs2){
            return(false);}

        if(!isValidEpsilonGroup(regime.epsilon_group)){
            return(false);
        }


        return(true); // all good
    }

    static_assert(verifyFlightRegimeData(TAKEOFF,"TAKE OFF REGIME CONFIG IS WRONG"));
    static_assert(verifyFlightRegimeData(LANDING,"TAKE OFF REGIME CONFIG IS WRONG"));
    static_assert(verifyFlightRegimeData(PRE_LANDING,"TAKE OFF REGIME CONFIG IS WRONG"));
 }

