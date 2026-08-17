#pragma once
#ifndef FLIGHT_REGIME_CONFIG_H
#define FLIGHT_REGIME_CONFIG_H


#include "../Utilities/Utilities.h"
#include "Arduino.h"
#include "../Config/EpsConfig.h"
#include "../Config/SafetyBounds.h"
/*
FILE CONTAINING PARAMETERS FOR TAKE OFF AND LANDING - PARAMATERS ARE VERIFIED (BOUNDS)
*/
    struct FlightRegimeData
    {
        char name[20]; // name of the flight regime
        bool isFlying; // if false (grounded), the max rates should be set to 0, and the lander should hold position and attitude. If false, the lander can move and rotate within the bounds of the max rates.
        float max_vertical_velocity_ms = 0.0f;
        float max_horizontal_velocity_ms = 0.0f;

        float max_vertical_acceleration_ms2 = 0.0f;
        float max_horizontal_acceleration_ms2 = 0.0f;

        float max_yaw_velocity_degs = 0.0f;
        float max_pitch_roll_velocity_degs = 0.0f;



        float max_yaw_acceleration_degs2 = 0.0f;
        float max_pitch_roll_acceleration_degs2 = 0.0f;

        float pitch_roll_abort_angle_deg = 0.0f;
        float yaw_abort_angle_deg = 0.0f;

        float min_thrust_percentage = 0.0f;
        float max_thrust_percentage = 0.0f;

        uint32_t holdTimeMs = 0;
        
        EpsilonGroup epsilon_group;

    };
    


 namespace FlightRegimeConfig {

    // values for configuring take off and landing and more.
    // These are not safety limiters - see SafetyBounds.h

    
    
    constexpr FlightRegimeData GROUND{
        .name = "GROUND",
        .isFlying = false,

        .min_thrust_percentage = 0.0f,
        .max_thrust_percentage = 0.0f,
        
        // epsilon will be used to determine if the lander is in the correct position and attitude for take off or landing. The lander will not take off or land if it is not within the epsilon bounds.
        .epsilon_group = {
            .epsH = 1.0, // large tolerance as we do not care about horizontal position
            .epsV = 1.0,
            .epsYaw = 5, // alignment is not a priority
        }
    };

    
    constexpr FlightRegimeData TAKEOFF{
        
        .name = "TAKEOFF",
        .isFlying = true,



        .max_vertical_velocity_ms = 1.0,
        .max_horizontal_velocity_ms = 0.3,

        .max_vertical_acceleration_ms2 = 1,
        .max_horizontal_acceleration_ms2 = 0.1,

        .max_yaw_velocity_degs = 15.0,
        .max_pitch_roll_velocity_degs = 10.0,

        .max_yaw_acceleration_degs2 = 40.0,
        .max_pitch_roll_acceleration_degs2 = 20.0,
        .pitch_roll_abort_angle_deg = 15.0,
        .yaw_abort_angle_deg = 15.0,

        .holdTimeMs = 500 , // ms

        .epsilon_group = {
            .epsH = 0.25, // large tolerance as we do not care about horizontal position
            .epsV = 0.15,
            .epsYaw = 5, // alignment is not a priority
        }
    };


    constexpr FlightRegimeData NAVIGATION{
        .name = "NAVIGATION",
        .isFlying = true,
        .max_vertical_velocity_ms = 0.2,
        .max_horizontal_velocity_ms = 0.1,

        .max_vertical_acceleration_ms2 = 1,
        .max_horizontal_acceleration_ms2 = 0.1,

        .max_yaw_velocity_degs = 20.0,
        .max_pitch_roll_velocity_degs = 10.0,

        .max_yaw_acceleration_degs2 = 40.0,
        .max_pitch_roll_acceleration_degs2 = 20.0,
         .pitch_roll_abort_angle_deg = 15.0,
        .yaw_abort_angle_deg = 15.0,

        .holdTimeMs =   1, // ms will be overriden by the mission waypoint hold time if the lander is in a waypoint

        .epsilon_group = {
            .epsH = 0.25, // larger tolerance as we do not care about horizontal position
            .epsV = 0.2,
            .epsYaw = 5, // alignment is not a priority
        } 
    };


    

    constexpr FlightRegimeData PRE_LANDING{
        .name = "PRE_LANDING",
        .isFlying = true,
        .max_vertical_velocity_ms = 0.2,
        .max_horizontal_velocity_ms = 0.1,

        .max_vertical_acceleration_ms2 = 1,
        .max_horizontal_acceleration_ms2 = 0.1,

        .max_yaw_velocity_degs = 20.0,
        .max_pitch_roll_velocity_degs = 10.0,

        .max_yaw_acceleration_degs2 = 40.0,
        .max_pitch_roll_acceleration_degs2 = 20.0,
         .pitch_roll_abort_angle_deg = 15.0,
        .yaw_abort_angle_deg = 15.0,

        .holdTimeMs =   1, // ms

        .epsilon_group = {
            .epsH = 0.25, // large tolerance as we do not care about horizontal position
            .epsV = 0.2,
            .epsYaw = 5, // alignment is not a priority
        } 
    };

    constexpr FlightRegimeData LANDING{
        .name = "LANDING",
        .isFlying = true,
        .max_vertical_velocity_ms = 0.3,
        .max_horizontal_velocity_ms = 0.1,

        .max_vertical_acceleration_ms2 = 1,
        .max_horizontal_acceleration_ms2 = 0.1,

        .max_yaw_velocity_degs = 20.0,
        .max_pitch_roll_velocity_degs = 10.0,

        .max_yaw_acceleration_degs2 = 40.0,
        .max_pitch_roll_acceleration_degs2 = 20.0,

         .pitch_roll_abort_angle_deg = 15.0,
        .yaw_abort_angle_deg = 15.0,

        .holdTimeMs =   1, // ms

        .epsilon_group = {
            .epsH = 0.35, // large tolerance as we do not care about horizontal position
            .epsV = 0.15,
            .epsYaw = 15, // alignment is not a priority
        }
    };


    constexpr bool verifyFlightRegimeData(const FlightRegimeData& regime)
{
    // Verify that all regime data is bounded.
    if(regime.isFlying){
    
        if (regime.max_vertical_velocity_ms >
        SafetyBounds::max_vertical_velocity_ms) {
        return false;
    }

    if (regime.max_horizontal_velocity_ms >
        SafetyBounds::max_horizontal_velocity_ms) {
        return false;
    }

    if (regime.max_vertical_acceleration_ms2 >
        SafetyBounds::max_vertical_acceleration_ms2) {
        return false;
    }

    if (regime.max_horizontal_acceleration_ms2 >
        SafetyBounds::max_horizontal_acceleration_ms2) {
        return false;
    }

    if (regime.max_yaw_velocity_degs >
        SafetyBounds::max_yaw_velocity_degs) {
        return false;
    }

    if (regime.max_pitch_roll_velocity_degs >
        SafetyBounds::max_pitch_roll_velocity_degs) {
        return false;
    }

    if (regime.max_yaw_acceleration_degs2 >
        SafetyBounds::max_yaw_acceleration_degs2) {
        return false;
    }

    if (regime.max_pitch_roll_acceleration_degs2 >
        SafetyBounds::max_pitch_roll_acceleration_degs2) {
        return false;
    }   

    if (regime.pitch_roll_abort_angle_deg >
        SafetyBounds::pitch_roll_abort_angle_deg) {
        return false;
    }

    if (regime.yaw_abort_angle_deg >
        SafetyBounds::yaw_abort_angle_deg) {
        return false;
    }

}




    if(!regime.isFlying){ // values should be 0 if not flying

           if (regime.max_vertical_velocity_ms != 0.0f) {
        return false;
    }

    if (regime.max_horizontal_velocity_ms != 0.0f) {
        return false;
    }

    if (regime.max_vertical_acceleration_ms2 != 0.0f) {
        return false;
    }

    if (regime.max_horizontal_acceleration_ms2 != 0.0f) {
        return false;
    }

    if (regime.max_yaw_velocity_degs != 0.0f) {
        return false;
    }

    if (regime.max_pitch_roll_velocity_degs != 0.0f) {
        return false;
    }

    if (regime.max_yaw_acceleration_degs2 != 0.0f) {
        return false;
    }

    if (regime.max_pitch_roll_acceleration_degs2 != 0.0f) {
        return false;
    }   
    
    }

    // END OF VERIFICATION OF BOUNDS


     if(regime.holdTimeMs < 0){
        return false;
    }


    if (!EpsConfig::isValidEpsilonGroup(regime.epsilon_group)) {
        return false;
    }

    return true;
}
    static_assert(verifyFlightRegimeData(TAKEOFF),"TAKE OFF REGIME CONFIG IS WRONG");
    static_assert(verifyFlightRegimeData(LANDING),"LANDING REGIME CONFIG IS WRONG");
    static_assert(verifyFlightRegimeData(PRE_LANDING),"PRE_LANDING REGIME CONFIG IS WRONG");
 }
#endif
