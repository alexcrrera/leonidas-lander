#pragma once
#ifndef FLIGHT_REGIME_CONFIG_H
#define FLIGHT_REGIME_CONFIG_H

#include "../Utilities/Utilities.h"
#include "Arduino.h"
#include "../Config/EpsConfig.h"
#include "../Config/SafetyBounds.h"

/*
FILE CONTAINING PARAMETERS FOR TAKE OFF AND LANDING - PARAMETERS ARE VERIFIED (BOUNDS)
*/

struct FlightRegimeData
{
    char name[20];
    bool isFlying;

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

    // Values for configuring take off, navigation and landing.
    // These are not safety limiters - see SafetyBounds.h


 

    constexpr FlightRegimeData TAKEOFF{
        .name = "TAKEOFF",
        .isFlying = true,

        .max_vertical_velocity_ms = 0.750f,
        .max_horizontal_velocity_ms = 0.5f,

        .max_vertical_acceleration_ms2 = 3.0f,
        .max_horizontal_acceleration_ms2 = 2.0f,

        .max_yaw_velocity_degs = 30.0f,
        .max_pitch_roll_velocity_degs = 30.0f,

        .max_yaw_acceleration_degs2 = 2500.0f,
        .max_pitch_roll_acceleration_degs2 = 5000.0f,

        .pitch_roll_abort_angle_deg = 7.0f,
        .yaw_abort_angle_deg = 15.0f,

        .min_thrust_percentage = 0.0f,
        .max_thrust_percentage = 0.0f,

        .holdTimeMs = 1000,

        .epsilon_group = {
            .epsH = 0.25f,
            .epsV = 0.05f,
            .epsYaw = 999.0f,
        }
    };

       constexpr FlightRegimeData GROUND = TAKEOFF; // OVERRIDE FOR TESTING PURPOSES - GROUND AND TAKEOFF ARE THE SAME
       /*
       
       {
        .name = "GROUND",
        .isFlying = false,

        .min_thrust_percentage = 0.0f,
        .max_thrust_percentage = 0.0f,

        .epsilon_group = {
            .epsH = 1.0f,
            .epsV = 1.0f,
            .epsYaw = 999.0f,
        }
    };*/





    constexpr FlightRegimeData NAVIGATION = TAKEOFF; // OVERRIDE FOR TESTING PURPOSES - NAVIGATION AND TAKEOFF ARE THE SAME
    
    /*{
        .name = "NAVIGATION",
        .isFlying = true,

        .max_vertical_velocity_ms = 0.2f,
        .max_horizontal_velocity_ms = 0.1f,

        .max_vertical_acceleration_ms2 = 3.0f,
        .max_horizontal_acceleration_ms2 = 2.0f,

        .max_yaw_velocity_degs = 20.0f,
        .max_pitch_roll_velocity_degs = 10.0f,

        .max_yaw_acceleration_degs2 = 40.0f,
        .max_pitch_roll_acceleration_degs2 = 20.0f,

        .pitch_roll_abort_angle_deg = 15.0f,
        .yaw_abort_angle_deg = 15.0f,

        .min_thrust_percentage = 0.0f,
        .max_thrust_percentage = 0.0f,

        .holdTimeMs = 1000,

        .epsilon_group = {
            .epsH = 0.25f,
            .epsV = 0.05f,
            .epsYaw = 999.0f,
        }
    };
    */


    constexpr FlightRegimeData PRE_LANDING{
        .name = "PRE_LANDING",
        .isFlying = true,

        .max_vertical_velocity_ms = 0.2f,
        .max_horizontal_velocity_ms = 0.1f,

        .max_vertical_acceleration_ms2 = 3.0f,
        .max_horizontal_acceleration_ms2 = 2.0f,

        .max_yaw_velocity_degs = 20.0f,
        .max_pitch_roll_velocity_degs = 10.0f,

        .max_yaw_acceleration_degs2 = 40.0f,
        .max_pitch_roll_acceleration_degs2 = 20.0f,

        .pitch_roll_abort_angle_deg = 15.0f,
        .yaw_abort_angle_deg = 15.0f,

        .min_thrust_percentage = 0.0f,
        .max_thrust_percentage = 0.0f,

        .holdTimeMs = 1000,

        .epsilon_group = {
            .epsH = 0.25f,
            .epsV = 0.07f,
            .epsYaw = 999.0f,
        }
    };


    constexpr FlightRegimeData LANDING{
        .name = "LANDING",
        .isFlying = true,

        .max_vertical_velocity_ms = 0.3f,
        .max_horizontal_velocity_ms = 0.1f,

        .max_vertical_acceleration_ms2 = 3.0f,
        .max_horizontal_acceleration_ms2 = 2.0f,

        .max_yaw_velocity_degs = 20.0f,
        .max_pitch_roll_velocity_degs = 10.0f,

        .max_yaw_acceleration_degs2 = 40.0f,
        .max_pitch_roll_acceleration_degs2 = 20.0f,

        .pitch_roll_abort_angle_deg = 15.0f,
        .yaw_abort_angle_deg = 15.0f,

        .min_thrust_percentage = 0.0f,
        .max_thrust_percentage = 0.0f,

        .holdTimeMs = 1,

        .epsilon_group = {
            .epsH = 0.35f,
            .epsV = 0.07f,
            .epsYaw = 999.0f,
        }
    };


    constexpr bool verifyFlightRegimeData(const FlightRegimeData& regime)
    {
        if (regime.isFlying) {

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


        if (!regime.isFlying) {

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


        if (!EpsConfig::isValidEpsilonGroup(regime.epsilon_group)) {
            return false;
        }

        return true;
    }


    static_assert(verifyFlightRegimeData(TAKEOFF), "TAKEOFF REGIME CONFIG IS WRONG");
    static_assert(verifyFlightRegimeData(NAVIGATION), "NAVIGATION REGIME CONFIG IS WRONG");
    static_assert(verifyFlightRegimeData(PRE_LANDING), "PRE_LANDING REGIME CONFIG IS WRONG");
    static_assert(verifyFlightRegimeData(LANDING), "LANDING REGIME CONFIG IS WRONG");

}

#endif