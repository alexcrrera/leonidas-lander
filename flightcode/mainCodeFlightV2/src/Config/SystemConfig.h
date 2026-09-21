
#pragma once


#include "../Utilities/Utilities.h"
#include "Arduino.h"

namespace SystemConfig {

   constexpr float lander_mass = 2.0; // 
      constexpr float internal_air_speed = 35.0; // m/s, internal air speed for testing purposes
   


    
   constexpr float rho_air = 1.225; // kg/m^3, density of air at sea level
   constexpr float gravity = 9.81; // m/s^2
   constexpr float dynamic_pressure = 0.5 * rho_air * internal_air_speed * internal_air_speed; // Pa, dynamic pressure

   constexpr float linear_CL_fn_AOA = 0.1; // linear lift coefficient per degree of angle of attack

    // Vane geometry
    constexpr float vane_perimeter_CM= 20.0f;
    constexpr float vane_width_CM = 4.5f; // m^2, area of a single vane

   


   // vanes per axis
   constexpr int N_vanes_pitch = 2; // number of vanes for pitch
   constexpr int N_vanes_roll = 2; // number of vanes for roll
   constexpr int N_vanes_yaw = 4; // number of vanes for yaw
   // levers 
   constexpr float lever_arm_pitch_roll = 0.17; // m, distance from the center of mass to the EDF thrust line
   constexpr float lever_arm_yaw = 0.04; // m, distance from the center of mass to the EDF thrust line


   constexpr float vane_area_M2 = (vane_perimeter_CM/100.0f) * (vane_width_CM/100.0f); // m^2, area of a single vane

   static_assert(lander_mass > 0.0f, "lander_mass must be > 0");
   static_assert(rho_air > 0.0f, "rho_air must be > 0");
   static_assert(gravity > 0.0f, "gravity must be > 0");
   static_assert(internal_air_speed > 0.0f, "internal_air_speed must be > 0");
   static_assert(dynamic_pressure > 0.0f, "dynamic_pressure must be > 0");
   static_assert(linear_CL_fn_AOA > 0.0f, "linear_CL_fn_AOA must be > 0");
   static_assert(vane_area_M2 > 0.0f, "vane_area must be > 0");
   static_assert(N_vanes_pitch > 0, "N_vanes_pitch must be > 0");
   static_assert(N_vanes_roll > 0, "N_vanes_roll must be > 0");
   static_assert(N_vanes_yaw > 0, "N_vanes_yaw must be > 0");
   static_assert(lever_arm_pitch_roll > 0.0f, "lever_arm_pitch_roll must be > 0");
   static_assert(lever_arm_yaw > 0.0f, "lever_arm_yaw must be > 0");

static_assert(
    N_vanes_pitch * dynamic_pressure * vane_area_M2 *
    linear_CL_fn_AOA * lever_arm_pitch_roll > 0.0f,
    "Invalid pitch aerodynamic effectiveness"
);

static_assert(
    N_vanes_roll * dynamic_pressure * vane_area_M2 *
    linear_CL_fn_AOA * lever_arm_pitch_roll > 0.0f,
    "Invalid roll aerodynamic effectiveness"
);

static_assert(
    N_vanes_yaw * dynamic_pressure * vane_area_M2 *
    linear_CL_fn_AOA * lever_arm_yaw > 0.0f,
    "Invalid yaw aerodynamic effectiveness"
);
}