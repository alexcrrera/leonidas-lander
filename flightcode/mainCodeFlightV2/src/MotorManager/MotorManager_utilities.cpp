#include "MotorManager.h"

float MotorManager::tau_pitch_to_alpha_deg(float tau_pitch) {
    return tau_pitch /
           (SystemConfig::N_vanes_pitch *
            SystemConfig::dynamic_pressure *
            SystemConfig::vane_area_M2 *
            SystemConfig::linear_CL_fn_AOA *
            SystemConfig::lever_arm_pitch_roll);
}

float MotorManager::tau_roll_to_beta_deg(float tau_roll) {
    // in practice idetical to tau_pitch_to_alpha_deg, but we keep it separate in case something changes in the future
    return tau_roll /
           (SystemConfig::N_vanes_roll *
            SystemConfig::dynamic_pressure *
            SystemConfig::vane_area_M2 *
            SystemConfig::linear_CL_fn_AOA *
            SystemConfig::lever_arm_pitch_roll);
}

float MotorManager::tau_yaw_to_gamma_deg(float tau_yaw) {
    return tau_yaw /
           (SystemConfig::N_vanes_yaw *
            SystemConfig::dynamic_pressure *
            SystemConfig::vane_area_M2 *
            SystemConfig::linear_CL_fn_AOA *
            SystemConfig::lever_arm_yaw);
}