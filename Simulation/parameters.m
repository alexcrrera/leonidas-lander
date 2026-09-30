%% 
clear;
clc;
regime_data;
gains;


gravity = 9.81; % Acceleration due to gravity (m/s^2)
rho_air = 1.225;

REGIME = TAKEOFF;
% Flight regimes:
% TAKEOFF
% NAVIGATION
% PRE_LANDING
% LANDING
lander_mass = 2.0;
lander_weight = lander_mass * gravity; % Calculate the weight of the lander

EDF_kgf_datasheet_max = 3.4; % unrealistic
EDF_thrust_factor  = 0.7;

THRUST_EDF_max = EDF_kgf_datasheet_max*gravity*EDF_thrust_factor ;
ESC_thrust_max_percentage = 90.0 / 100.0; 
ESC_thrust_min_percentage = 50.0 / 100.0;

EDF_hover_percentage = 80 / 100;
EDF_TWR  = lander_weight/THRUST_EDF_max;

DIAMETER_EDF_MM = 90.0;
DIAMETER_EDF_M = DIAMETER_EDF_MM / 1000;
EDF_area_M2 = pi * (DIAMETER_EDF_M / 2)^2;

EDF_RPM_MAX = 37000;
EDF_efficiency = 0.8; % 
EDF_POWER_MAX = 2050; % Wattage

EDF_HOVER_RPM = EDF_RPM_MAX * sqrt(EDF_TWR);

% EDF angular velocity
EDF_OMEGA_MAX = 2*pi*EDF_RPM_MAX/60;
EDF_OMEGA_HOVER = 2*pi*EDF_HOVER_RPM/60;

% Mechanical power
EDF_MECHANICAL_POWER_MAX = EDF_POWER_MAX * EDF_efficiency;

% Maximum EDF reaction torque
EDF_TORQUE_MAX = EDF_MECHANICAL_POWER_MAX / EDF_OMEGA_MAX;

% Hover EDF reaction torque
EDF_TORQUE_HOVER = EDF_TORQUE_MAX * (EDF_HOVER_RPM/EDF_RPM_MAX)^2;





 TVC_TOTAL_AUTHORITY_BUDGET_deg = 12.0; % absolute max value before stalling of the VANES
  TVC_YAW_AUTHORITY_BUDGET_deg = 6.0; % YAW (Z) authority budget


TVC_PITCH_ROLL_AUTHORITY_BUDGET_deg = TVC_TOTAL_AUTHORITY_BUDGET_deg - TVC_YAW_AUTHORITY_BUDGET_deg; % pitch/yaw (X/Y) authority budget





internal_airspeed_theo = sqrt(THRUST_EDF_max / (rho_air * EDF_area_M2));
alpha_airspeed = 0.8; % quality factor to account for losses
internal_airspeed = alpha_airspeed * internal_airspeed_theo


alpha_airspeed_rec =0.95;% 0.95; 
rec_airspeed = alpha_airspeed_rec * internal_airspeed;

% flight config
takeOff_altitude_m = 0.5;          % m
takeOff_yaw_deg = 0.0;              % deg

landing_yaw_deg = 0.0;              % deg
landing_descentStart_altitude_m = 0.25; % m

% CL
CL_alpha = 0.11; % CL = AOA * CL_alpha




% lever arms
lever_arm_pitch = 0.13;
lever_arm_roll = lever_arm_pitch;
lever_arm_yaw = 0.04;

% vanes config
N_vanes_pitch = 2;
N_vanes_roll = 2;
N_vanes_yaw = 4; 

% stator
N_vanes_stator = 4;
stator_AOA = -10;

stator_chord_CM = 5; % in cm
stator_extrusion_CM = 3; 
stator_area_M2 = (stator_chord_CM/100.0) * (stator_extrusion_CM/100.0);% m^2, area of a single vane
stator_lever_arm = 0.05;




vane_chord_CM = 5.0; % in cm
vane_extrusion_CM = 4.5; 
vane_area_M2 = (vane_chord_CM/100.0) * (vane_extrusion_CM/100.0);% m^2, area of a single vane


test = N_vanes_yaw*0.5*rho_air*internal_airspeed^2*vane_area_M2*lever_arm_yaw*CL_alpha 


yaw_inertia_SI = 0.03466;
roll_inertia_SI = 0.03961;
pitch_inertia_SI = 0.06645; % antennas included so inertia is slightly different

servo_resolution = 0.1;

YAW = struct( ...
    'inertia', yaw_inertia_SI, ...
    'N_vanes', N_vanes_yaw, ...
    'N_vanes_stator', N_vanes_stator, ...
    'lever_arm', lever_arm_yaw, ...
    'gravity', 0.0, ...
    'lander_mass', 0.0000000001); 

PITCH = struct( ...
    'inertia', pitch_inertia_SI, ...
    'N_vanes', N_vanes_pitch, ...
      'N_vanes_stator',0, ...
    'lever_arm', lever_arm_pitch, ...
    'gravity', gravity, ...
    'lander_mass', lander_mass); 



ROLL = struct( ...
    'inertia', roll_inertia_SI, ...
    'N_vanes', N_vanes_roll, ...
    'N_vanes_stator',0, ...
    'lever_arm', lever_arm_roll, ...
    'gravity', gravity, ...
    'lander_mass', lander_mass);

SEL_AXIS = YAW;