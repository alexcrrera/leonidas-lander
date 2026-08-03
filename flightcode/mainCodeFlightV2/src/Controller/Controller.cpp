#include "Mission.h"
#include <cmath>
#include "FlightRegimeConfig.h"
#include "CoordinateConfig.h"



Controller::Controller(){
 PID_position = {
    .axis_x = PID(ControllerPID::PID_gains_position_North),
    .axis_y = PID(ControllerPID::PID_gains_position_East),
    .axis_z = PID(ControllerPID::PID_gains_position_Down)
 };

 PID_velocity = {
    .axis_x = PID(ControllerPID::PID_gains_velocity_North),
    .axis_y = PID(ControllerPID::PID_gains_velocity_East),
    .axis_z = PID(ControllerPID::PID_gains_velocity_Down)
 };
 
 PID_attitude {
    .axis_x = PID(ControllerPID::PID_gains_attitude_roll),
    .axis_y = PID(ControllerPID::PID_gains_velocity_pitch),
    .axis_z = PID(ControllerPID::PID_gains_velocity_yaw)
 };


  PID_body_rates {
    .axis_x = PID(ControllerPID::PID_gains_body_rates_roll),
    .axis_y = PID(ControllerPID::PID_gains_body_rates_pitch),
    .axis_z = PID(ControllerPID::PID_gains_body_rates_yaw)
 };


}

void Controller::update(float dt, const LanderState& lander_state, FlightRegimeData current_regime){

   if(!active){
        return;
    }
    NED_coordinates current_position = lander_state.position;
    NED_coordinates current_velocity = lander_state.velocity;
    Rotation_Euler_coordinates current_attitude = lander_state.attitude;
    Rotation_Euler_coordinates current_body_rates = lander_state.body_rates;

    updatePIDOutputsLimits(); // 

    //--------------------------------------------------------------------------
    // Position -> Velocity
    //--------------------------------------------------------------------------

    float desiredNorthVelocity =
        PID_position.axis_x.compute(dt, current_position.North_SI);

    float desiredEastVelocity =
        PID_position.axis_y.compute(dt, current_position.East_SI);

    float desiredDownVelocity =
        PID_position.axis_z.compute(dt, current_position.Down_SI);

    setVelocitySetpoint({
        desiredNorthVelocity,
        desiredEastVelocity,
        desiredDownVelocity
    });

    //--------------------------------------------------------------------------
    // Velocity -> Pitch / Roll / Thrust
    //--------------------------------------------------------------------------

    float desiredPitch =
        PID_velocity.axis_x.compute(dt, current_velocity.North_SI);

    float desiredRoll =
        PID_velocity.axis_y.compute(dt, current_velocity.East_SI);

    float desiredThrust =
        PID_velocity.axis_z.compute(dt, current_velocity.Down_SI);

    setAttitudeSetpoint({
        .roll = desiredRoll,
        .pitch = desiredPitch,
        .yaw = getYawSetpoint()     // Keep yaw commanded by the mission
    });

    //--------------------------------------------------------------------------
    // Attitude -> Body Rates
    //--------------------------------------------------------------------------

    float desiredRollRate =
        PID_attitude.axis_x.compute(dt, current_attitude.Roll_SI);

    float desiredPitchRate =
        PID_attitude.axis_y.compute(dt, current_attitude.Pitch_SI);

    float desiredYawRate =
        PID_attitude.axis_z.compute(dt, current_attitude.Yaw_SI);

    setBodyRatesSetpoint({
        desiredRollRate,
        desiredPitchRate,
        desiredYawRate
    });

    //--------------------------------------------------------------------------
    // Body Rates -> Torques
    //--------------------------------------------------------------------------

    float rollTorque =
        PID_body_rates.axis_x.compute(dt, current_body_rates.Roll_SI);

    float pitchTorque =
        PID_body_rates.axis_y.compute(dt, current_body_rates.Pitch_SI);

    float yawTorque =
        PID_body_rates.axis_z.compute(dt, current_body_rates.Yaw_SI);

    //--------------------------------------------------------------------------
    // Final control command
    //--------------------------------------------------------------------------

    ControlCommand output = {
        .tau_roll = rollTorque,
        .tau_pitch = pitchTorque,
        .tau_yaw = yawTorque,
        .thrust = desiredThrust
    };

    setControlCmd(output);
}


void Controller::updatePIDOutputsLimits()
{
    //--------------------------------------------------------------------------
    // Position -> Velocity (m/s)
    //--------------------------------------------------------------------------

    PID_position.axis_x.setOutputLimits(-current_regime.max_horizontal_velocity_ms,current_regime.max_horizontal_velocity_ms);

    PID_position.axis_y.setOutputLimits(-current_regime.max_horizontal_velocity_ms,current_regime.max_horizontal_velocity_ms);

    PID_position.axis_z.setOutputLimits(-current_regime.max_vertical_velocity_ms, current_regime.max_vertical_velocity_ms);

    //--------------------------------------------------------------------------
    // Velocity -> Attitude / Thrust
    //--------------------------------------------------------------------------

    PID_velocity.axis_x.setOutputLimits(-current_regime.max_pitch_deg, current_regime.max_pitch_deg);

    PID_velocity.axis_y.setOutputLimits(-current_regime.max_roll_deg, current_regime.max_roll_deg);

    PID_velocity.axis_z.setOutputLimits(ActuatorsConfig::min_thrust, ActuatorsConfig::max_thrust);

    //--------------------------------------------------------------------------
    // Attitude -> Body Rates
    //--------------------------------------------------------------------------

    PID_attitude.axis_x.setOutputLimits(
        -current_regime.max_roll_rate_degs,
         current_regime.max_roll_rate_degs);

    PID_attitude.axis_y.setOutputLimits(
        -current_regime.max_pitch_rate_degs,
         current_regime.max_pitch_rate_degs);

    PID_attitude.axis_z.setOutputLimits(
        -current_regime.max_yaw_rate_degs,
         current_regime.max_yaw_rate_degs);

    //--------------------------------------------------------------------------
    // Body Rates -> Actuators
    //--------------------------------------------------------------------------

    PID_body_rates.axis_x.setOutputLimits(
        -current_regime.max_roll_command,
         current_regime.max_roll_command);

    PID_body_rates.axis_y.setOutputLimits(
        -current_regime.max_pitch_command,
         current_regime.max_pitch_command);

    PID_body_rates.axis_z.setOutputLimits(
        -current_regime.max_yaw_command,
         current_regime.max_yaw_command);
}

void Controller::resetIntegralForAllPID(){
    PID_position.axis_x.reset();
    PID_position.axis_y.reset();
    PID_position.axis_z.reset();

    PID_velocity.axis_x.reset();
    PID_velocity.axis_y.reset();
    PID_velocity.axis_z.reset();

    PID_attitude.axis_x.reset();
    PID_attitude.axis_y.reset();
    PID_attitude.axis_z.reset();

    PID_body_rates.axis_x.reset();
    PID_body_rates.axis_y.reset();
    PID_body_rates.axis_z.reset();

}


void Controller::setPositionSetpoint(const NED_coordinates& position_target){
   PID_position.axis_x.setTarget(position_target.North_SI);
   PID_position.axis_y.setTarget(position_target.East_SI);
   PID_position.axis_z.setTarget(position_target.Down_SI);
}


void Controller::setVelocitySetpoint(const NED_coordinates& velocity_target){
   PID_velocity.axis_x.setTarget(velocity_target.North_SI);
   PID_velocity.axis_y.setTarget(velocity_target.East_SI);
   PID_velocity.axis_z.setTarget(velocity_target.Down_SI);
}

void Controller::setAttitudeSetpoint(const Rotation_Euler_coordinates& attitude_target){
   PID_attitude.axis_x.setTarget(attitude_target.Roll_SI);
   PID_attitude.axis_y.setTarget(attitude_target.Pitch_SI);
   PID_attitude.axis_z.setTarget(attitude_target.Yaw_SI);
}

void Controller::setBodyRatesSetpoint(const Rotation_Euler_coordinates& body_rates_target){
   PID_body_rates.axis_x.setTarget(body_rates_target.Roll_SI);
   PID_body_rates.axis_y.setTarget(body_rates_target.Pitch_SI);
   PID_body_rates.axis_z.setTarget(body_rates_target.Yaw_SI);
}


