#include "../Mission/Mission.h"
#include <cmath>
#include "../Config/FlightRegimeConfig.h"
#include "../Config/CoordinateConfig.h"
#include "../Config/ControllerConfig.h"
#include "Controller.h"
#include "../FlightManager/FlightManager.h"
#include "../Config/SystemConfig.h"


Controller::Controller()
    :
    PID_position{
        PID(ControllerPID::PID_gains_position_North, ControllerPID::position_NED_update_frequency),
        PID(ControllerPID::PID_gains_position_East, ControllerPID::position_NED_update_frequency),
        PID(ControllerPID::PID_gains_position_Down, ControllerPID::position_NED_update_frequency)
    },

    PID_velocity{
        PID(ControllerPID::PID_gains_velocity_North, ControllerPID::velocity_NED_update_frequency),
        PID(ControllerPID::PID_gains_velocity_East, ControllerPID::velocity_NED_update_frequency),
        PID(ControllerPID::PID_gains_velocity_Down, ControllerPID::velocity_NED_update_frequency)
    },

    PID_attitude{
        PID(ControllerPID::PID_gains_attitude_roll, ControllerPID::attitude_update_frequency),
        PID(ControllerPID::PID_gains_attitude_pitch, ControllerPID::attitude_update_frequency),
        PID(ControllerPID::PID_gains_attitude_yaw, ControllerPID::attitude_update_frequency)
    },

    PID_body_rates{
        PID(ControllerPID::PID_gains_body_rates_roll, ControllerPID::body_rates_update_frequency),
        PID(ControllerPID::PID_gains_body_rates_pitch, ControllerPID::body_rates_update_frequency),
        PID(ControllerPID::PID_gains_body_rates_yaw, ControllerPID::body_rates_update_frequency)
    }
{
}




void Controller::update()
{

    
    if(flight_manager == nullptr){
        //Serial.println("Controller: FlightManager pointer is null. Cannot update controller.");
        return;
    }
    const auto& landerState = flight_manager->getLander().getState();

    NED_coordinates currentPosition = landerState.position;
  //  Serial.print("Current Position: ");
    // print current position and current velocity:
  ///  Serial.print(currentPosition.North_SI, 6);
  //  Serial.print("Current Position from lander: ");
    // print current position and current velocity:
    //Serial.print(flight_manager, 6);
    NED_coordinates currentVelocity = landerState.velocity;
    NED_coordinates currentAcceleration = landerState.acceleration;

    Rotation_Euler_coordinates currentAttitude = landerState.attitude;
    Rotation_Euler_coordinates currentAngularVelocity = landerState.angularVelocity;

    updatePIDOutputsLimits();

    auto mission_target = flight_manager->getMission().getTarget();
    float yaw_target_deg = mission_target.target.yaw_deg; // 

    setPositionSetpoint(mission_target.target.positionNED);

    // Position -> Velocity
    float desiredNorthVelocity = PID_position.axis_x.update(currentPosition.North_SI, currentVelocity.North_SI);
    float desiredEastVelocity = PID_position.axis_y.update(currentPosition.East_SI, currentVelocity.East_SI);


   
   
   // Z PID DEBUG


float desiredDownVelocity = PID_position.axis_z.update(
    currentPosition.Down_SI,
    currentVelocity.Down_SI
);



    setVelocitySetpoint({desiredNorthVelocity, desiredEastVelocity, desiredDownVelocity});

    // Velocity -> Pitch / Roll / Thrust
    float desiredPitch = PID_velocity.axis_x.update(currentVelocity.North_SI, currentAcceleration.North_SI);
    float desiredRoll = PID_velocity.axis_y.update(currentVelocity.East_SI, currentAcceleration.East_SI);
    float desiredDownAcceleration    = PID_velocity.axis_z.update(currentVelocity.Down_SI, currentAcceleration.Down_SI);

    Rotation_Euler_coordinates attitudeTarget{
        .Roll_SI = desiredRoll,
        .Pitch_SI = desiredPitch,
        .Yaw_SI = yaw_target_deg
    };

    setAttitudeSetpoint(attitudeTarget);

    // Attitude -> Body Rates
    float desiredRollRate = PID_attitude.axis_x.update(currentAttitude.Roll_SI, currentAngularVelocity.Roll_SI);
    float desiredPitchRate = PID_attitude.axis_y.update(currentAttitude.Pitch_SI, currentAngularVelocity.Pitch_SI);
    float desiredYawRate = PID_attitude.axis_z.update(currentAttitude.Yaw_SI, currentAngularVelocity.Yaw_SI);

    setBodyRatesSetpoint({desiredRollRate, desiredPitchRate, desiredYawRate});

    // Body Rates -> Torques
    float rollTorque = PID_body_rates.axis_x.update(currentAngularVelocity.Roll_SI);
    float pitchTorque = PID_body_rates.axis_y.update(currentAngularVelocity.Pitch_SI);
    float yawTorque = PID_body_rates.axis_z.update(currentAngularVelocity.Yaw_SI);

    float desiredThrust = desiredDownAccelerationToThrust(desiredDownAcceleration);
    desiredThrust = constrain(desiredThrust, ActuatorsConfig::THRUST_EDF_min, ActuatorsConfig::THRUST_EDF_max);
    


    ControlCommand output{
        .tau_yaw = yawTorque,
        .tau_pitch = pitchTorque,
        .tau_roll = rollTorque,
        .thrust_N = desiredThrust
    };

    setControlCmd(output);
}


float Controller::desiredDownAccelerationToThrust(float accel_Down_SI){

    return(SystemConfig::lander_mass*(Utilities::GRAVITY_MS2-accel_Down_SI));

}


void Controller::updatePIDOutputsLimits()


{
    const auto currentRegime =
        flight_manager->getStateMachine().getCurrentFlightRegimeData();

    // Position [m] -> Velocity [m/s]
    PID_position.axis_x.setOutputLimits(
        -currentRegime.max_horizontal_velocity_ms,
         currentRegime.max_horizontal_velocity_ms);

    PID_position.axis_y.setOutputLimits(
        -currentRegime.max_horizontal_velocity_ms,
         currentRegime.max_horizontal_velocity_ms);

    PID_position.axis_z.setOutputLimits(
        -currentRegime.max_vertical_velocity_ms,
         currentRegime.max_vertical_velocity_ms);

    // Velocity [m/s] -> Attitude [deg] / Acceleration [m/s²]
    PID_velocity.axis_x.setOutputLimits(
        -currentRegime.max_pitch_roll_velocity_degs,
         currentRegime.max_pitch_roll_velocity_degs);

    PID_velocity.axis_y.setOutputLimits(
        -currentRegime.max_pitch_roll_velocity_degs,
         currentRegime.max_pitch_roll_velocity_degs);

    PID_velocity.axis_z.setOutputLimits(
        -currentRegime.max_vertical_acceleration_ms2,
         currentRegime.max_vertical_acceleration_ms2);


   

    // Attitude [deg] -> Body Rates [deg/s]
    PID_attitude.axis_x.setOutputLimits(
        -currentRegime.max_pitch_roll_velocity_degs,
         currentRegime.max_pitch_roll_velocity_degs);

    PID_attitude.axis_y.setOutputLimits(
        -currentRegime.max_pitch_roll_velocity_degs,
         currentRegime.max_pitch_roll_velocity_degs);

    PID_attitude.axis_z.setOutputLimits(
        -currentRegime.max_yaw_velocity_degs,
         currentRegime.max_yaw_velocity_degs);

    // Body Rates [deg/s] -> Angular Acceleration [deg/s²]
    PID_body_rates.axis_x.setOutputLimits(
        -currentRegime.max_pitch_roll_acceleration_degs2,
         currentRegime.max_pitch_roll_acceleration_degs2);

    PID_body_rates.axis_y.setOutputLimits(
        -currentRegime.max_pitch_roll_acceleration_degs2,
         currentRegime.max_pitch_roll_acceleration_degs2);

    PID_body_rates.axis_z.setOutputLimits(
        -currentRegime.max_yaw_acceleration_degs2,
         currentRegime.max_yaw_acceleration_degs2);
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




void Controller::updatePIDIntegralLimits(){

     const auto currentRegime = flight_manager->getStateMachine().getCurrentFlightRegimeData();


     float output_limit_PID_vx = PID_velocity.axis_x.getPID_limit_integral_min();
     float output_limit_PID_vy = PID_velocity.axis_y.getPID_limit_integral_min();
     float output_limit_PID_vz = PID_velocity.axis_z.getPID_limit_integral_min();

     PID_velocity.axis_x.setIntegralLimits(-output_limit_PID_vx, output_limit_PID_vx); // Set integral limits for PID_velocity axis_x
     PID_velocity.axis_y.setIntegralLimits(-output_limit_PID_vy, output_limit_PID_vy);
     PID_velocity.axis_z.setIntegralLimits(-output_limit_PID_vz, output_limit_PID_vz);
}