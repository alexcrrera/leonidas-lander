#include "../Mission/Mission.h"
#include <cmath>
#include "../Config/FlightRegimeConfig.h"
#include "../Config/CoordinateConfig.h"
#include "../Config/ControllerConfig.h"
#include "Controller.h"




Controller::Controller()
    :
    PID_position{
        PID(ControllerPID::PID_gains_position_North),
        PID(ControllerPID::PID_gains_position_East),
        PID(ControllerPID::PID_gains_position_Down)
    },

    PID_velocity{
        PID(ControllerPID::PID_gains_velocity_North),
        PID(ControllerPID::PID_gains_velocity_East),
        PID(ControllerPID::PID_gains_velocity_Down)
    },

    PID_attitude{
        PID(ControllerPID::PID_gains_attitude_roll),
        PID(ControllerPID::PID_gains_attitude_pitch),
        PID(ControllerPID::PID_gains_attitude_yaw)
    },

    PID_body_rates{
        PID(ControllerPID::PID_gains_body_rates_roll),
        PID(ControllerPID::PID_gains_body_rates_pitch),
        PID(ControllerPID::PID_gains_body_rates_yaw)
    }
{
}


void Controller::update(
    float dt,
    const LanderState& lander_state,
    FlightRegimeData current_regime
)
{
    if (!active) {
        return;
    }

    NED_coordinates current_position = lander_state.position;
    NED_coordinates current_velocity = lander_state.velocity;

    Rotation_Euler_coordinates current_attitude =
        lander_state.attitude;

    Rotation_Euler_coordinates current_body_rates =
        lander_state.angularVelocity;


    updatePIDOutputsLimits(current_regime);


    //--------------------------------------------------------------------------
    // Position -> Velocity
    //--------------------------------------------------------------------------

    float desiredNorthVelocity =
        PID_position.axis_x.compute(
            dt,
            current_position.North_SI
        );

    float desiredEastVelocity =
        PID_position.axis_y.compute(
            dt,
            current_position.East_SI
        );

    float desiredDownVelocity =
        PID_position.axis_z.compute(
            dt,
            current_position.Down_SI
        );


    setVelocitySetpoint({
        desiredNorthVelocity,
        desiredEastVelocity,
        desiredDownVelocity
    });


    //--------------------------------------------------------------------------
    // Velocity -> Pitch / Roll / Thrust
    //--------------------------------------------------------------------------

    float desiredPitch =
        PID_velocity.axis_x.compute(
            dt,
            current_velocity.North_SI
        );

    float desiredRoll =
        PID_velocity.axis_y.compute(
            dt,
            current_velocity.East_SI
        );

    float desiredThrust =
        PID_velocity.axis_z.compute(
            dt,
            current_velocity.Down_SI
        );


    Rotation_Euler_coordinates attitudeTarget{
        .Roll_SI = desiredRoll,
        .Pitch_SI = desiredPitch,
        .Yaw_SI = 0.0f
    };

    setAttitudeSetpoint(attitudeTarget);


    //--------------------------------------------------------------------------
    // Attitude -> Body Rates
    //--------------------------------------------------------------------------

    float desiredRollRate =
        PID_attitude.axis_x.compute(
            dt,
            current_attitude.Roll_SI
        );

    float desiredPitchRate =
        PID_attitude.axis_y.compute(
            dt,
            current_attitude.Pitch_SI
        );

    float desiredYawRate =
        PID_attitude.axis_z.compute(
            dt,
            current_attitude.Yaw_SI
        );


    setBodyRatesSetpoint({
        desiredRollRate,
        desiredPitchRate,
        desiredYawRate
    });


    //--------------------------------------------------------------------------
    // Body Rates -> Torques
    //--------------------------------------------------------------------------

    float rollTorque =
        PID_body_rates.axis_x.compute(
            dt,
            current_body_rates.Roll_SI
        );

    float pitchTorque =
        PID_body_rates.axis_y.compute(
            dt,
            current_body_rates.Pitch_SI
        );

    float yawTorque =
        PID_body_rates.axis_z.compute(
            dt,
            current_body_rates.Yaw_SI
        );


    //--------------------------------------------------------------------------
    // Final control command
    //--------------------------------------------------------------------------

    ControlCommand output{
        .tau_yaw = yawTorque,
        
        .tau_pitch = pitchTorque,
        .tau_roll = rollTorque,
        
        .thrust = desiredThrust
    };

    setControlCmd(output);
}


void Controller::updatePIDOutputsLimits(
    const FlightRegimeData& current_regime
)
{
    //--------------------------------------------------------------------------
    // Position -> Velocity
    //--------------------------------------------------------------------------

    PID_position.axis_x.setOutputLimits(
        -current_regime.max_horizontal_velocity_ms,
         current_regime.max_horizontal_velocity_ms
    );

    PID_position.axis_y.setOutputLimits(
        -current_regime.max_horizontal_velocity_ms,
         current_regime.max_horizontal_velocity_ms
    );

    PID_position.axis_z.setOutputLimits(
        -current_regime.max_vertical_velocity_ms,
         current_regime.max_vertical_velocity_ms
    );


    //--------------------------------------------------------------------------
    // Velocity -> Attitude / Thrust
    //--------------------------------------------------------------------------

    // These PIDs produce the commanded acceleration/attitude response.
    // The current FlightRegimeData does not contain pitch/roll angle limits.

    PID_velocity.axis_x.setOutputLimits(
        -current_regime.max_horizontal_acceleration_ms2,
         current_regime.max_horizontal_acceleration_ms2
    );

    PID_velocity.axis_y.setOutputLimits(
        -current_regime.max_horizontal_acceleration_ms2,
         current_regime.max_horizontal_acceleration_ms2
    );

    PID_velocity.axis_z.setOutputLimits(
        -current_regime.max_vertical_acceleration_ms2,
         current_regime.max_vertical_acceleration_ms2
    );


    //--------------------------------------------------------------------------
    // Attitude -> Body Rates
    //--------------------------------------------------------------------------

    PID_attitude.axis_x.setOutputLimits(
        -current_regime.max_pitch_roll_velocity_degs,
         current_regime.max_pitch_roll_velocity_degs
    );

    PID_attitude.axis_y.setOutputLimits(
        -current_regime.max_pitch_roll_velocity_degs,
         current_regime.max_pitch_roll_velocity_degs
    );

    PID_attitude.axis_z.setOutputLimits(
        -current_regime.max_yaw_velocity_degs,
         current_regime.max_yaw_velocity_degs
    );


    //--------------------------------------------------------------------------
    // Body Rates -> Actuators
    //--------------------------------------------------------------------------

    PID_body_rates.axis_x.setOutputLimits(
        -current_regime.max_pitch_roll_acceleration_degs2,
         current_regime.max_pitch_roll_acceleration_degs2
    );

    PID_body_rates.axis_y.setOutputLimits(
        -current_regime.max_pitch_roll_acceleration_degs2,
         current_regime.max_pitch_roll_acceleration_degs2
    );

    PID_body_rates.axis_z.setOutputLimits(
        -current_regime.max_yaw_acceleration_degs2,
         current_regime.max_yaw_acceleration_degs2
    );
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


