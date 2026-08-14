#include "StateEstimator.h"




void StateEstimator::begin()
{

    //Serial.println("STATE ESTIMATOR: BEGIN");
    
}


void StateEstimator::update(LanderState& state, SensorMeasurements& sensorMeasurements)
{
    estimateAttitude_EULER(state, sensorMeasurements);
    estimateAcceleration_EULER(state, sensorMeasurements);
    estimateVelocity_EULER(state, sensorMeasurements);

    estimateAcceleration_NED(state, sensorMeasurements);
    estimateVelocity_NED(state, sensorMeasurements);
    estimatePosition_NED(state, sensorMeasurements);
}



void StateEstimator::estimateAttitude_EULER(LanderState& state, const SensorMeasurements& sensorMeasurements)
{
   float Roll_SI = sensorMeasurements.vn300_data.yprLinearAccelGyro.Roll;
   float Pitch_SI = sensorMeasurements.vn300_data.yprLinearAccelGyro.Pitch;
   float Yaw_SI = sensorMeasurements.vn300_data.yprLinearAccelGyro.Yaw;


   state.attitude.Roll_SI = Roll_SI;    
   state.attitude.Pitch_SI = Pitch_SI;
   state.attitude.Yaw_SI = Yaw_SI;

}

void StateEstimator::estimateAcceleration_NED(LanderState& state, const SensorMeasurements& sensorMeasurements)
{
    float LinAccelN = sensorMeasurements.vn300_data.yprLinearAccelGyro.LinAccelN;
    float LinAccelE = sensorMeasurements.vn300_data.yprLinearAccelGyro.LinAccelE;
    float LinAccelD = sensorMeasurements.vn300_data.yprLinearAccelGyro.LinAccelD;
    state.acceleration.North_SI = LinAccelN;
    state.acceleration.East_SI = LinAccelE;
    state.acceleration.Down_SI = LinAccelD;

}


void StateEstimator::estimateVelocity_NED(LanderState& state, const SensorMeasurements& sensorMeasurements)
{
    
    float VelN = sensorMeasurements.vn300_data.insSolution.VelN;
    float VelE = sensorMeasurements.vn300_data.insSolution.VelE;
    float VelD = sensorMeasurements.vn300_data.insSolution.VelD;

    
    state.velocity.North_SI = VelN;
    state.velocity.East_SI = VelE;
    state.velocity.Down_SI = VelD;
}
