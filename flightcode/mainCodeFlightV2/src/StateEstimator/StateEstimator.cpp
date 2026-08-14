#include "StateEstimator.h"


#include "StateEstimator.h"
#include "../Lander/Lander.h"




void StateEstimator::begin(Lander& lander_)
{
    lander = &lander_; // pass lander object to state estimator
    //Serial.println("STATE ESTIMATOR: BEGIN");
    
}


void StateEstimator::update(SensorMeasurements& sensorMeasurements)
{
    estimateAttitude_EULER(state, sensorMeasurements);
    
    estimateVelocity_EULER(state, sensorMeasurements);

    estimateAcceleration_NED(state, sensorMeasurements);
    estimateVelocity_NED(state, sensorMeasurements);
    estimatePosition_NED(state, sensorMeasurements);
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



void StateEstimator::estimatePosition_NED(LanderState& state, const SensorMeasurements& sensorMeasurements)
{
    // Convert latitude, longitude, and altitude to NED coordinates
    double lat = sensorMeasurements.vn300_data.insSolution.PosLat;
    double lon = sensorMeasurements.vn300_data.insSolution.PosLon;
    double alt = sensorMeasurements.vn300_data.insSolution.PosAlt;

    // Assuming a simple conversion for demonstration purposes
    // In practice, you would use a proper geodetic to NED conversion
    state.position.North_SI = static_cast<float>(lat);
    state.position.East_SI = static_cast<float>(lon);
    state.position.Down_SI = static_cast<float>(alt);
}


void StateEstimator::estimateVelocity_EULER(LanderState& state, const SensorMeasurements& sensorMeasurements)
{
    // use gyroscope data to estimate angular velocity in Euler angles
    float GyroX = sensorMeasurements.vn300_data.yprLinearAccelGyro.GyroX;
    float GyroY = sensorMeasurements.vn300_data.yprLinearAccelGyro.GyroY;
    float GyroZ = sensorMeasurements.vn300_data.yprLinearAccelGyro.GyroZ;   
    state.angularVelocity.Roll_SI = GyroX;
    state.angularVelocity.Pitch_SI = GyroY;
    state.angularVelocity.Yaw_SI = GyroZ;
}


void StateEstimator::estimateAttitude_EULER(LanderState& state, const SensorMeasurements& sensorMeasurements)
{
    // use VN300 data to estimate attitude in Euler angles
    float Yaw = sensorMeasurements.vn300_data.yprLinearAccelGyro.Yaw;
    float Pitch = sensorMeasurements.vn300_data.yprLinearAccelGyro.Pitch;
    float Roll = sensorMeasurements.vn300_data.yprLinearAccelGyro.Roll;
    state.attitude.Roll_SI = Roll;
    state.attitude.Pitch_SI = Pitch;
    state.attitude.Yaw_SI = Yaw;
}