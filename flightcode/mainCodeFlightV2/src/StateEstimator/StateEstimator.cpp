#include "StateEstimator.h"


#include "StateEstimator.h"
#include "../Lander/Lander.h"




void StateEstimator::begin(Lander& lander_)
{
    lander = &lander_; // pass lander object to state estimator
    //Serial.println("STATE ESTIMATOR: BEGIN");
    
}

void StateEstimator::setHomePosition(const NED_coordinates& home_position)
{
    home_position_NED = home_position;
    home_position_set = true;
}

bool StateEstimator::isFlightConditionValid(){
    return true; // placeholder for actual flight condition validation logic
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
    
    float VelN = sensorMeasurements.opticalFlow_data.velocity_North_SI;
    float VelE = sensorMeasurements.opticalFlow_data.velocity_East_SI;
   


    
    state.velocity.North_SI = VelN;
    state.velocity.East_SI = VelE;
    state.velocity.Down_SI = sensorMeasurements.lidar_data.velocity_Down_SI;

     state.validity.velocityValid = sensorMeasurements.opticalFlow_data.velocityValid; // set velocity validity based on optical flow data
}



void StateEstimator::estimatePosition_NED(LanderState& state, const SensorMeasurements& sensorMeasurements)
{
    // Convert latitude, longitude, and altitude to NED coordinates

    state.position.North_SI = 0.0;
    state.position.East_SI = 0.0;
    state.position.Down_SI = -sensorMeasurements.lidar_data.altitude_M; // Use Lidar altitude for Down coordinate
    state.altitude_M = sensorMeasurements.lidar_data.altitude_M; // Store altitude in the state
    state.validity.positionValid = true; // Assuming position is valid if Lidar data is available
    state.validity.altitudeValid = true; // Assuming altitude is valid if Lidar data is available
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