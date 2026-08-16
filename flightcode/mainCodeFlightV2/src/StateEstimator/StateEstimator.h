#pragma once

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H


#include <Arduino.h>

#include "../Utilities/Utilities.h"
#include "../VN300/VN300.h"
#include "../Lidar/Lidar.h"
#include "../OpticalFlow/OpticalFlow.h"
#include "../Lander/Lander_structs.h"

class Lander;

struct SensorMeasurements
{ 
    VN300Measurement vn300_data;
    LidarMeasurement lidar_data;
    OpticalFlowMeasurement opticalFlow_data;
};




class StateEstimator
{

    public:
        StateEstimator() = default;
        
        void begin(Lander& lander_);
        void update(SensorMeasurements& sensorMeasurements);
        LanderState& getState() { return state; }
        

    private:

        Lander*  lander = nullptr;
        LanderState state;
        SensorMeasurements sensorMeasurements;
        
        void estimateAttitude_EULER(LanderState& state, const SensorMeasurements& sensorMeasurements);
       
        void estimateVelocity_EULER(LanderState& state, const SensorMeasurements& sensorMeasurements    );


        void estimateAcceleration_NED(LanderState& state, const SensorMeasurements& sensorMeasurements);
        void estimateVelocity_NED(LanderState& state, const SensorMeasurements& sensorMeasurements);
        void estimatePosition_NED(LanderState& state, const SensorMeasurements& sensorMeasurements);
};

#endif // STATE_ESTIMATOR_H