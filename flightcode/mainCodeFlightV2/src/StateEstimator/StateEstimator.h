#pragma once

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H


#include <Arduino.h>

#include "../Utilities/Utilities.h"
#include "../VN300/VN300.h"
#include "../Lidar/Lidar.h"
#include "../OpticalFlow/OpticalFlow.h"
#include "State_structs.h"
#include "../Lander/Lander.h"

struct SensorMeasurements
{ 
    VN300Measurement vn300_data;
    LidarMeasurement lidar_data;
    OpticalFlowMeasurement opticalFlow_data;
};




class StateEstimator
{

    public:
        StateEstimator(Lander& lander) : lander_(lander) {}
        void begin();
        void update();

    private:

        Lander& lander_;
        LanderState state;
        SensorMeasurements sensorMeasurements;

        void estimateAttitude_EULER();
        void estimateAcceleration_EULER();
        void estimateVelocity_EULER();


        void estimateAcceleration_NED();
        void estimateVelocity_NED();
        void estimatePosition_NED();
};

#endif // STATE_ESTIMATOR_H