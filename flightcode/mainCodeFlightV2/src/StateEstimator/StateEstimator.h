#pragma once

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H


#include <Arduino.h>

#include "../Utilities/Utilities.h"
#include "../VN300/VN300.h"
#include "../Lidar/Lidar.h"
#include "../OpticalFlow/OpticalFlow.h"
#include "../Lander/Lander_structs.h"

class FlightManager; // forward declaration to avoid circular dependency

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
        
        void begin(FlightManager* flightManager_);
        void update(SensorMeasurements& sensorMeasurements);
        LanderState& getState() { return state; }
        
        bool isFlightConditionValid();

        void request_setHomePosition();

        bool isHomePositionSet() const { return home_position_set; }
    private:



        FlightManager* flight_manager = nullptr;
        LanderState state;
        SensorMeasurements sensorMeasurements;
        
        void estimateAttitude_EULER(LanderState& state, const SensorMeasurements& sensorMeasurements);
       
        void estimateVelocity_EULER(LanderState& state, const SensorMeasurements& sensorMeasurements    );


        void estimateAcceleration_NED(LanderState& state, const SensorMeasurements& sensorMeasurements);
        void estimateVelocity_NED(LanderState& state, const SensorMeasurements& sensorMeasurements);
        void estimatePosition_NED(LanderState& state, const SensorMeasurements& sensorMeasurements);

        bool home_position_set = false;
        NED_coordinates home_position_NED;
    };

#endif // STATE_ESTIMATOR_H