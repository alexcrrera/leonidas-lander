#ifndef LANDER_H
#define LANDER_H

#include <Arduino.h>

#include "../VN300/VN300.h"
#include "../Lidar/Lidar.h"
#include "../OpticalFlow/OpticalFlow.h"

#include "../Utilities/Utilities.h"

#include "../StateEstimator/StateEstimator.h"

#include "Lander_structs.h"

// ============================================================
// Lander owns the sensors and the state estimator &
// exposes the lander state to the flight manager
// ============================================================

class Lander
{
public:
    Lander()=default;

    void begin();
    void update();


    //  getters for the lander state and sensors
    VN300& getVN300() { return vn300; }
    Lidar& getLidar() { return lidar; }
    OpticalFlow& getOpticalFlow() { return opticalFlow; }

    LanderState& getState() { return stateEstimator.getState(); }
    StateEstimator& getStateEstimator() { return stateEstimator; }

private:

    VN300 vn300;
    Lidar lidar;
    OpticalFlow opticalFlow;

    

    SensorMeasurements SensorData;
    
    StateEstimator stateEstimator;
    


};

#endif
