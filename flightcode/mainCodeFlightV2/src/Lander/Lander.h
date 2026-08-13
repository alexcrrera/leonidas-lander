#ifndef LANDER_H
#define LANDER_H

#include <Arduino.h>

#include "../Sensor/Sensor.h"
#include "../VN300/VN300.h"
#include "../Lidar/Lidar.h"
#include "../OpticalFlow/OpticalFlow.h"

#include "../Utilities/Utilities.h"

#include "../StateEstimator/StateEstimator.h"

#include "Lander_structs.h"

class Lander
{
public:
    Lander()=default;

    void begin();
    void update();

    const LanderSolution& getSolution() const;

    bool hasPositionSolution() const;
    bool hasVelocitySolution() const;
    bool hasAltitudeSolution() const;
    bool hasAttitudeSolution() const;

    bool hasError(LanderError error) const;
    uint16_t getErrors() const;

    
    VN300& getVN300() { return vn300; }
    Lidar& getLidar() { return lidar; }
    OpticalFlow& getOpticalFlow() { return opticalFlow; }

private:
    VN300 vn300;
    Lidar lidar;
    OpticalFlow opticalFlow;
    
    StateEstimator stateEstimator;
    LanderSolution solution;



    // First valid Register 58 GNSS position defines local NED origin.
    bool positionOriginInitialized = false;
    double originLatitude = 0.0;
    double originLongitude = 0.0;
    double originAltitude = 0.0;

    void clearSolutionStatus();

    void estimateAttitude();
    void estimateAcceleration();
    void estimateVelocity();
    void estimatePosition();
    void estimateAltitude();

    bool gnssSolutionUsable(
        const VN300Measurement& measurement
    ) const;

    void initializePositionOrigin(
        const VN300Measurement& measurement
    );

    void convertLlaToLocalNed(
        const VN300Measurement& measurement,
        NED_coordinates& position
    ) const;

    void updateErrors();

    void setError(LanderError error);
};

#endif
