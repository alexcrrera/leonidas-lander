#include "Lander.h"

#include <math.h>

void Lander::begin()

{
    vn300.begin();
    opticalFlow.begin();
    lidar.begin();
    stateEstimator.begin(*this);
}

void Lander::update()
{
    

    vn300.update();
    opticalFlow.update();
    lidar.update();

    SensorData.vn300_data = vn300.getMeasurement();
    SensorData.opticalFlow_data = opticalFlow.getMeasurement();
    SensorData.lidar_data = lidar.getMeasurement();

    stateEstimator.update(SensorData);

   
    
}

