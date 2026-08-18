#include "Lander.h"

#include <math.h>
#include "../FlightManager/FlightManager.h"

void Lander::begin(FlightManager* flightManager)
{
    vn300.begin();
    opticalFlow.begin();
    lidar.begin();
    stateEstimator.begin(flightManager);
}

void Lander::update()
{
    

    vn300.update();
    opticalFlow.update(getState());
    lidar.update(getState());

    SensorData.vn300_data = vn300.getMeasurement();
    SensorData.opticalFlow_data = opticalFlow.getMeasurement();
    SensorData.lidar_data = lidar.getMeasurement();

    stateEstimator.update(SensorData);


        

    // print position here:
    // Serial.print("Lander Position: ");
    // Serial.print(state.position.North_SI, 6);
    // Serial.print(", ");
    // Serial.print(state.position.East_SI, 6);
    // Serial.print(", ");
    // Serial.println(state.position.Down_SI, 6);

   
    
}

