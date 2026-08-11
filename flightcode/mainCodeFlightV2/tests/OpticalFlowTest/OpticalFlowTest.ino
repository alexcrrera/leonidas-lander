#include "OpticalFlow.h"
OpticalFlow opticalFlow(
    Serial8,
    19200
);


void printFaults(
    const OpticalFlow& sensor
)
{
    Serial.print("Status: 0x");
    Serial.println(sensor.status(), HEX);

    if (sensor.hasFault(SensorFault::NotInitialized)) {
        Serial.println("  NotInitialized");
    }

    if (sensor.hasFault(SensorFault::NoData)) {
        Serial.println("  NoData");
    }

    if (sensor.hasFault(SensorFault::Timeout)) {
        Serial.println("  Timeout");
    }

    if (sensor.hasFault(SensorFault::OutOfRange)) {
        Serial.println("  OutOfRange");
    }

    if (sensor.hasFault(SensorFault::FastDelta)) {
        Serial.println("  FastDelta");
    }

    if (sensor.hasFault(SensorFault::InvalidData)) {
        Serial.println("  InvalidData");
    }

    if (sensor.hasFault(SensorFault::CommunicationError)) {
        Serial.println("  CommunicationError");
    }

    if (sensor.hasFault(SensorFault::SensorError)) {
        Serial.println("  SensorError");
    }
}


void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("=================================");
    Serial.println("Optical Flow Sensor Test");
    Serial.println("Serial8 @ 19200 baud");
    Serial.println("=================================");


    const bool initialized =
        opticalFlow.begin();


    Serial.print("Begin: ");
    Serial.println(initialized ? "OK" : "FAIL");


    Serial.println();
}


void loop()
{
    opticalFlow.update();


    static unsigned long lastPrintTime = 0;

    const unsigned long currentTime = millis();


    if ((currentTime - lastPrintTime) < 100) {
        return;
    }


    lastPrintTime = currentTime;


    const OpticalFlowMeasurement measurement =
        opticalFlow.getMeasurement();


    Serial.println("---------------------------------");

    Serial.print("Healthy: ");
    Serial.println(opticalFlow.isHealthy());


    Serial.print("Has valid data: ");
    Serial.println(opticalFlow.hasValidData());


    Serial.print("Continuity: ");
    Serial.println(opticalFlow.hasContinuity());


    Serial.print("Delta X: ");
    Serial.println(measurement.deltaX);


    Serial.print("Delta Y: ");
    Serial.println(measurement.deltaY);


    Serial.print("Surface quality: ");
    Serial.println(measurement.surfaceQuality);


    Serial.print("Flow X: ");
    Serial.print(measurement.flowRateX, 5);
    Serial.println(" rad/s");


    Serial.print("Flow Y: ");
    Serial.print(measurement.flowRateY, 5);
    Serial.println(" rad/s");


    Serial.print("dt: ");
    Serial.print(measurement.dt, 5);
    Serial.println(" s");


    Serial.print("Last packet: ");
    Serial.print(
        currentTime - opticalFlow.getLastPacketTime()
    );
    Serial.println(" ms ago");


    Serial.print("Last valid measurement: ");
    Serial.print(
        currentTime -
        opticalFlow.getLastValidMeasurementTime()
    );
    Serial.println(" ms ago");


    printFaults(opticalFlow);
}