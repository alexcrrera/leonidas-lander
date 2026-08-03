#include "VN300.h"

#include <math.h>


VN300::VN300(HardwareSerial& serialPort)
    : vectornav(serialPort),
      commaParser(",")
{
}


bool VN300::begin()
{
    Serial.println("INITIALIZING VECTORNAV");

    vectornav.begin(115200);

    delay(200);


    // ---------------------------------------------------------
    // Select asynchronous output register
    // ---------------------------------------------------------

    // Register 1:
    // Yaw / Pitch / Roll
    //
    // Output:
    // $VNYPR,yaw,pitch,roll
    //
    // vectornav.println("$VNWRG,6,1*XX");


    // Register 240:
    // Yaw / Pitch / Roll
    // Linear acceleration NED
    // Compensated angular rates
    //
    // Output:
    // $VNYIA,
    // yaw,pitch,roll,
    // linAccelN,linAccelE,linAccelD,
    // gyroX,gyroY,gyroZ
    //
    vectornav.println("$VNWRG,6,17*XX");


    // Asynchronous output frequency
    vectornav.println("$VNWRG,7,300*XX");


    dataIndex = 0;

    incomingDataString = "";

    offset = {};

    filteredMeasurement = {};

    filterInitialized = false;


    initializeSensorState(vn300TimeoutMs);

    return true;
}


void VN300::update()
{
    if (millis() - lastGnssPollMs >= gnssPollPeriodMs)
    {
        lastGnssPollMs = millis();

        pollGnssSolution();
    }


    while (vectornav.available() > 0)
    {
        char incomingChar = vectornav.read();


        if (incomingChar == '\n')
        {
            incomingData[dataIndex] = '\0';

            processVectornav();

            dataIndex = 0;

            incomingDataString = "";
        }
        else if (incomingChar != '\r')
        {
            incomingData[dataIndex] = incomingChar;

            incomingDataString += incomingChar;

            dataIndex++;

            checkOverflowVectornav();
        }
    }


    checkTimeout();
}

void VN300::processVectornav()



{
    // print temporary debug output to serial monitor
   // Serial.println(incomingDataString);


    int vectornavIdentity = checkHeaderVectornav();

    char headerVectornav[10];


    switch (vectornavIdentity)
    {
        // =====================================================
        // REGISTER 1
        //
        // Yaw / Pitch / Roll
        // =====================================================

        case 1:
        {
            VN300Measurement rawMeasurement;


            commaParser.parseLine(
                incomingData,
                headerVectornav,

                rawMeasurement.yaw,
                rawMeasurement.pitch,
                rawMeasurement.roll
            );


            registerPacket();

            clearFaults();


            validateMeasurement(rawMeasurement);


            if (isHealthy())
            {
                storeRawMeasurement(rawMeasurement);


                // VN300 attitude is already internally filtered.
                // Do not add additional EWA latency here.

                filteredMeasurement.yaw =
                    rawMeasurement.yaw;

                filteredMeasurement.pitch =
                    rawMeasurement.pitch;

                filteredMeasurement.roll =
                    rawMeasurement.roll;


                // Register 1 contains no acceleration or gyro.
                // Leave the other values at their existing values.
            }


            break;
        }

        case 58:
        {
            parseGnssSolution();

            break;
        }


        // =====================================================
        // REGISTER 240
        //
        // YPR
        // Linear Acceleration NED
        // Gyro XYZ
        // =====================================================

        case 240:
        {
            VN300Measurement rawMeasurement;


            commaParser.parseLine(
                incomingData,
                headerVectornav,

                rawMeasurement.yaw,
                rawMeasurement.pitch,
                rawMeasurement.roll,

                rawMeasurement.accelNorth,
                rawMeasurement.accelEast,
                rawMeasurement.accelDown,

                rawMeasurement.gyroX,
                rawMeasurement.gyroY,
                rawMeasurement.gyroZ
            );


            registerPacket();

            clearFaults();


            validateMeasurement(rawMeasurement);


            if (isHealthy())
            {
                storeRawMeasurement(rawMeasurement);

                applyFilter(rawMeasurement);
            }


            break;
        }


        default:
        {
            break;
        }
    }
}


void VN300::applyFilter(
    const VN300Measurement& rawMeasurement
)
{
    // ---------------------------------------------------------
    // Attitude
    //
    // No additional EWA filtering.
    // ---------------------------------------------------------

    filteredMeasurement.yaw =
        rawMeasurement.yaw;

    filteredMeasurement.pitch =
        rawMeasurement.pitch;

    filteredMeasurement.roll =
        rawMeasurement.roll;


    // ---------------------------------------------------------
    // First valid sample
    //
    // Initialize directly from measurement to avoid filtering
    // from zero during startup.
    // ---------------------------------------------------------

    if (!filterInitialized)
    {
        filteredMeasurement.accelNorth =
            rawMeasurement.accelNorth;

        filteredMeasurement.accelEast =
            rawMeasurement.accelEast;

        filteredMeasurement.accelDown =
            rawMeasurement.accelDown;


        filteredMeasurement.gyroX =
            rawMeasurement.gyroX;

        filteredMeasurement.gyroY =
            rawMeasurement.gyroY;

        filteredMeasurement.gyroZ =
            rawMeasurement.gyroZ;


        filterInitialized = true;

        return;
    }


    // ---------------------------------------------------------
    // Linear acceleration EWA
    // ---------------------------------------------------------

    filteredMeasurement.accelNorth =
        ewa(
            filteredMeasurement.accelNorth,
            rawMeasurement.accelNorth
        );

    filteredMeasurement.accelEast =
        ewa(
            filteredMeasurement.accelEast,
            rawMeasurement.accelEast
        );

    filteredMeasurement.accelDown =
        ewa(
            filteredMeasurement.accelDown,
            rawMeasurement.accelDown
        );


    // ---------------------------------------------------------
    // Gyroscope EWA
    // ---------------------------------------------------------

    filteredMeasurement.gyroX =
        ewa(
            filteredMeasurement.gyroX,
            rawMeasurement.gyroX
        );

    filteredMeasurement.gyroY =
        ewa(
            filteredMeasurement.gyroY,
            rawMeasurement.gyroY
        );

    filteredMeasurement.gyroZ =
        ewa(
            filteredMeasurement.gyroZ,
            rawMeasurement.gyroZ
        );
}


float VN300::ewa(
    float previousValue,
    float newValue
) const
{
    return previousValue * (1.0f - filterAlpha)
        + newValue * filterAlpha;
}


void VN300::setFilterAlpha(float alpha)
{
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }

    if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }


    filterAlpha = alpha;
}


void VN300::validateMeasurement(
    const VN300Measurement& rawMeasurement
)
{
    // ---------------------------------------------------------
    // NaN / infinity detection
    // ---------------------------------------------------------

    if (!isfinite(rawMeasurement.yaw) ||
        !isfinite(rawMeasurement.pitch) ||
        !isfinite(rawMeasurement.roll) ||

        !isfinite(rawMeasurement.accelNorth) ||
        !isfinite(rawMeasurement.accelEast) ||
        !isfinite(rawMeasurement.accelDown) ||

        !isfinite(rawMeasurement.gyroX) ||
        !isfinite(rawMeasurement.gyroY) ||
        !isfinite(rawMeasurement.gyroZ))
    {
        setFault(SensorFault::InvalidData);

        return;
    }


    // ---------------------------------------------------------
    // Attitude sanity checking
    // ---------------------------------------------------------

    if (rawMeasurement.yaw < -180.0f ||
        rawMeasurement.yaw > 360.0f ||

        rawMeasurement.pitch < -180.0f ||
        rawMeasurement.pitch > 180.0f ||

        rawMeasurement.roll < -180.0f ||
        rawMeasurement.roll > 180.0f)
    {
        setFault(SensorFault::OutOfRange);

        return;
    }


    // ---------------------------------------------------------
    // Dynamic sanity limits
    //
    // Intentionally generous. These should catch corrupted
    // packets, not restrict the lander's flight envelope.
    // ---------------------------------------------------------

    constexpr float maxAcceleration = 200.0f;
    constexpr float maxGyroRate = 50.0f;


    if (fabsf(rawMeasurement.accelNorth) > maxAcceleration ||
        fabsf(rawMeasurement.accelEast) > maxAcceleration ||
        fabsf(rawMeasurement.accelDown) > maxAcceleration)
    {
        setFault(SensorFault::OutOfRange);

        return;
    }


    if (fabsf(rawMeasurement.gyroX) > maxGyroRate ||
        fabsf(rawMeasurement.gyroY) > maxGyroRate ||
        fabsf(rawMeasurement.gyroZ) > maxGyroRate)
    {
        setFault(SensorFault::OutOfRange);

        return;
    }
}


VN300Measurement VN300::getMeasurement() const
{
    VN300Measurement measurement =
        filteredMeasurement;


    // Zeroing applies only to attitude.

    measurement.yaw += offset.yaw;

    measurement.pitch += offset.pitch;

    measurement.roll += offset.roll;


    return measurement;
}


void VN300::zero()
{
    if (!hasValidData())
    {
        return;
    }


    offset.yaw =
        -filteredMeasurement.yaw;

    offset.pitch =
        -filteredMeasurement.pitch;

    offset.roll =
        -filteredMeasurement.roll;
}


void VN300::checkOverflowVectornav()
{
    if (dataIndex >= bufferSize - 1)
    {
        dataIndex = 0;

        incomingDataString = "";

        setFault(SensorFault::InvalidData);
    }
}




int VN300::checkHeaderVectornav()
{
    // Register 1
    if (incomingDataString.indexOf("$VNYPR") != -1)
    {
        return 1;
    }


    // Register 240
    if (incomingDataString.indexOf("$VNYIA") != -1)
    {
        return 240;
    }


    // Register 58 read response
    if (incomingDataString.indexOf("$VNRRG,58,") != -1)
    {
        return 58;
    }


    return -1;
}


void VN300::pollGnssSolution()
{
    vectornav.println("$VNRRG,58*XX");
}



void VN300::parseGnssSolution()
{
    VN300Measurement gnssMeasurement;

    char* payload = strstr(
        incomingData,
        "$VNRRG,58,"
    );

    if (payload == nullptr)
    {
        return;
    }

    // Move past "$VNRRG,58,"
    payload += strlen("$VNRRG,58,");

    char dummyHeader[4];
    char parseBuffer[bufferSize];

    // TextParser expects a header before the first comma.
    snprintf(
        parseBuffer,
        sizeof(parseBuffer),
        "GPS,%s",
        payload
    );

    commaParser.parseLine(
        parseBuffer,
        dummyHeader,

        gnssMeasurement.gpsTow,
        gnssMeasurement.gpsWeek,
        gnssMeasurement.gnssFix,
        gnssMeasurement.numSats,

        gnssMeasurement.latitude,
        gnssMeasurement.longitude,
        gnssMeasurement.altitude,

        gnssMeasurement.velocityNorth,
        gnssMeasurement.velocityEast,
        gnssMeasurement.velocityDown,

        gnssMeasurement.positionUncertaintyNorth,
        gnssMeasurement.positionUncertaintyEast,
        gnssMeasurement.positionUncertaintyDown,

        gnssMeasurement.velocityUncertainty,
        gnssMeasurement.timeUncertainty
    );

    // Register 58 is considered usable only with a 2D or 3D GNSS fix.
    if (gnssMeasurement.gnssFix < 2)
    {
        return;
    }

    if (!isfinite(gnssMeasurement.latitude) ||
        !isfinite(gnssMeasurement.longitude) ||
        !isfinite(gnssMeasurement.altitude) ||
        !isfinite(gnssMeasurement.velocityNorth) ||
        !isfinite(gnssMeasurement.velocityEast) ||
        !isfinite(gnssMeasurement.velocityDown) ||
        !isfinite(gnssMeasurement.positionUncertaintyNorth) ||
        !isfinite(gnssMeasurement.positionUncertaintyEast) ||
        !isfinite(gnssMeasurement.positionUncertaintyDown) ||
        !isfinite(gnssMeasurement.velocityUncertainty) ||
        !isfinite(gnssMeasurement.timeUncertainty))
    {
        return;
    }

    // Do not overwrite attitude, acceleration or gyro.
    // Those arrive from Register 240.

    filteredMeasurement.gpsTow =
        gnssMeasurement.gpsTow;

    filteredMeasurement.gpsWeek =
        gnssMeasurement.gpsWeek;

    filteredMeasurement.gnssFix =
        gnssMeasurement.gnssFix;

    filteredMeasurement.numSats =
        gnssMeasurement.numSats;

    filteredMeasurement.latitude =
        gnssMeasurement.latitude;

    filteredMeasurement.longitude =
        gnssMeasurement.longitude;

    filteredMeasurement.altitude =
        gnssMeasurement.altitude;

    filteredMeasurement.velocityNorth =
        gnssMeasurement.velocityNorth;

    filteredMeasurement.velocityEast =
        gnssMeasurement.velocityEast;

    filteredMeasurement.velocityDown =
        gnssMeasurement.velocityDown;

    filteredMeasurement.positionUncertaintyNorth =
        gnssMeasurement.positionUncertaintyNorth;

    filteredMeasurement.positionUncertaintyEast =
        gnssMeasurement.positionUncertaintyEast;

    filteredMeasurement.positionUncertaintyDown =
        gnssMeasurement.positionUncertaintyDown;

    filteredMeasurement.velocityUncertainty =
        gnssMeasurement.velocityUncertainty;

    filteredMeasurement.timeUncertainty =
        gnssMeasurement.timeUncertainty;
}
