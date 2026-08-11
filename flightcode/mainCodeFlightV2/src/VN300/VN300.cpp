#include "VN300.h"

#include <math.h>
#include "../Utilities/Utilities.h"
#include <string.h>

#include "../Config/SensorConfig.h"


VN300::VN300()
    : vectornav(nullptr),
      commaParser(",")
{
}


bool VN300::begin()
{
    Serial.println("INITIALIZING VECTORNAV");

    vectornav = SensorConfig::VN300.port;

    if (vectornav == nullptr)
    {
        setFault(
            SensorFault::CommunicationError
        );

        return false;
    }

    vectornav->begin(
        SensorConfig::VN300.baudrate
    );

    delay(200);


    // Register 240:
    // Yaw / Pitch / Roll
    // Linear acceleration NED
    // Compensated angular rates
    vectornav->println(
        "$VNWRG,6,17*XX"
    );


    // Asynchronous output frequency
    vectornav->println(
        "$VNWRG,7,300*XX"
    );


    dataIndex = 0;

    incomingDataString = "";

    offset = {};

    filteredMeasurement = {};

    filterInitialized = false;


    initializeSensorState(
        vn300TimeoutMs
    );


    return true;
}



void VN300::update()
{
    const uint32_t now = millis();

    // Periodically request GNSS data
    if (now - lastGnssPollMs >= gnssPollPeriodMs)
    {
        lastGnssPollMs = now;

        pollGnssSolution();
    }


    // Consume exactly one byte per update
    if (vectornav->available() > 0)
    {
        const char incomingChar = vectornav->read();


        // Complete message
        if (incomingChar == '\n')
        {
            incomingData[dataIndex] = '\0';

            processVectornav();

            dataIndex = 0;
            incomingDataString = "";

            return;
        }


        // Ignore carriage return
        if (incomingChar == '\r')
        {
            return;
        }


        // Prevent buffer overflow
        if (dataIndex >= bufferSize - 1)
        {
            dataIndex = 0;
            incomingDataString = "";

            setFault(
                SensorFault::InvalidData
            );

            return;
        }


        // Store byte
        incomingData[dataIndex] = incomingChar;
        dataIndex++;

        incomingDataString += incomingChar;
    }


    checkTimeout();
}


void VN300::processVectornav()
{
    int vectornavIdentity =
        checkHeaderVectornav();


    char headerVectornav[10];


    switch (vectornavIdentity)
    {
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

            validateMeasurement(
                rawMeasurement
            );


            if (isHealthy())
            {
                storeRawMeasurement(
                    rawMeasurement
                );

                filteredMeasurement.yaw =
                    rawMeasurement.yaw;

                filteredMeasurement.pitch =
                    rawMeasurement.pitch;

                filteredMeasurement.roll =
                    rawMeasurement.roll;
            }

            break;
        }


        case 58:
        {
            parseGnssSolution();

            break;
        }


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

            validateMeasurement(
                rawMeasurement
            );


            if (isHealthy())
            {
                storeRawMeasurement(
                    rawMeasurement
                );

                applyFilter(
                    rawMeasurement
                );
            }

            break;
        }


        default:
            break;
    }
}


void VN300::applyFilter(
    const VN300Measurement& rawMeasurement
)
{
    // ---------------------------------------------------------
    // Attitude
    // ---------------------------------------------------------

    filteredMeasurement.yaw =
        rawMeasurement.yaw;

    filteredMeasurement.pitch =
        rawMeasurement.pitch;

    filteredMeasurement.roll =
        rawMeasurement.roll;


    // ---------------------------------------------------------
    // First valid measurement
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
        Utilities::EWA(
            filterAlpha,
            filteredMeasurement.accelNorth,
            rawMeasurement.accelNorth
        );

    filteredMeasurement.accelEast =
        Utilities::EWA(
            filterAlpha,
            filteredMeasurement.accelEast,
            rawMeasurement.accelEast
        );

    filteredMeasurement.accelDown =
        Utilities::EWA(
            filterAlpha,
            filteredMeasurement.accelDown,
            rawMeasurement.accelDown
        );


    // ---------------------------------------------------------
    // Angular velocity EWA
    // ---------------------------------------------------------

    filteredMeasurement.gyroX =
        Utilities::EWA(
            filterAlpha,
            filteredMeasurement.gyroX,
            rawMeasurement.gyroX
        );

    filteredMeasurement.gyroY =
        Utilities::EWA(
            filterAlpha,
            filteredMeasurement.gyroY,
            rawMeasurement.gyroY
        );

    filteredMeasurement.gyroZ =
        Utilities::EWA(
            filterAlpha,
            filteredMeasurement.gyroZ,
            rawMeasurement.gyroZ
        );
}


void VN300::setFilterAlpha(
    float alpha
)
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
    if (
        !isfinite(rawMeasurement.yaw) ||
        !isfinite(rawMeasurement.pitch) ||
        !isfinite(rawMeasurement.roll) ||

        !isfinite(rawMeasurement.accelNorth) ||
        !isfinite(rawMeasurement.accelEast) ||
        !isfinite(rawMeasurement.accelDown) ||

        !isfinite(rawMeasurement.gyroX) ||
        !isfinite(rawMeasurement.gyroY) ||
        !isfinite(rawMeasurement.gyroZ)
    )
    {
        setFault(
            SensorFault::InvalidData
        );

        return;
    }


    if (
        rawMeasurement.yaw < -180.0f ||
        rawMeasurement.yaw > 360.0f ||

        rawMeasurement.pitch < -180.0f ||
        rawMeasurement.pitch > 180.0f ||

        rawMeasurement.roll < -180.0f ||
        rawMeasurement.roll > 180.0f
    )
    {
        setFault(
            SensorFault::OutOfRange
        );

        return;
    }


    constexpr float maxAcceleration =
        200.0f;

    constexpr float maxGyroRate =
        50.0f;


    if (
        fabsf(rawMeasurement.accelNorth) >
            maxAcceleration ||

        fabsf(rawMeasurement.accelEast) >
            maxAcceleration ||

        fabsf(rawMeasurement.accelDown) >
            maxAcceleration
    )
    {
        setFault(
            SensorFault::OutOfRange
        );

        return;
    }


    if (
        fabsf(rawMeasurement.gyroX) >
            maxGyroRate ||

        fabsf(rawMeasurement.gyroY) >
            maxGyroRate ||

        fabsf(rawMeasurement.gyroZ) >
            maxGyroRate
    )
    {
        setFault(
            SensorFault::OutOfRange
        );

        return;
    }
}


VN300Measurement VN300::getMeasurement() const
{
    VN300Measurement measurement =
        filteredMeasurement;


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

        setFault(
            SensorFault::InvalidData
        );
    }
}


int VN300::checkHeaderVectornav()
{
    if (
        incomingDataString.indexOf(
            "$VNYPR"
        ) != -1
    )
    {
        return 1;
    }


    if (
        incomingDataString.indexOf(
            "$VNYIA"
        ) != -1
    )
    {
        return 240;
    }


    if (
        incomingDataString.indexOf(
            "$VNRRG,58,"
        ) != -1
    )
    {
        return 58;
    }


    return -1;
}


void VN300::pollGnssSolution()
{
    vectornav->println("$VNRRG,58*XX" );
}


void VN300::parseGnssSolution()
{
    VN300Measurement gnssMeasurement;


    char* payload =
        strstr(
            incomingData,
            "$VNRRG,58,"
        );


    if (payload == nullptr)
    {
        return;
    }


    payload += strlen(
        "$VNRRG,58,"
    );


    char dummyHeader[4];

    char parseBuffer[bufferSize];


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


    if (gnssMeasurement.gnssFix < 2)
    {
        return;
    }


    if (
        !isfinite(gnssMeasurement.latitude) ||
        !isfinite(gnssMeasurement.longitude) ||
        !isfinite(gnssMeasurement.altitude) ||
        !isfinite(gnssMeasurement.velocityNorth) ||
        !isfinite(gnssMeasurement.velocityEast) ||
        !isfinite(gnssMeasurement.velocityDown) ||
        !isfinite(
            gnssMeasurement.positionUncertaintyNorth
        ) ||
        !isfinite(
            gnssMeasurement.positionUncertaintyEast
        ) ||
        !isfinite(
            gnssMeasurement.positionUncertaintyDown
        ) ||
        !isfinite(
            gnssMeasurement.velocityUncertainty
        ) ||
        !isfinite(
            gnssMeasurement.timeUncertainty
        )
    )
    {
        return;
    }


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