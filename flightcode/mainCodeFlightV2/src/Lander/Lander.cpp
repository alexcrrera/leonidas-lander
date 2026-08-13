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

    stateEstimator.update();

   

    clearSolutionStatus();

    

    estimateAttitude();
    

    estimateAcceleration();
    

    estimateVelocity();
    

    estimatePosition();
    

    estimateAltitude();
    

    updateErrors();

    
}

void Lander::clearSolutionStatus()
{
    solution.validity.positionValid = false;
    solution.validity.velocityValid = false;
    solution.validity.accelerationValid = false;
    solution.validity.attitudeValid = false;
    solution.validity.altitudeValid = false;

    solution.source.position = PositionSource::None;
    solution.source.velocity = VelocitySource::None;
    solution.source.altitude = AltitudeSource::None;
    solution.source.attitude = AttitudeSource::None;

    solution.errors = LANDER_ERROR_NONE;
}

bool Lander::gnssSolutionUsable(
    const VN300Measurement& measurement
) const
{
    // Register 58 fix:
    // 0 = no fix
    // 1 = time only
    // 2 = 2D
    // 3 = 3D
    //
    // Require a 3D fix for the lander navigation solution.

    if (measurement.gnssFix < 3)
    {
        return false;
    }

    if (measurement.gpsWeek == 0)
    {
        return false;
    }

    if (!isfinite(measurement.latitude) ||
        !isfinite(measurement.longitude) ||
        !isfinite(measurement.altitude) ||
        !isfinite(measurement.velocityNorth) ||
        !isfinite(measurement.velocityEast) ||
        !isfinite(measurement.velocityDown) ||
        !isfinite(measurement.positionUncertaintyNorth) ||
        !isfinite(measurement.positionUncertaintyEast) ||
        !isfinite(measurement.positionUncertaintyDown) ||
        !isfinite(measurement.velocityUncertainty) ||
        !isfinite(measurement.timeUncertainty))
    {
        return false;
    }

    return true;
}

void Lander::estimateAttitude()
{
    if (!vn300.isHealthy() || !vn300.hasValidData())
    {
        return;
    }

    VN300Measurement measurement = vn300.getMeasurement();

    solution.state.attitude.Roll_SI = measurement.roll;
    solution.state.attitude.Pitch_SI = measurement.pitch;
    solution.state.attitude.Yaw_SI = measurement.yaw;

    solution.state.angularVelocity.Roll_SI = measurement.gyroX;
    solution.state.angularVelocity.Pitch_SI = measurement.gyroY;
    solution.state.angularVelocity.Yaw_SI = measurement.gyroZ;

    solution.validity.attitudeValid = true;
    solution.source.attitude = AttitudeSource::VN300;
}

void Lander::estimateAcceleration()
{
    if (!vn300.isHealthy() || !vn300.hasValidData())
    {
        return;
    }

    VN300Measurement measurement = vn300.getMeasurement();

    // Register 240 provides linear acceleration in NED.
    solution.state.acceleration.North_SI = measurement.accelNorth;
    solution.state.acceleration.East_SI = measurement.accelEast;
    solution.state.acceleration.Down_SI = measurement.accelDown;

    solution.validity.accelerationValid = true;
}

void Lander::estimateVelocity()
{
    if (
        !opticalFlow.isHealthy() ||
        !opticalFlow.hasValidData()
    )
    {
        return;
    }

    OpticalFlowMeasurement measurement =
        opticalFlow.getMeasurement();

    solution.state.velocity.North_SI =
        measurement.flowRateX;

    solution.state.velocity.East_SI =
        measurement.flowRateY;

    // Optical flow does not provide vertical velocity.
    solution.state.velocity.Down_SI =
        0.0f;

    solution.validity.velocityValid = true;

    solution.source.velocity =
        VelocitySource::OpticalFlow;
}




void Lander::initializePositionOrigin(
    const VN300Measurement& measurement
)
{
    originLatitude = measurement.latitude;
    originLongitude = measurement.longitude;
    originAltitude = measurement.altitude;

    positionOriginInitialized = true;
}

void Lander::convertLlaToLocalNed(
    const VN300Measurement& measurement,
    NED_coordinates& position
) const
{
    constexpr double earthRadiusM = 6378137.0;
    constexpr double degToRad = PI / 180.0;

    const double latitude0Rad =
        originLatitude * degToRad;

    const double deltaLatitudeRad =
        (measurement.latitude - originLatitude) * degToRad;

    const double deltaLongitudeRad =
        (measurement.longitude - originLongitude) * degToRad;

    position.North_SI = static_cast<float>(
        deltaLatitudeRad * earthRadiusM
    );

    position.East_SI = static_cast<float>(
        deltaLongitudeRad *
        earthRadiusM *
        cos(latitude0Rad)
    );

    // NED convention: positive Z is Down.
    position.Down_SI = static_cast<float>(
        originAltitude - measurement.altitude
    );
}

void Lander::estimatePosition()
{
    if (!vn300.isHealthy() || !vn300.hasValidData())
    {
        return;
    }

    VN300Measurement measurement = vn300.getMeasurement();

    if (!gnssSolutionUsable(measurement))
    {
        return;
    }

    if (!positionOriginInitialized)
    {
        initializePositionOrigin(measurement);
    }

    convertLlaToLocalNed(
        measurement,
        solution.state.position
    );

    solution.validity.positionValid = true;
    solution.source.position = PositionSource::VN300;
}

void Lander::estimateAltitude()
{
    const bool lidarUsable =
        lidar.isHealthy() &&
        lidar.hasValidData();

    if (!lidarUsable)
    {
        solution.validity.altitudeValid = false;
        solution.source.altitude = AltitudeSource::None;

        return;
    }

    LidarMeasurement measurement = lidar.getMeasurement();

    // Positive height above ground.
    solution.state.altitude = measurement.filteredDistanceM;

    solution.validity.altitudeValid = true;
    solution.source.altitude = AltitudeSource::Lidar;
}

void Lander::updateErrors()
{
    if (!vn300.isHealthy())
    {
        setError(LANDER_ERROR_VN300_UNAVAILABLE);
    }

    if (!lidar.isHealthy())
    {
        setError(LANDER_ERROR_LIDAR_UNAVAILABLE);
    }

    if (vn300.isHealthy() && vn300.hasValidData())
    {
        VN300Measurement measurement = vn300.getMeasurement();

        if (!gnssSolutionUsable(measurement))
        {
            setError(LANDER_ERROR_GNSS_INVALID);
        }
    }

    if (!solution.validity.positionValid)
    {
        setError(LANDER_ERROR_NO_POSITION);
    }

    if (!solution.validity.velocityValid)
    {
        setError(LANDER_ERROR_NO_VELOCITY);
    }

    if (!solution.validity.altitudeValid)
    {
        setError(LANDER_ERROR_NO_ALTITUDE);
    }

    if (!solution.validity.attitudeValid)
    {
        setError(LANDER_ERROR_NO_ATTITUDE);
    }
}

void Lander::setError(LanderError error)
{
    solution.errors |= static_cast<uint16_t>(error);
}

bool Lander::hasError(LanderError error) const
{
    return (
        solution.errors &
        static_cast<uint16_t>(error)
    ) != 0;
}

uint16_t Lander::getErrors() const
{
    return solution.errors;
}

const LanderSolution& Lander::getSolution() const
{
    return solution;
}

bool Lander::hasPositionSolution() const
{
    return solution.validity.positionValid;
}

bool Lander::hasVelocitySolution() const
{
    return solution.validity.velocityValid;
}

bool Lander::hasAltitudeSolution() const
{
    return solution.validity.altitudeValid;
}

bool Lander::hasAttitudeSolution() const
{
    return solution.validity.attitudeValid;
}

void Lander::printStatus(Stream& serialPort) const
{
    serialPort.println();
    serialPort.println("================================");
    serialPort.println("            LANDER");
    serialPort.println("================================");

    serialPort.println("----- ATTITUDE -----");

    serialPort.print("Valid: ");
    serialPort.println(solution.validity.attitudeValid);

    if (solution.validity.attitudeValid)
    {
        serialPort.print("Roll:  ");
        serialPort.println(solution.state.attitude.Roll_SI, 3);

        serialPort.print("Pitch: ");
        serialPort.println(solution.state.attitude.Pitch_SI, 3);

        serialPort.print("Yaw:   ");
        serialPort.println(solution.state.attitude.Yaw_SI, 3);

        serialPort.print("Gyro X: ");
        serialPort.println(solution.state.angularVelocity.Roll_SI, 5);

        serialPort.print("Gyro Y: ");
        serialPort.println(solution.state.angularVelocity.Pitch_SI, 5);

        serialPort.print("Gyro Z: ");
        serialPort.println(solution.state.angularVelocity.Yaw_SI, 5);
    }
    else
    {
        serialPort.println("NO ATTITUDE SOLUTION");
    }

    serialPort.println();
    serialPort.println("----- ACCELERATION NED -----");

    serialPort.print("Valid: ");
    serialPort.println(solution.validity.accelerationValid);

    if (solution.validity.accelerationValid)
    {
        serialPort.print("North: ");
        serialPort.print(solution.state.acceleration.North_SI, 3);
        serialPort.println(" m/s^2");

        serialPort.print("East:  ");
        serialPort.print(solution.state.acceleration.East_SI, 3);
        serialPort.println(" m/s^2");

        serialPort.print("Down:  ");
        serialPort.print(solution.state.acceleration.Down_SI, 3);
        serialPort.println(" m/s^2");
    }
    else
    {
        serialPort.println("NO ACCELERATION SOLUTION");
    }

    serialPort.println();
    serialPort.println("----- POSITION NED -----");

    serialPort.print("Valid: ");
    serialPort.println(solution.validity.positionValid);

    if (solution.validity.positionValid)
    {
        serialPort.print("North: ");
        serialPort.print(solution.state.position.North_SI, 3);
        serialPort.println(" m");

        serialPort.print("East:  ");
        serialPort.print(solution.state.position.East_SI, 3);
        serialPort.println(" m");

        serialPort.print("Down:  ");
        serialPort.print(solution.state.position.Down_SI, 3);
        serialPort.println(" m");
    }
    else
    {
        serialPort.println("NO POSITION SOLUTION");
    }

    serialPort.println();
    serialPort.println("----- VELOCITY NED -----");

    serialPort.print("Valid: ");
    serialPort.println(solution.validity.velocityValid);

    if (solution.validity.velocityValid)
    {
        serialPort.print("North: ");
        serialPort.print(solution.state.velocity.North_SI, 3);
        serialPort.println(" m/s");

        serialPort.print("East:  ");
        serialPort.print(solution.state.velocity.East_SI, 3);
        serialPort.println(" m/s");

        serialPort.print("Down:  ");
        serialPort.print(solution.state.velocity.Down_SI, 3);
        serialPort.println(" m/s");
    }
    else
    {
        serialPort.println("NO VELOCITY SOLUTION");
    }

    serialPort.println();
    serialPort.println("----- ALTITUDE -----");

    serialPort.print("Valid: ");
    serialPort.println(solution.validity.altitudeValid);

    if (solution.validity.altitudeValid)
    {
        serialPort.print("Altitude AGL: ");
        serialPort.print(solution.state.altitude, 3);
        serialPort.println(" m");

        serialPort.println("Source: LIDAR");
    }
    else
    {
        serialPort.println("NO ALTITUDE SOLUTION");
    }

    serialPort.println();
    serialPort.println("----- VN300 GNSS -----");

    if (vn300.hasValidData())
    {
        VN300Measurement measurement = vn300.getMeasurement();

        serialPort.print("GPS TOW: ");
        serialPort.println(measurement.gpsTow, 3);

        serialPort.print("GPS Week: ");
        serialPort.println(measurement.gpsWeek);

        serialPort.print("GNSS Fix: ");
        serialPort.println(measurement.gnssFix);

        serialPort.print("Satellites: ");
        serialPort.println(measurement.numSats);

        serialPort.print("Latitude: ");
        serialPort.println(measurement.latitude, 8);

        serialPort.print("Longitude: ");
        serialPort.println(measurement.longitude, 8);

        serialPort.print("Ellipsoid altitude: ");
        serialPort.print(measurement.altitude, 3);
        serialPort.println(" m");

        serialPort.print("Velocity N: ");
        serialPort.print(measurement.velocityNorth, 3);
        serialPort.println(" m/s");

        serialPort.print("Velocity E: ");
        serialPort.print(measurement.velocityEast, 3);
        serialPort.println(" m/s");

        serialPort.print("Velocity D: ");
        serialPort.print(measurement.velocityDown, 3);
        serialPort.println(" m/s");

        serialPort.print("Position uncertainty N: ");
        serialPort.print(
            measurement.positionUncertaintyNorth,
            3
        );
        serialPort.println(" m");

        serialPort.print("Position uncertainty E: ");
        serialPort.print(
            measurement.positionUncertaintyEast,
            3
        );
        serialPort.println(" m");

        serialPort.print("Position uncertainty D: ");
        serialPort.print(
            measurement.positionUncertaintyDown,
            3
        );
        serialPort.println(" m");

        serialPort.print("Velocity uncertainty: ");
        serialPort.print(
            measurement.velocityUncertainty,
            3
        );
        serialPort.println(" m/s");

        serialPort.print("Time uncertainty: ");
        serialPort.print(
            measurement.timeUncertainty,
            9
        );
        serialPort.println(" s");
    }

    serialPort.println();
    serialPort.println("----- ERRORS -----");

    serialPort.print("Bitmask: ");
    serialPort.println(solution.errors);

    if (solution.errors == LANDER_ERROR_NONE)
    {
        serialPort.println("NONE");
        return;
    }

    if (hasError(LANDER_ERROR_NO_POSITION))
    {
        serialPort.println("NO_POSITION");
    }

    if (hasError(LANDER_ERROR_NO_ALTITUDE))
    {
        serialPort.println("NO_ALTITUDE");
    }

    if (hasError(LANDER_ERROR_NO_ATTITUDE))
    {
        serialPort.println("NO_ATTITUDE");
    }

    if (hasError(LANDER_ERROR_NO_VELOCITY))
    {
        serialPort.println("NO_VELOCITY");
    }

    if (hasError(LANDER_ERROR_VN300_UNAVAILABLE))
    {
        serialPort.println("VN300_UNAVAILABLE");
    }

    if (hasError(LANDER_ERROR_LIDAR_UNAVAILABLE))
    {
        serialPort.println("LIDAR_UNAVAILABLE");
    }

    if (hasError(LANDER_ERROR_GNSS_INVALID))
    {
        serialPort.println("GNSS_INVALID");
    }

    if (hasError(LANDER_ERROR_SOLUTION_STALE))
    {
        serialPort.println("SOLUTION_STALE");
    }
}
