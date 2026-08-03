// ================================================ THERE IS A PARAMETER SANITY CHECK (c_parametersCheck.ino) BUT USERS SHOULD TRIPLECHECK THESE VALUES ===================================

// ======================================================================================================================================================================
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==================================== USER PARAMETERS ================================================================================================
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ======================================================================================================================================================================
//
//  This file is concatenated FIRST (after the empty main sketch file) because Arduino
//  builds .ino files in alphabetical order and globals must be declared before use.
//  Keep it limited to values a user is expected to tune.
//
//  SECTION 1 (constexpr) = compile-time constants. They are verified by static_assert
//                          in c_parametersCheck.ino, so a bad value fails the BUILD,
//                          not the flight. Never made writable at runtime.
//  SECTION 2 (mutable)   = startup DEFAULTS that can also be changed live over telemetry
//                          (PID gains, TVC limits, etc). Must stay non-const.



// ======================================================================================================================================================================
// ---------------------------------------------- SECTION 1 : FIXED USER CONSTANTS (compile-time verified) ----------------------------------------------
// ======================================================================================================================================================================

//////////////////////////////////// PWM / ACTUATOR RATES ////////////////////////////////////
constexpr int   SERVOFREQUENCY = 333;   // TVC vane PWM frequency [Hz]
constexpr int   resBit         = 12;    // analogWrite() resolution [bits] (Teensy)

// //////////////////////////////////// ESC PULSE CALIBRATION ////////////////////////////////////
// constexpr int   ESCLOWPOINT  = 1000;    // ESC min pulse [us] (0% throttle)
// constexpr int   ESCHIGHPOINT = 2000;    // ESC max pulse [us] (100% throttle)

//////////////////////////////////// MOTOR LIMITS [%] ////////////////////////////////////
// constexpr float MOTORMAX     = 80.0f;   // max commandable motor output [%]
// constexpr float MOTORMIN     = 50.0f;   // min controllable motor output [%]
// constexpr float MOTORSTARTUP = 67.0f;   // motor output at the start of the take-off ramp [%]

//////////////////////////////////// FLIGHT PROFILE ////////////////////////////////////
constexpr float TAKEOFFALTITUDE = 0.75f; // target hover/altitude setpoint [m]
constexpr float LANDINGSPEED    = 0.21f; // vertical ramp rate for take-off/landing [m/s]
constexpr float maxAbortAngle   = 45.0f; // tilt beyond which the flight is aborted [deg]
constexpr float timeFlight      = 21.0f; // auto-land timeout after take-off [s]
constexpr float timeFlightCheck = 1.0f;  // motor spool-up duration before take-off [s]

//////////////////////////////////// POSITION / ALTITUDE TOLERANCES ////////////////////////////////////
constexpr float distanceEpsilon = 0.085f; // waypoint-reached radius [m]
constexpr float verticalTol     = 0.05f;  // vertical setpoint tolerance [m]

//////////////////////////////////// SENSOR FUSION / FILTER GAINS ////////////////////////////////////
constexpr float RTK_2_LiDAR_RATIO          = 0.7f;  // altitude blend: 1.0 = pure RTK, 0.0 = pure LiDAR
constexpr float lidarAlphaEWA              = 0.21f; // LiDAR EWMA smoothing factor (0..1)
constexpr float EMWA_VECTORNAV             = 0.10f; // IMU angle EWMA smoothing factor (0..1)
constexpr float EWMA_PID_ORIENTATION_SERVO = 0.33f; // TVC output EWMA smoothing factor (0..1)

//////////////////////////////////// TVC VANE TRIM [deg] ////////////////////////////////////
// Mechanical centering offset per vane. Applied on top of the PID command.
// constexpr float MissSX1 = -1.0f * 12.0f;
// constexpr float MissSX2 = -1.0f * 3.4f;
// constexpr float MissSY1 = -1.0f * 1.0f;
// constexpr float MissSY2 = -1.0f * -5.0f;

//////////////////////////////////// VANE CONFIG ////////////////////////////////////
constexpr bool isBluebird = false; // selects the servo pulse range used by servoCalculator2()

// ---- CM509MG vane calibration (used by servoCalculator) -------------------------------------
// 0 deg = center = 1500 us, symmetric range, 333 Hz, resolution = resBit.
constexpr float servoMinAngleDeg   = -50.0f;
constexpr float servoMaxAngleDeg   =  50.0f;
constexpr float servoMinPulseUs    = 1000.0f;
constexpr float servoCenterPulseUs = 1500.0f;
constexpr float servoMaxPulseUs    = 2000.0f;



// ======================================================================================================================================================================
// ---------------------------------------------- SECTION 2 : TUNABLE DEFAULTS (also editable live over telemetry) ----------------------------------------------
// ======================================================================================================================================================================
// These MUST stay non-const: the telemetry command parser writes to them at runtime
// (see i_telemetryUtilities.ino). The values here are only the power-on defaults.

//////////////////////////////////// TELEMETRY / LOGGING RATES [Hz] ////////////////////////////////////
int TELEMETRYFREQUENCY = 10; // telemetry downlink rate (cmd "TLM")
int PLTRATE            = 25; // serial plot/print rate   (cmd "PLT")

//////////////////////////////////// ORIENTATION PID GAINS ////////////////////////////////////
float pGainAngleX = 1.0; float iGainAngleX = 0.1; float dGainAngleX = 0.15; // cmd PID-X
float pGainAngleY = 1.0; float iGainAngleY = 0.1; float dGainAngleY = 0.15; // cmd PID-Y
float pGainAngleZ = 1.0; float iGainAngleZ = 0.1; float dGainAngleZ = 0.15; // cmd PID-Z

//////////////////////////////////// TVC ANGLE LIMITS [deg] ////////////////////////////////////
// float tvcMaxAngle  = 12.0;            // total TVC authority           (cmd "MAXANGLETVC")
// float maxAngleZTVC = 5.0;             // roll (Z) authority budget
// float maxAngleXYTVC = tvcMaxAngle - maxAngleZTVC; // pitch/yaw (X/Y) authority budget
// float rollOffset   = 0;               // roll trim added to Z PID output (cmd index 14)
// float abortAngle   = 45.0;            // live abort-angle setpoint     (cmd "ABORTANGLE")
