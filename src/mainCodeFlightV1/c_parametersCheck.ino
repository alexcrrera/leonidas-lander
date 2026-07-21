// ======================================================================================================================================================================
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==================================== USER PARAMETERS CHECK - **DO NOT** MODIFY ================================================================================================
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ======================================================================================================================================================================
//
//  Compile-time sanity checks on the FIXED user constants in a_userParameters.ino.
//  A violated assumption fails the BUILD with the message shown, so a mistuned
//  constant can never reach the vehicle. Only constexpr values can be checked here;
//  the live-tunable defaults (Section 2 of a_userParameters.ino) are clamped at runtime.

// ---- PWM / actuator rates ----
static_assert(SERVOFREQUENCY > 0,            "SERVOFREQUENCY must be > 0");
static_assert(SERVOFREQUENCY <= 500,         "SERVOFREQUENCY too high for analog servos");
static_assert(resBit >= 8 && resBit <= 15,   "resBit out of Teensy PWM range (8..15)");

// ---- ESC pulse calibration ----
static_assert(ESCLOWPOINT >= 800,            "ESCLOWPOINT suspiciously low");
static_assert(ESCHIGHPOINT <= 2200,          "ESCHIGHPOINT suspiciously high");
static_assert(ESCLOWPOINT < ESCHIGHPOINT,    "ESCLOWPOINT >= ESCHIGHPOINT");

// ---- Motor limits ----
static_assert(MOTORMIN >= 0.0f,              "MOTORMIN < 0");
static_assert(MOTORMAX <= 100.0f,            "MOTORMAX > 100");
static_assert(MOTORMIN < MOTORMAX,           "MOTORMIN >= MOTORMAX");
static_assert(MOTORSTARTUP >= MOTORMIN && MOTORSTARTUP <= MOTORMAX, "MOTORSTARTUP outside [MOTORMIN, MOTORMAX]");

// ---- Flight profile ----
static_assert(TAKEOFFALTITUDE > 0.0f,        "TAKEOFFALTITUDE must be > 0");
static_assert(LANDINGSPEED > 0.0f,           "LANDINGSPEED must be > 0");
static_assert(maxAbortAngle > 0.0f && maxAbortAngle < 90.0f, "maxAbortAngle must be in (0, 90)");
static_assert(timeFlight > 0.0f,             "timeFlight must be > 0");
static_assert(timeFlightCheck > 0.0f,        "timeFlightCheck must be > 0");

// ---- Tolerances ----
static_assert(distanceEpsilon > 0.0f,        "distanceEpsilon must be > 0");
static_assert(verticalTol > 0.0f,            "verticalTol must be > 0");

// ---- Filter gains must be valid EWMA weights in (0, 1] ----
static_assert(RTK_2_LiDAR_RATIO >= 0.0f && RTK_2_LiDAR_RATIO <= 1.0f,                   "RTK_2_LiDAR_RATIO must be in [0, 1]");
static_assert(lidarAlphaEWA > 0.0f && lidarAlphaEWA <= 1.0f,                            "lidarAlphaEWA must be in (0, 1]");
static_assert(EMWA_VECTORNAV > 0.0f && EMWA_VECTORNAV <= 1.0f,                          "EMWA_VECTORNAV must be in (0, 1]");
static_assert(EWMA_PID_ORIENTATION_SERVO > 0.0f && EWMA_PID_ORIENTATION_SERVO <= 1.0f,  "EWMA_PID_ORIENTATION_SERVO must be in (0, 1]");

// ---- Vane pulse calibration ----
static_assert(servoMinAngleDeg < servoMaxAngleDeg,                       "servo min/max angle swapped");
static_assert(servoMinPulseUs < servoCenterPulseUs && servoCenterPulseUs < servoMaxPulseUs, "servo pulse calibration not monotonic");
