// ==================== LEONIDAS - VERTICAL LANDER - FLIGHT V1 ====================
//
//  FIRST FLIGHT TEST - LEONIDAS - basic vertical control.
//
//  This file is intentionally (almost) empty. Arduino concatenates the sketch's
//  .ino files alphabetically, with this main file first, so the project is split
//  by responsibility using a letter-prefix ordering convention:
//
//    a_userParameters.ino    - values the user is meant to tune
//    b_systemParameters.ino  - libraries, hardware objects, pins, runtime state
//    c_parametersCheck.ino   - compile-time static_assert sanity checks
//    e_..n_ utilities        - subsystem logic (setup, sensors, PID, telemetry...)
//    z_mainBody.ino          - setup() and loop()
//
//  Keep declarations in a_/b_ so every later file can see them (declared before use).
// ================================================================================
