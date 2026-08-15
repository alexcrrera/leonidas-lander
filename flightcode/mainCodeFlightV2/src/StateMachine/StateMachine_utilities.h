#pragma once


enum class STATE_MACHINE_STATES {
    BOOT,
    BOOTED,
    STANDBY,
    NOGO, // sensor error or other critical error, cannot proceed with launch, grounded
    PRE_LAUNCH, // pre launch checks, waiting for launch command
    SPOOL_UP,
    LAUNCH,
    ASCENT,
    DESCENT,
    LANDING,
    SPOOL_DOWN,
    LANDED
};


