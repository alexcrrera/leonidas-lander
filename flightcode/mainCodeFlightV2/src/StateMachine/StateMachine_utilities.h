#pragma once



enum class STATE_MACHINE_STATES {
    BOOT,
    STANDBY,
    NOGO,
    PRE_LAUNCH,
    LAUNCH,
    ASCENT,
    DESCENT,
    LANDING,
    LANDED
};