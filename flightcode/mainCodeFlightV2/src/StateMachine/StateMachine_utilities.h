#pragma once


enum class STATE_MACHINE_STATES {
    BOOT,
    BOOTED,
    
    NOGO, // sensor error or other critical error, cannot proceed with launch, grounded
    
    SPOOL_UP,
    LAUNCH,
    
    NAVIGATION,
   
    PRE_LANDING,
    LANDING,

    SPOOL_DOWN,
    LANDED
};


