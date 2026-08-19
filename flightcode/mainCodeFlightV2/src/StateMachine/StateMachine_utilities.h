#pragma once


enum class STATE_MACHINE_STATES {
    BOOT,
    AWAIT,
    GO,
    NOGO, // sensor error or other critical error, cannot proceed with launch, grounded
    

    LAUNCH,
    POST_LAUNCH_HOVER,
    
    NAVIGATION,
   
    PRE_LANDING,
    LANDING,

    LANDED,
    ABORT
};


