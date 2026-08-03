
#include "Utilities.h"
#include "Arduino.h"

 namespace MissionConfig {

    constexpr ParameterConfig<uint32_t> holdTimeMs = {
        .min = 0,
        .max = 20 * 1000, // in ms
        .default_ = 0 // default is 0 means we move on automaticallyS
    };


 }

