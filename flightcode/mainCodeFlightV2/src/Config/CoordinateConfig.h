 
#include "Utilities.h"
#include "SafetyBounds.h"

enum class CoordinateType {
    Altitude_m,
    North_m,
    East_m,
    Down_m,
    Yaw_deg
};




 namespace CoordinateConfig {

    constexpr ParameterConfig<float> Altitude_m = { // max accepted values for the  commanded coordinates
        .min =SafetyBounds::minHoverAltitude_m, //minimum flight altitude
        .max =  SafetyBounds::maxAltitude_m, // max possible altitude that the aircraft can be directed to 
        .default_ = 0.75
    };

    constexpr ParameterConfig<float> North_m = { // 
        .min = - 2,
        .max = 1.0,
        .default_ = 0.1
    };

    constexpr ParameterConfig<float> East_m = { // Tolerances for VERTICAL NED positioning
        .min = - 2,
        .max = 1.0,
        .default_ = 0.1
    };

    constexpr ParameterConfig<float> Yaw_deg = { //
        .min = SafetyBounds::minYaw_deg,
        .max = SafetyBounds::maxYaw_deg,
        .default_ = 0
    };


    // DO NOT MODIFY

    constexpr ParameterConfig<float> Down_m = { // should be negative as the convention dictates it (negative *altitude* is not permitted)
        .min = -Altitude_m.max, // reversed as negative sign is used. If 0.5< altitude < 2 then -0.5 > altitude > -2 ...
        .max = -Altitude_m.min, // NED convention > 
        .default_ = -Altitude_m.default_
    };
       


    constexpr const ParameterConfig<float>& getData(CoordinateType type){

        switch (type)
        {
        case CoordinateType::Altitude_m:
            return(Altitude_m);

        case CoordinateType::North_m:
            return(North_m);

        case CoordinateType::East_m:
            return(East_m);

        case CoordinateType::Down_m:
            return(Down_m);

        case CoordinateType::Yaw_deg:
            return(Yaw_deg);

        default:
             return(Altitude_m); // safeguard, should never happen
        }
    }



    bool isValidCoordinate(float val, CoordinateType type){
        // checks if coordinate is within the specified range

        ParameterConfig coord_data = getData(type);

        if(Utilities::isBounded(val,coord_data.min, coord_data.max)){
            return true;
        }

        return false; // coordinate out of range

    }

    

    bool isValidPositionNED(const Vector3& positionNED){
    // checks if NED positions is valid

    float North_m = positionNED.x;
    float East_m = positionNED.y;
    float Down_m = positionNED.z;

    if(!isValidCoordinate(North_m,CoordinateType::North_m)){
        return false;
    }

    if(!isValidCoordinate(East_m,CoordinateType::East_m)){
        return false;
    }

    if(!isValidCoordinate(Down_m,CoordinateType::Down_m)){
        return false;
    }

    bool isValid = true;

    }

    

}


 

