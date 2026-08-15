 
#include "../Utilities/Utilities.h"
#include "../Config/SafetyBounds.h"

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



    constexpr bool isValidCoordinate(float val, CoordinateType type){
        // checks if coordinate is within the specified range

        ParameterConfig coord_data = getData(type);

        if(Utilities::isBounded(val,coord_data.min, coord_data.max)){
            return true;
        }

        return false; // coordinate out of range

    }

    

constexpr bool isValidPositionNED(const Vector3& positionNED)
{
    return isValidCoordinate(positionNED.x, CoordinateType::North_m) &&
           isValidCoordinate(positionNED.y, CoordinateType::East_m) &&
           isValidCoordinate(positionNED.z, CoordinateType::Down_m);
}


    constexpr bool isValidPositionNED(const NED_coordinates& positionNED){
    // checks if NED positions is valid
    return isValidPositionNED(Vector3{positionNED.North_SI,positionNED.East_SI,positionNED.Down_SI});
    }
    

    

}


 

