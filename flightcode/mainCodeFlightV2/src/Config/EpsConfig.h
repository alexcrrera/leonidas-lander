#include "Utilities.h"

enum class EpsType {
    H,
    V,
    Yaw
};


struct EpsilonGroup { // epsilon group
    float epsH = EpsilonGroup::epsH.default_; // default value
    float epsV = EpsilonGroup::epsV.default_;
    float epsYaw=EpsilonGroup::epsYaw.default_;
}


 namespace EpsConfig {



    constexpr ParameterConfig<float> epsH = { // Boundings for tolerances for HORIZONTAL NED positioning
        .min = 0.025,
        .max = 1.0,
        .default_ = 0.15
    };

    constexpr ParameterConfig<float> epsV = { // Boundings for tolerances for VERTICAL NED positioning
        .min = 0.025,
        .max = 1.0,
        .default_ = 0.1
    };

     constexpr ParameterConfig<float> epsYaw = { // Boundings for tolerances for YAW alignment
        .min = 0.5,
        .max = 15.0,
        .default_ = 1
    };






    // ============= HELPER FUNCTIONS NO PARAMS HERE =============
    constexpr const ParameterConfig<float>& getData(EpsType type){

        switch (type)
        {
        case EpsType::H:
            return(epsH);
        case EpsType::V:
            return(epsV);
        case EpsType::Yaw:
            return(epsYaw);
        
        default:
             return(epsH); // safeguard, should never happen
        }
    }

bool isValidEpsilonGroup(EpsilonGroup eps_group){
    if(!isValidEpsilon(eps_group.epsH,EpsType::H)){
        return(false);
    }
    
    if(!isValidEpsilon(eps_group.epsV,EpsType::V)){
        return(false);
    }
     if(!isValidEpsilon(eps_group.epsYaw,EpsType::Yaw)){
        return(false);
    }

    return(true);
}


 bool isValidEpsilon(float eps, EpsType type){
    // checks if epsilon is within the specified range
    ParameterConfig<float> eps_data = EpsConfig::getData(type);

    if(Utilities::isBounded(eps,eps_data.min, eps_data.max)){
        return true;
    }

    return false; // epsilon out of range

 }

 }


