#pragma once

#include "../Utilities/Utilities.h"

enum class EpsType {
    H,
    V,
    Yaw
};

namespace EpsConfig {

constexpr ParameterConfig<float> epsH = {
    .min = 0.025,
    .max = 1.0,
    .default_ = 0.15
};

constexpr ParameterConfig<float> epsV = {
    .min = 0.025,
    .max = 1.0,
    .default_ = 0.1
};

constexpr ParameterConfig<float> epsYaw = {
    .min = 0.5,
    .max = 999,
    .default_ = 1.0
};

constexpr const ParameterConfig<float>& getData(EpsType type) {

    switch (type) {
        case EpsType::H:
            return epsH;

        case EpsType::V:
            return epsV;

        case EpsType::Yaw:
            return epsYaw;

        default:
            return epsH;
    }
}

} // namespace EpsConfig


struct EpsilonGroup {

    float epsH = EpsConfig::epsH.default_;
    float epsV = EpsConfig::epsV.default_;
    float epsYaw = EpsConfig::epsYaw.default_;
};


namespace EpsConfig {

constexpr bool isValidEpsilon(float eps, EpsType type) {

    const auto& epsData = getData(type);

    return Utilities::isBounded(
        eps,
        epsData.min,
        epsData.max
    );
}

 constexpr bool isValidEpsilonGroup(
        const EpsilonGroup& epsGroup
    ) {

        if (!isValidEpsilon(epsGroup.epsH, EpsType::H)) {
            return false;
        }

        if (!isValidEpsilon(epsGroup.epsV, EpsType::V)) {
            return false;
        }

        if (!isValidEpsilon(epsGroup.epsYaw, EpsType::Yaw)) {
            return false;
        }

        return true;
    }

}