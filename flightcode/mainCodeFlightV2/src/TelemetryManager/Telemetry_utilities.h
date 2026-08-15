#pragma once

#include <Arduino.h>
#include <type_traits>

#include "../Utilities/Utilities.h"

namespace TelemetryUtilities
{
    void addDebugTitle(String& payload, const String& title);

    void addDebugGroup(String& payload, const String& name);
    void addDebugGroup(String& payload, const NED_coordinates& NED_payload, const String& name);
    void addDebugGroup(String& payload, const Rotation_Euler_coordinates& Euler_payload, const String& name);

    void addDebugField(String& payload, const String& name, const String& value);
    void addDebugField(String& payload, const String& name, float value);
    void addDebugField(String& payload, const String& name, bool value);

    template <typename T>
    inline void addTelemetryField(String& payload, T value, int decimals = -1)
    {
    if (payload.length() > 0)
        payload += ",";

    if constexpr (std::is_same_v<T, bool>)
        payload += value ? "[OK]" : "[X]";
    else if constexpr (std::is_same_v<T, String>)
        payload += value;
    else if (decimals >= 0)
        payload += String(value, decimals);
    else
        payload += String(value);
    }

    void addTelemetryGroup(String& payload, const NED_coordinates& NED_payload);
    void addTelemetryGroup(String& payload, const Rotation_Euler_coordinates& Euler_payload);
}