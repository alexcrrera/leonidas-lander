#include "Telemetry_utilities.h"


void TelemetryUtilities::addDebugTitle(String& payload, const String& title)
{
    payload += "\n===== ";
    payload += title;
    payload += " =====\n";
}


void TelemetryUtilities::addDebugGroup(String& payload, const NED_coordinates& NED_payload, const String& name)
{
    addDebugGroup(payload, name);

    payload += "North: " + String(NED_payload.North_SI);
    payload += " | East: " + String(NED_payload.East_SI);
    payload += " | Down: " + String(NED_payload.Down_SI);
    payload += "\n";
}


void TelemetryUtilities::addDebugGroup(String& payload, const Rotation_Euler_coordinates& Euler_payload, const String& name)
{
    addDebugGroup(payload, name);

    payload += "Roll: " + String(Euler_payload.Roll_SI);
    payload += " | Pitch: " + String(Euler_payload.Pitch_SI);
    payload += " | Yaw: " + String(Euler_payload.Yaw_SI);
    payload += "\n";
}


void TelemetryUtilities::addDebugGroup(String& payload, const String& name)
{
    payload += "\n--- ";
    payload += name;
    payload += " ---\n";
}


void TelemetryUtilities::addDebugField(String& payload, const String& name, const String& value)
{
    payload += name;
    payload += ": ";
    payload += value;
    payload += "\n";
}


void TelemetryUtilities::addDebugField(String& payload, const String& name, float value)
{
    payload += name;
    payload += ": ";
    payload += String(value, 3);
    payload += "\n";
}


void TelemetryUtilities::addDebugField(String& payload, const String& name, bool value)
{
    payload += name;
    payload += ": ";
    payload += value ? "YES" : "NO";
    payload += "\n";
}


void TelemetryUtilities::addTelemetryGroup(String& payload, const NED_coordinates& NED_payload)
{
    addTelemetryField(payload, NED_payload.North_SI, 3);
    addTelemetryField(payload, NED_payload.East_SI, 3);
    addTelemetryField(payload, NED_payload.Down_SI, 3);
}


void TelemetryUtilities::addTelemetryGroup(String& payload, const Rotation_Euler_coordinates& Euler_payload)
{
    addTelemetryField(payload, Euler_payload.Roll_SI, 3);
    addTelemetryField(payload, Euler_payload.Pitch_SI, 3);
    addTelemetryField(payload, Euler_payload.Yaw_SI, 3);
}