#include "Telemetry_utilities.h"





// ============================================================
// DEBUG PAYLOAD
// ============================================================





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





String TelemetryUtilities::getFlightRegimeDataAsString(const FlightRegimeData& regime_data)
{
    String payload = "";
    addDebugGroup(payload, "Flight Regime Data : " + String(regime_data.name));
    addDebugGroup(payload, regime_data.epsilon_group);
    addDebugField(payload, "Is Flying", regime_data.isFlying);
    addDebugField(payload, "Name", regime_data.name);
    addDebugField(payload, "Max Pitch/Roll Velocity", regime_data.max_pitch_roll_velocity_degs);
    addDebugField(payload, "Max Yaw Velocity", regime_data.max_yaw_velocity_degs);
    addDebugField(payload, "Max Pitch/Roll Acceleration", regime_data.max_pitch_roll_acceleration_degs2);
    addDebugField(payload, "Max Yaw Acceleration", regime_data.max_yaw_acceleration_degs2);
    

    return payload;
}


void TelemetryUtilities::addDebugGroup(String& payload, const EpsilonGroup& epsilon_group)
{
    addDebugField(payload, "Epsilon Horizontal", epsilon_group.epsH);
    addDebugField(payload, "Epsilon Vertical", epsilon_group.epsV);
    addDebugField(payload, "Epsilon Yaw", epsilon_group.epsYaw);

   

}


void TelemetryUtilities::addTelemetryGroup(String& payload, const NED_coordinates& NED_payload)
{
    addTelemetryField(payload, NED_payload.North_SI, 2);
    addTelemetryField(payload, NED_payload.East_SI, 2);
    addTelemetryField(payload, NED_payload.Down_SI, 2);
}


void TelemetryUtilities::addTelemetryGroup(String& payload, const Rotation_Euler_coordinates& Euler_payload)
{
    addTelemetryField(payload, Euler_payload.Roll_SI, 2);
    addTelemetryField(payload, Euler_payload.Pitch_SI, 2);
    addTelemetryField(payload, Euler_payload.Yaw_SI, 2);
}

