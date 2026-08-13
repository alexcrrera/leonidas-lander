#include "TelemetryManager.h"

#include "../FlightManager/FlightManager.h"


TelemetryManager::TelemetryManager(
    FlightManager* flightManager
)
    : flightManager(flightManager)
{
}


// ============================================================
// BEGIN
// ============================================================

void TelemetryManager::begin(SerialOutputType debugOutputType,SerialOutputType telemetryOutputType)
{

    send_USB_String("INIT START OF TELEM",true);

    if (CommsConfig::USB.enabled)
    {
        USB_output_type = debugOutputType;
        USB_output_initialized = true;
        USB_output_active = true;

        Serial.begin(CommsConfig::USB.baudrate);

        send_USB_String("INIT SERIAL OK",true);
    }


    if (CommsConfig::RADIO.enabled)
    {
        RADIO_output_type = telemetryOutputType;
        RADIO_output_active = true;
        CommsConfig::RADIO.port->begin(CommsConfig::RADIO.baudrate);
        RADIO_output_initialized = true;

        send_RADIO_String(get_telemetry_payload(),false);
    }
}


// ============================================================
// UPDATE
// ============================================================
void TelemetryManager::handleInput()
{

}


void TelemetryManager::update(uint32_t now)
{
    handleInput();


    // --------------------------------------------------------
    // USB
    // --------------------------------------------------------

    if (now - USB_last_output_time >=static_cast<uint32_t>(1000.0f / CommsConfig::USB.frequency)
    )
    {
        
        USB_last_output_time = now;


        if (isUSB_OutputEnabled())
        {
            if (
                USB_output_type ==
                SerialOutputType::HUMAN_READABLE
            )
            {
                send_USB_String(
                    get_debug_payload(),
                    false
                );
            }
            else
            {
                send_USB_String(
                    get_telemetry_payload(),
                    false
                );
            }
        }
    }


    // --------------------------------------------------------
    // RADIO
    // --------------------------------------------------------

    if (now - RADIO_last_output_time >=static_cast<uint32_t>(1000.0f / CommsConfig::RADIO.frequency))
    {
        RADIO_last_output_time = now;


        if (isRADIO_OutputEnabled())
        {
           
            if (
                RADIO_output_type ==
                SerialOutputType::HUMAN_READABLE
            )
            {
                
                send_RADIO_String(
                    get_debug_payload(),
                    false
                );
            }
            else
            {
                 
                send_RADIO_String(
                    get_telemetry_payload(),
                    false
                );
            }
        }
    }
}


// ============================================================
// TELEMETRY PAYLOAD
// ============================================================


String TelemetryManager::get_telemetry_payload()
{
    String payload;

    if (flightManager == nullptr)
        return "";

    const LanderSolution& solution = flightManager->getLander().getSolution();

    payload += CommsConfig::outputHeaderTelemetryFormat;

    // -------------------2-4--------------------------
    //addTelemetryGroup(payload, solution.state.position);
    // random data for testing
    addTelemetryField(payload, 69);
    addTelemetryField(payload, 420);
    addTelemetryField(payload, 1337);

    // -------------------5-7--------------------------
    addTelemetryGroup(payload, solution.state.velocity);

    // -------------------8-10--------------------------
    addTelemetryGroup(payload, solution.state.attitude);

    // -------------------11-13 ------- GPS DATA -----LATITUDE LONGITUDE ALTITUDE --------------
   // addTelemetryField(payload, solution.state.latitude, 6);
    addTelemetryField(payload, 48.8466, 6);
    addTelemetryField(payload, 2.35455, 6);
    ///addTelemetryField(payload, solution.state.longitude, 6);
    addTelemetryField(payload, solution.state.altitude, 2);

    // -------------------Sensor states - 14 - 17--------------------------
    addTelemetryField(payload, solution.validity.positionValid);
    addTelemetryField(payload, solution.validity.velocityValid);
    addTelemetryField(payload, solution.validity.accelerationValid);
    addTelemetryField(payload, solution.validity.altitudeValid);




    //addTelemetryField(payload, solution.state.altitude, 3);


    addTelemetryField(payload, solution.errors);
    addTelemetryField(payload, solution.timestamp);

    payload += CommsConfig::outputLineEndingTelemetryFormat;
    payload += "\n";

    return payload;
}

// ============================================================
// DEBUG PAYLOAD
// ============================================================


String TelemetryManager::get_debug_payload()
{
    String payload;

    if (flightManager == nullptr)
        return "FlightManager: NULL\n";

    Lander& lander = flightManager->getLander();
    const LanderSolution& solution = lander.getSolution();

    addDebugTitle(payload, "FLIGHT STATUS");

    // --------------------------------------------------------
    // Lander
    // --------------------------------------------------------

    addDebugGroup(payload, "LANDER");

    addDebugField(payload, "Position Valid", solution.validity.positionValid);
    addDebugField(payload, "Velocity Valid", solution.validity.velocityValid);
    addDebugField(payload, "Acceleration Valid", solution.validity.accelerationValid);
    addDebugField(payload, "Altitude Valid", solution.validity.altitudeValid);
    addDebugField(payload, "Attitude Valid", solution.validity.attitudeValid);

    // --------------------------------------------------------
    // Position
    // --------------------------------------------------------

    addDebugGroup(payload, solution.state.position, "POSITION NED");

    // --------------------------------------------------------
    // Velocity
    // --------------------------------------------------------

    addDebugGroup(payload, solution.state.velocity, "VELOCITY NED");

    // --------------------------------------------------------
    // Acceleration
    // --------------------------------------------------------

    addDebugGroup(payload, solution.state.acceleration, "ACCELERATION NED");

    // --------------------------------------------------------
    // Attitude
    // --------------------------------------------------------

    addDebugGroup(payload, solution.state.attitude, "ATTITUDE");

    // --------------------------------------------------------
    // Altitude
    // --------------------------------------------------------

    addDebugGroup(payload, "ALTITUDE");
    addDebugField(payload, "Altitude", solution.state.altitude);

    // --------------------------------------------------------
    // Solution
    // --------------------------------------------------------

    addDebugGroup(payload, "SOLUTION");
    addDebugField(payload, "Errors", String(solution.errors));
    addDebugField(payload, "Timestamp", String(solution.timestamp));

    payload += "\n";

    return payload;
}



// ============================================================
// DEBUG HELPERS
// ============================================================

void TelemetryManager::addDebugTitle(
    String& payload,
    const String& title
)
{
    payload += "\n===== ";
    payload += title;
    payload += " =====\n";
}


void TelemetryManager::addDebugGroup(String& payload, const NED_coordinates& NED_payload, const String& name)
{
    addDebugGroup(payload, name);

    payload += "North: " + String(NED_payload.North_SI);
    payload += " | East: " + String(NED_payload.East_SI);
    payload += " | Down: " + String(NED_payload.Down_SI);
    payload += "\n";
}

void TelemetryManager::addDebugGroup(String& payload, const Rotation_Euler_coordinates& Euler_payload, const String& name)
{
    addDebugGroup(payload, name);

    payload += "Roll: " + String(Euler_payload.Roll_SI);
    payload += " | Pitch: " + String(Euler_payload.Pitch_SI);
    payload += " | Yaw: " + String(Euler_payload.Yaw_SI);
    payload += "\n";
}


void TelemetryManager::addDebugGroup(
    String& payload,
    const String& name
)
{
    payload += "\n--- ";
    payload += name;
    payload += " ---\n";
}


void TelemetryManager::addDebugField(
    String& payload,
    const String& name,
    const String& value
)
{
    payload += name;
    payload += ": ";
    payload += value;
    payload += "\n";
}


void TelemetryManager::addDebugField(
    String& payload,
    const String& name,
    float value
)
{
    payload += name;
    payload += ": ";
    payload += String(value, 3);
    payload += "\n";
}


void TelemetryManager::addDebugField(
    String& payload,
    const String& name,
    bool value
)
{
    payload += name;
    payload += ": ";
    payload += value ? "YES" : "NO";
    payload += "\n";
}


// ============================================================
// RADIO OUTPUT
// ============================================================

void TelemetryManager::send_RADIO_String(
    const String& content,
    bool jumpToNextLine
)
{
    if (!isRADIO_OutputEnabled())
    {
        return;
    }


    CommsConfig::RADIO.port->print(
        content
    );


    if (jumpToNextLine)
    {
        CommsConfig::RADIO.port->print(
            "\n"
        );
    }
}


// ============================================================
// USB OUTPUT
// ============================================================

void TelemetryManager::send_USB_String(
    const String& content,
    bool jumpToNextLine
)
{

    if (!isUSB_OutputEnabled())
    {
        return;
    }


    Serial.print(content);


    if (jumpToNextLine)
    {
        Serial.print("\n");
    }
}





void TelemetryManager::addTelemetryGroup(String& payload, const NED_coordinates& NED_payload)
{
    addTelemetryField(payload, NED_payload.North_SI, 3);
    addTelemetryField(payload, NED_payload.East_SI, 3);
    addTelemetryField(payload, NED_payload.Down_SI, 3);
}

void TelemetryManager::addTelemetryGroup(String& payload, const Rotation_Euler_coordinates& Euler_payload)
{
    addTelemetryField(payload, Euler_payload.Roll_SI, 3);
    addTelemetryField(payload, Euler_payload.Pitch_SI, 3);
    addTelemetryField(payload, Euler_payload.Yaw_SI, 3);
}


// ============================================================
// OUTPUT CONTROL
// ============================================================

void TelemetryManager::toggle_RADIO(bool enable)
{
    if (!RADIO_output_initialized)
    {
        RADIO_output_active = false;
        return;
    }

    RADIO_output_active = enable;
}


void TelemetryManager::toggle_USB(bool enable)
{
    if (!USB_output_initialized)
    {
        USB_output_active = false;
        return;
    }

    USB_output_active = enable;
}





// ============================================================
// OUTPUT STATE
// ============================================================

bool TelemetryManager::isUSB_OutputEnabled()
{
    return
        USB_output_active &&
        USB_output_initialized &&
        CommsConfig::USB.enabled;
}


bool TelemetryManager::isRADIO_OutputEnabled()
{
    return
        RADIO_output_active &&
        RADIO_output_initialized &&
        CommsConfig::RADIO.enabled;
}