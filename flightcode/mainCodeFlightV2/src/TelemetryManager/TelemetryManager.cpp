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

     send_USB_String(
            "INIT    OK",
            true
        );
    if (CommsConfig::USB.enabled)
    {
        USB_output_type = debugOutputType;
        USB_output_initialized = true;
        USB_output_active = true;

        Serial.begin(CommsConfig::USB.baudrate);

        send_USB_String(
            "INIT SERIAL OK",
            true
        );
    }


    if (CommsConfig::RADIO.enabled)
    {
        RADIO_output_type = telemetryOutputType;
        RADIO_output_initialized = true;
        RADIO_output_active = true;

        CommsConfig::RADIO.port->begin(
            CommsConfig::RADIO.baudrate
        );

        send_RADIO_String(
            get_telemetry_payload(),
            false
        );
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
    {
        return "";
    }


    const LanderSolution& solution =
        flightManager->getLander().getSolution();


    payload +=
        CommsConfig::outputHeaderTelemetryFormat;


    // --------------------------------------------------------
    // Position NED
    // --------------------------------------------------------

    payload += String(
        solution.state.position.North_SI,
        3
    );

    payload += ",";

    payload += String(
        solution.state.position.East_SI,
        3
    );

    payload += ",";

    payload += String(
        solution.state.position.Down_SI,
        3
    );


    // --------------------------------------------------------
    // Velocity NED
    // --------------------------------------------------------

    payload += ",";

    payload += String(
        solution.state.velocity.North_SI,
        3
    );

    payload += ",";

    payload += String(
        solution.state.velocity.East_SI,
        3
    );

    payload += ",";

    payload += String(
        solution.state.velocity.Down_SI,
        3
    );


    // --------------------------------------------------------
    // Altitude
    // --------------------------------------------------------

    payload += ",";

    payload += String(
        solution.state.altitude,
        3
    );


    // --------------------------------------------------------
    // Attitude
    // --------------------------------------------------------

    payload += ",";

    payload += String(
        solution.state.attitude.Yaw_SI,
        3
    );

    payload += ",";

    payload += String(
        solution.state.attitude.Pitch_SI,
        3
    );

    payload += ",";

    payload += String(
        solution.state.attitude.Roll_SI,
        3
    );


    // --------------------------------------------------------
    // Errors
    // --------------------------------------------------------

    payload += ",";

    payload += String(
        solution.errors
    );


    // --------------------------------------------------------
    // Line ending
    // --------------------------------------------------------

    payload +=
        CommsConfig::outputLineEndingTelemetryFormat;


    return payload;
}


// ============================================================
// DEBUG PAYLOAD
// ============================================================



String TelemetryManager::get_debug_payload()
{
    String payload;

    if (flightManager == nullptr)
    {
        return "FlightManager: NULL\n";
    }


    Lander& lander =
        flightManager->getLander();

    const LanderSolution& solution =
        lander.getSolution();

    const OpticalFlowMeasurement opticalFlow =
        lander.getOpticalFlow().getMeasurement();


    addDebugTitle(
        payload,
        "FLIGHT STATUS"
    );




    // --------------------------------------------------------
    // Lander
    // --------------------------------------------------------

    addDebugGroup(
        payload,
        "LANDER"
    );


    addDebugField(
        payload,
        "Position Valid",
        solution.validity.positionValid
    );

    addDebugField(
        payload,
        "Velocity Valid",
        solution.validity.velocityValid
    );

    addDebugField(
        payload,
        "Acceleration Valid",
        solution.validity.accelerationValid
    );

    addDebugField(
        payload,
        "Altitude Valid",
        solution.validity.altitudeValid
    );

    addDebugField(
        payload,
        "Attitude Valid",
        solution.validity.attitudeValid
    );


    // --------------------------------------------------------
    // Position
    // --------------------------------------------------------

    addDebugGroup(
        payload,
        "POSITION NED"
    );

    addDebugField(
        payload,
        "North",
        solution.state.position.North_SI
    );

    addDebugField(
        payload,
        "East",
        solution.state.position.East_SI
    );

    addDebugField(
        payload,
        "Down",
        solution.state.position.Down_SI
    );


    // --------------------------------------------------------
    // Velocity
    // --------------------------------------------------------

    addDebugGroup(
        payload,
        "VELOCITY NED"
    );

    addDebugField(
        payload,
        "North",
        solution.state.velocity.North_SI
    );

    addDebugField(
        payload,
        "East",
        solution.state.velocity.East_SI
    );

    addDebugField(
        payload,
        "Down",
        solution.state.velocity.Down_SI
    );


    // --------------------------------------------------------
    // Acceleration
    // --------------------------------------------------------

    addDebugGroup(
        payload,
        "ACCELERATION NED"
    );

    addDebugField(
        payload,
        "North",
        solution.state.acceleration.North_SI
    );

    addDebugField(
        payload,
        "East",
        solution.state.acceleration.East_SI
    );

    addDebugField(
        payload,
        "Down",
        solution.state.acceleration.Down_SI
    );


    // --------------------------------------------------------
    // Attitude
    // --------------------------------------------------------

    addDebugGroup(
        payload,
        "ATTITUDE"
    );

    addDebugField(
        payload,
        "Yaw",
        solution.state.attitude.Yaw_SI
    );

    addDebugField(
        payload,
        "Pitch",
        solution.state.attitude.Pitch_SI
    );

    addDebugField(
        payload,
        "Roll",
        solution.state.attitude.Roll_SI
    );


    // --------------------------------------------------------
    // Altitude
    // --------------------------------------------------------

    addDebugGroup(
        payload,
        "ALTITUDE"
    );

    addDebugField(
        payload,
        "Altitude",
        solution.state.altitude
    );


    // --------------------------------------------------------
    // Solution
    // --------------------------------------------------------

    addDebugGroup(
        payload,
        "SOLUTION"
    );

    addDebugField(
        payload,
        "Errors",
        String(solution.errors)
    );

    addDebugField(
        payload,
        "Timestamp",
        String(solution.timestamp)
    );


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