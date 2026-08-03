#include <Arduino.h>
#include <Wire.h>
#include <Handler.h>

#include "src/Sensor/Sensor.h"
#include "src/VN300/VN300.h"
#include "src/Lidar/Lidar.h"
#include "src/Lander/Lander.h"

// ============================================================
// Serial ports
// ============================================================

#define TELEM Serial5
#define TELEMRTK Serial4
#define RTK Serial3
#define Vectornav Serial8

// ============================================================
// Sensors
// ============================================================

VN300 vn300(Vectornav);
Lidar lidar(Wire);

// ============================================================
// Lander
// ============================================================

Lander lander(vn300, lidar);

// ============================================================
// Handlers
// ============================================================

Handler printHandler;

// ============================================================
// Handler functions
// ============================================================

void printLander()
{
    lander.printStatus(Serial);
}

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    Vectornav.begin(115200);
    TELEM.begin(115200);
    TELEMRTK.begin(115200);
    RTK.begin(115200);

    Wire.begin();

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("       LANDER OOP TEST");
    Serial.println("================================");

    Serial.println();
    Serial.println("Initializing VN300...");

    if (vn300.begin())
    {
        Serial.println("VN300 initialized");
    }
    else
    {
        Serial.println("VN300 initialization failed");
    }

    Serial.println();
    Serial.println("Initializing LiDAR...");

    if (lidar.begin())
    {
        Serial.println("LiDAR initialized");
    }
    else
    {
        Serial.println("LiDAR initialization failed");
    }

    // Print lander state at 2 Hz
    printHandler.begin(10.0, printLander);

    Serial.println();
    Serial.println("Starting Lander");
}

// ============================================================
// Loop
// ============================================================

void loop()
{
    // Poll sensors and update the lander solution
    LEONIDAS.update();

    // Handle scheduled operations
    printHandler.handle();
}