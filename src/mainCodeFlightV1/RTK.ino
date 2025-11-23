#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS gnss;   // Library object

void setup() {
  Serial.begin(115200);

  Wire1.begin();                // Use Wire1 (Teensy secondary I2C)
  Wire1.setClock(400000);

  // Connect to the GNSS on Wire1
  if (gnss.begin(Wire1) == false) {
    Serial.println("GNSS not detected.");
    while (1);
  }

  Serial.println("GNSS connected ✔");

  // -------------------------------
  // Configure UART1 (PORTID = 1)
  // -------------------------------
  // UBX input only  → Bit0 = UBX
  // UBX output only → Bit0 = UBX
  // Baud = 57600

  bool success = gnss.setPortMode(
      COM_PORT_UART1,
      COM_TYPE_UBX,      // input protocols
      COM_TYPE_UBX       // output protocols
  );

  if (!success) {
    Serial.println("Failed to set PortMode ❌");
  } else {
    Serial.println("PortMode set ✔");
  }

  success = gnss.setUART1Baudrate(57600);
  if (!success) {
    Serial.println("Failed to set UART1 baud ❌");
  } else {
    Serial.println("UART1 = 57600 ✔");
  }

  Serial.println("Configuration finished.");
}

void loop() {
  // Put anything you want here
}
