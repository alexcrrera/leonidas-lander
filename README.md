
# Leonidas – Autonomous Guidance, Navigation & Control Stack


# UPDATE PENDING THIS IS OUTDATED!!!!!!!!!!!!

![alt text](https://github.com/alexcrrera/leonidas-lander/blob/main/side%20view.jpeg)
Latest image - Stator tube missing



![alt text](https://github.com/alexcrrera/leonidas-lander/blob/main/top%20view.jpeg)

## Latest Test - Altitude Hold Only - No GPS/2D Hold:

https://drive.google.com/file/d/1w0kUfSoaq_kFeYcmaZeCMRkhNgNvOfrM/view?usp=sharing

Leonidas is a high-performance embedded flight/navigation stack designed for autonomous unmanned vehicles.
It integrates VectorNav IMUs, u-blox RTK GNSS, LiDAR altimetry, sensor fusion, real-time control loops,
and robust telemetry into a modular Teensy-based architecture.

This repository contains all firmware modules (“.ino” files) organized by system function.
The code adheres to a fully functional architecture (no classes), high-rate control loops,
and reliability-focused sensor pipelines. It also includes the .json file needed for Serial Studio (used for the GUI).

------------------------------------------------------------
 SYSTEM ARCHITECTURE
------------------------------------------------------------
![alt text](https://github.com/alexcrrera/leonidas-lander/blob/main/notes/DIAGRAM.png)




------------------------------------------------------------
NAVIGATION SUBSYSTEMS
------------------------------------------------------------

VECTORNAV IMU
• Provides Yaw / Pitch / Roll using VN-300
• Computes angular velocity for derivative PID terms

LIDAR ALTIMETRY
• Converts LiDAR distance into vertical altitude using tilt correction:
      corrected = lidar / sqrt(tan²(roll) + tan²(pitch) + 1)
• Rejects invalid readings

GNSS / RTK POSITIONING
• Reads N/E/D relative coordinates
• Detects fix types (No Fix, 2D, 3D, DGNSS, RTK Float, RTK Fix)
• Detects accuracy degradation and signal timeouts

------------------------------------------------------------
CONTROL SUBSYSTEMS
------------------------------------------------------------

ATTITUDE CONTROL (PID)
• Roll, Pitch, Yaw loops
• Anti-windup on integrators
• dt-protected derivative terms
• Uses IMU attitude + gyro rates

POSITION CONTROL
• X, Y, and Z PID
• Heading-corrected frame transformation
• GNSS timeout fallbacks

ACTUATOR SYSTEM
• EDF thrust mapping
• Servo control
• Motor/EDF safety logic, arming checks, failure detection

------------------------------------------------------------
TELEMETRY SYSTEM
------------------------------------------------------------

• Sends: IMU, GNSS, RTK status, motor outputs, system health
• Receives: mode commands, remote control overrides



------------------------------------------------------------
GUI
------------------------------------------------------------

![Image](https://github.com/alexcrrera/leonidas-lander/blob/main/Base%20Station/Serial%20Studio%20Overview.png)


------------------------------------------------------------
INSTALLATION & USAGE
------------------------------------------------------------

REQUIREMENTS
• Teensy 4.1
• VectorNav VN-300
• u-blox ZED-F9P GNSS receiver
• LiDAR Lite v3 (Garmin)
• Arduino IDE + TeensyDuino

BUILD INSTRUCTIONS
1. Clone the repository:
       git clone https://github.com/<yourusername>/Leonidas.git

2. Open the folder in Arduino IDE.
3. Ensure that all .ino files are inside the same sketch directory.
4. Select Teensy 4.1 as the board.
5. Upload.

------------------------------------------------------------
LOGGING (SD CARD)
------------------------------------------------------------

• Stores GNSS logs, IMU logs, PID performance, battery, telemetry timestamps
• Data is used for post-flight analysis and debugging

------------------------------------------------------------
DEVELOPMENT PHILOSOPHY
------------------------------------------------------------

• Use modular functions instead of classes
• Use camelCase naming conventions
• Keep setup() and loop() minimal
• Prioritize reliability, readability, and deterministic timing

------------------------------------------------------------
FUTURE WORK
------------------------------------------------------------

• EKF-style sensor fusion
• Autonomous mission sequencing
• Obstacle avoidance
• VIO integration
• Simulation pipeline (Python + MATLAB)

------------------------------------------------------------
CONTACT
------------------------------------------------------------

For questions or contributions:
<alexcarrerav2@gmail.com>
