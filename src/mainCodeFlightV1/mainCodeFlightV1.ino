
//================================LEONIDAS 23/11/2025================================================
/*
 *   FIRST FLIGHT TEST - LEONIDAS - basic only vertical control removed SD stuff
 */

#define SERIAL_RX_BUFFER_SIZE_3 256

/////////////////////////////////////////////LIBRARIES/////////////////////////////////////////
#include <Wire.h>
#include <textparser.h> 
#include <string.h>
#include <LIDARLite.h>
#include <String.h>
#include <Servo.h>
#include <SPI.h>       // Include SPI library (needed for SD card)
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
//unsigned long timeSD =0;

bool START_SYSTEM = true;



/////////////////////////////////////////////LIBTOOLS/////////////////////////////////////////
#define LIDARLite_ADDR 0x62
#define I2CDEVICES 2

TextParser commaParser(",");  // Delimiter is a comma followed by a space.
LIDARLite myLidarLite;

#define TELEM Serial5 //1
#define TELEMRTK Serial4
#define RTK Serial3
#define Vectornav Serial8

SFE_UBLOX_GNSS myGNSS;

int TELEMETRYFREQUENCY = 10;


const float FREQUENCYCHECK = 0.1;
unsigned int CLKCOUNTER = 0;
unsigned long timeClck = 0;

bool flightMode = false;
bool landingNow = false;
bool takeOff = false;
bool takeOffCommand = false;
bool timeFlag = false;
bool spoolMotor = false;
bool landingFlag  = false;

bool FLAGMOTOR = false; bool MOTORARMED = false; bool MOTORON = false;

bool MOTORTEST = false;
bool SERVOTEST = false; // false = normal activity, true means 0º

elapsedMillis  rampLandingTime = 0;
elapsedMillis timerFlight;
elapsedMillis timerSpoolMotor;

String errorMessage = "NO ERRORS";

////////////////////////////////////// PIN DEFINITION ///////////////////////////////////////////////////////
const int switchMain = 32;  // check PCB
const int internal_LED_BUZZER = 13;   // =35;
const int LEDLR = 26; const int LEDLG = 25;  const int LEDLB = 24;
const int servoX1 = 2; const int servoX2 = 3; const int servoY1 = 4; const int servoY2 = 5;
const int motorCtrlPin = 16;

/////////////////////////////////////// POSITION PID //////////////////////////////////////////
float integralX = 0.0;
float integralY = 0.0;
float integralZ = 0.0;

/////////////////////////////////////// ANGLES //////////////////////////////////////////
float angleOffsetX = 0; float angleOffsetY = 0; float angleOffsetZ = 0;
float AngleX = 0, AngleY = 0, AngleZ = 0;
float desiredAngleX = 0, desiredAngleY = 0, desiredAngleZ = 0;
/////////////////////////////////////// VECTORNAV ///////////////////////////////////////////
float vectornavAngleX = 0; float vectornavAngleY = 0; float vectornavAngleZ = 0;
bool vectornavAnglesUpdate = true;


/////////////////////////////////////// RTK  ///////////////////////////////////////////
String  fixTypeText = "";
String gnssTimestamp = "";
int fixTypeRTK = 0; 
double latitudeDeg = 0;
double longitudeDeg = 0;
double altitudeM = 0;
uint8_t numSV = 0;
float gSpeed = 0;     // m/s
float headingDeg = 0;
float posN= 0.0; float posE= 0.0; float posD= 0.0;
float accuracyN = 0.0; float accuracyE = 0.0; float accuracyD = 0.0; float averageRTK_accuracy =0.0;
float positionX_RTK = 0; float positionY_RTK = 0; float positionZ_RTK = 0; 
float RTK_N_Offset = 0.0; float RTK_E_Offset = 0.0; float RTK_D_Offset = 0.0;
bool RTK_ENABLED = false; 
unsigned long  lastGnssUpdate = 0; // for GNSS time out
String RTK_error_message = "STARTUP";
/////////////////////////////////////// POSITION ///////////////////////////////////////////
float distanceEpsilon = 0.085;
float positionX = 0; float positionY = 0; float positionZ = 0;
float desiredPositionX = 0.0; float desiredPositionY = 0.0; float desiredPositionZ = 0.0;

/////////////////////////////////////// LiDAR ///////////////////////////////////////////
float lidarReadings[4] = {0.0,0.0,0.0,0.0};// first value, av val, normalised, offset
float lidarOffset = 0.0;

/////////////////////////////////////// PID //////////////////////////////////////////
float MpGain = 0.2; float MiGain = 0.2; float MdGain = 0.03;
float integralAngleX = 0; float integralAngleY = 0; float integralAngleZ = 0; float integralMotor = 0; // Integ error for pid motor
float errorPreviousAngleX, errorPreviousAngleY,errorPreviousAngleZ, errorPreviousM;
float finalOutputX1pid = 0, finalOutputY1pid = 0, finalOutputX2pid, finalOutputY2pid = 0; float finalOutputMpid=0;

float pGainAngleX = 1.0; float iGainAngleX = 0.1; float dGainAngleX =  0.15;
float pGainAngleY = 1.0; float iGainAngleY = 0.1; float dGainAngleY =  0.15;
float pGainAngleZ = 1.0; float iGainAngleZ = 0.1; float dGainAngleZ = 0.15;

float percentageMotor = 0;



int ki = 0; // blender stuff

/////////////////////////////////// FLIGHT STUFF ////////////////////////////////////////
float TAKEOFFALTITUDE = 0.75;
float LANDINGSPEED = 0.15;
float maxAbortAngle = 45.0;
float abortAngle = 45.0; // lol?

/////////////////////////////////// TVC ///////////////////////////////////////////////////////
float rollOffset = 0; // -3.0;//4.0;
float tvcMaxAngle = 12.0;

float maxAngleZTVC = 5.0;
float maxAngleXYTVC = tvcMaxAngle-maxAngleZTVC;

const int bufferSize = 80;  // Define the maximum length of the input string
//char incomingDataJavad[bufferSize];
char incomingDataTelem[bufferSize];

char incomingDataVectornav[bufferSize];       // Define the character array to store incoming data
                 // Index to keep track of where to store the next character


bool PLOTMODE = false;

/////////////////////////////////////////////TVC MOUNT/////////////////////////////////////////
int pidMulti = 1;
//13; // 15 max, 12,5 safe point, 13 to round
// servoX1.attach(29); servoY1.attach(36); servoX2.attach(33); servoY2.attach(37);
float MissSX1 =-1* 12.0;  // 5;
float MissSX2 = -1*3.4;  //5;
float MissSY1 = -1*1.0;  //10;
float MissSY2 = -1*-5.0;  //-5;


////////////////////////////////////////////SD CARD/////////////////////////////////////////
float SDFREQUENCY = 10.0;
//const int chipSelect = BUILTIN_SDCARD; // Built-in SD card reader
//File dataFile;
int sdWrite = -1; // not recording

////////////////////////////////////////////====================SETUP BEGIN====================/////////////////////////////////////////
float MOTORMAX =80; //%
float MOTORMIN =50;


int motorTestLaunch = 0;
unsigned long timeServo = 0;
float verticalTol= 0.05;
float ESCOFFSET = 70;

int PLTRATE = 25;
Servo ESC;
const int ESCPIN = 10;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int MIN_PULSE = 1000; // in microseconds
const int MAX_PULSE = 2000; // in microseconds
/////////////////////////////////////////////DEFINITIONS/////////////////////////////////////////

const int ANGLEDELTA = 100;
const int ESCLOWPOINT = 1000;
const int ESCHIGHPOINT = 2000;
const int SERVOLOWPOINT = 900;
const int SERVOHIGHPOINT = 2100;
const int SERVOFREQUENCY = 333;


const int resBit = 12;
float periodServo = 1000.0/(SERVOFREQUENCY*1.0);

const int BUFFERMESSAGESIZE = 128;
const int SYNCHMESSAGELENGHT = 2; // 2 CHARS for header is standard
const int SIZEOFLENGTH = 3; // 3 chars for hex
byte messageData[BUFFERMESSAGESIZE]; // stores bytes of the body
 


