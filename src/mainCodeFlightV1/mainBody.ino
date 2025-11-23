


void setup() {
  
  Serial.begin(115200);
  Wire.begin();   
  Serial.println(F("STARTUP PROCEDURE"));
 
  initSystem();
  Serial.println(F("\nSENSORS GO"));


 // ledMainCtrl(3);
 // checkI2c();
}

////////////////////////////////////////////====================SETUP END & MAIN LOOP START====================/////////////////////////////////////////

void loop() {

//`&if((micros()-timeClck)*1.0>=1000.0*1000/FREQUENCYCHECK){
  //   timeClck = micros();
    //Serial.print("\nFreq: ");
   // Serial.print(CLKCOUNTER/FREQUENCYCHECK);
    //CLKCOUNTER = 0;
  //}
  spoolMotorCheck();
 // CLKCOUNTER++;

  handleVectornav();
 // handleLidar();
  handleTelemetry();
  handleTelem();
 // updateData();
  
  handleFlightMode();


  handleEDF();
  handleServos();

  handlePrint();


  testHelper();
  checkMaxAngle();


 // handleInternalLED();
  //handleRTK();
 // handleSD();
 /* handleReceiver();
 
checkMaxAngle
  handleMotor();
  handleCalculations();
  handlePID();
  handleEDF();
  
  handleAuxiliary();
*/

}

////////////////////////////////////////////====================MAIN LOOP END====================/////////////////////////////////////////

