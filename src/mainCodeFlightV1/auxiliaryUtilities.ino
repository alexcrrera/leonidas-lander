void handleInternalLED(){
    static uint32_t lastEventTime = 0;
    static bool isBeeping = false;

    uint32_t now = millis();

    if (!isBeeping) {
        // Wait for the 2s period
        if (now - lastEventTime >= 3000) {
            // Start beep
            isBeeping = true;
            lastEventTime = now;
            analogWrite(internal_LED_BUZZER, 21); // Max amplitude
            ledMainCtrl(3);
        }
    } 
    else {
        // Stop after 100 ms
        if (now - lastEventTime >= 100) {
            isBeeping = false;
            lastEventTime = now; // Reset timer for next 2s cycle
            analogWrite(internal_LED_BUZZER, 0); // Off
            ledMainCtrl(0);
        }
    }
}












void ledMainCtrl(int a) {
  switch (a) {
    case 0:  // Off
      digitalWrite(LEDLR, LOW);
      digitalWrite(LEDLG, LOW);
      digitalWrite(LEDLB, LOW);
      break;

    case 1:  // Red
      digitalWrite(LEDLR, HIGH);
      digitalWrite(LEDLG, LOW);
      digitalWrite(LEDLB, LOW);
      break;

    case 2:  // Green
      digitalWrite(LEDLR, LOW);
      digitalWrite(LEDLG, HIGH);
      digitalWrite(LEDLB, LOW);
      break;

    case 3:  // Blue
      digitalWrite(LEDLR, LOW);
      digitalWrite(LEDLG, LOW);
      digitalWrite(LEDLB, HIGH);
      break;
  }
}



void checkI2c() { // counts how many devices are connected
  while (!Serial)
    ;

  Wire.begin();

  Serial.println(F("Scanning I2C bus..."));
  int deviceCount = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.print("Total number of devices found: ");
  Serial.println(deviceCount);
  delay(2000);


  if (deviceCount != I2CDEVICES) {
    Serial.println(F("I2C ERROR!"));
    while (1) {
      delay(100);
      ledMainCtrl(0);

      delay(200);
      ledMainCtrl(1);
    }
  }
}



unsigned long timePrint = 0;

void handlePrint(){ // gets LiDAR data at appropriate rate , raises flag if NACK
  const int PRINTFREQUENCY = PLTRATE; // in Hz
  if((micros()-timePrint)*1.0>=1000.0*1000/PRINTFREQUENCY){
    //Serial.println("LETS");
    timePrint = micros();
    getPrint(); 
  }
}

void printSingle(String header, float d1){

  String res = header + ": ";
 


  if(PLOTMODE){
    res = "";
  }
  Serial.print(res +String(d1)+",");
}


void printGroup(String header, float d1, float d2, float d3){

  String res = header + ": ";

  if(PLOTMODE){
    res = "";
  }
  Serial.print(res  + String(d1) + "," + String(d2) + ","+ String(d3)+",");


}

void printGroup4(String header, float d1, float d2, float d3,float d4){

  String res = header + ": ";

  if(PLOTMODE){
    res = "";
  }
  Serial.print(res  + String(d1) + "," + String(d2) + ","+ String(d3)+ ","+ String(d3)+",");
 
}

void getPrint() {

  
     float timeNowSD = millis()/1000.0;
///////////////////////////////////////////////////////////  Serial.print("\n");
 ///////////////////////////////////////////////printSingle("Time",timeNowSD);
  //printBlender();
  //printGroup("Desired Position",desiredPositionX,desiredPositionY,desiredPositionZ);
  //printSingle("Desired Altitude",desiredPositionZ);
  //printSingle("Altitude",positionZ);
 
 // printGroup("Position",positionX,positionY,positionZ);
 
// //////////////////////////////////////////// printSingle("Motor", percentageMotor/100);
  /////////////////////// printSingle("Time S", timeTestTakeOff/1000.0);
 ////////////////////////////////////////////printGroup("x,y,z",posN,posE,posD);
 ////////////////////////////////// printSingle("RTK status",rtkStatus*1.0);
  
  //printSingle("Mi",Mi);


 //printGroup("LiDAR Readings", lidarReadings[0],lidarReadings[1],lidarReadings[2]);
  //printWarnings();
  //printSingle("AngleX",AngleX);
 
  //printSingle("Desired AngleX", desiredAngleX);
  //printSingle("Current AngleX", AngleX);
 ///////////////////////////////////////////// printGroup("Current Angles",AngleX,AngleY,AngleZ);
 //printGroup("Desired Angles",desiredAngleX,desiredAngleY,desiredAngleZ);

 //printGroup("PID",finalOutputX1pid,finalOutputY1pid,finalOutputX2pid);
   //printGroup("GAINS",pGainAngleX,iGainAngleX,dGainAngleX);
   // printGroup("INTEGRAL",integralAngleX,integralAngleY,integralAngleZ);
 //   printSingle("Motor PWM",motorInputPWM);
 //// printSingle("Motor Arm?",MOTORARMED);
    //printSingle("Take Off?",takeOff);
    
    //printSingle("Motor On",MOTORON);
    //printSingle("max TVCANGLEZ",maxAngleZTVC);
    //printSingle("max TVCANGLEZ",maxAngleZTVC);
  // printSingle("MAX ABORT ANGLE", abortAngle);
 // printGroup4("PID OUTPUT",finalOutputX1pid,finalOutputY1pid,finalOutputX2pid,finalOutputY2pid);
   // printSingle("Roll offsets",  rollOffset);
  
}


void printBlender(){
  //Serial.print("\n");
  ki++;

  if(ki>90){
    ki = 0;
  }
  //printGroup(-Zangle,-Yangle,-Xangle);
  //printGroup(0,0,lidarReadings[1]);
  Serial.print("0,0,");
  Serial.print(lidarReadings[1],3);
  Serial.print(",0,0,0;");
}


void printWarnings(){

  if(FLAGMOTOR){
    Serial.print("WARNING, MOTOR NOT ARMED!");
  }
}












