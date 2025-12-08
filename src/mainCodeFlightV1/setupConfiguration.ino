

void initSystem() {
 // delay(1000);
  
  pinMode(motorCtrlPin, INPUT);
    pinMode(LEDLR, OUTPUT);
  pinMode(LEDLG, OUTPUT);
  pinMode(LEDLB, OUTPUT);
   pinMode(switchMain, INPUT);
  pinMode(internal_LED_BUZZER, OUTPUT);
  initTelem();
  sendMessage("[OK] TELEM INIT");
 // delay(1000);
  sendMessage("TESTING SERVOS");

  initServos();
  sendMessage("[OK] SERVOS");
  initVectornav();
  sendMessage("[OK] IMU READY");
 // delay(500);
  sendMessage("CHECKING RTK");
  initRTK();

  sendMessage("INIT RTK TELEM");
  initTelemRTK();

  sendMessage("INIT LiDAR");
  initLidar();

  sendMessage("STARTUP COMPLETE");
  sendMessage("WAITING FOR SIGNAL");
 // delay(1000);
}

void initLidar(){
  myLidarLite.begin(0, true);  
  myLidarLite.configure(0);  
  int dist = myLidarLite.distance(true);
  if (!(dist<=0 and dist<=100)) {
    sendMessage("[OK] LiDAR CHECK");
  }
  else{
      sendMessage("[X] LiDAR ERROR");
  }
}




void initServos() {
  ESC.attach(ESCPIN,ESCLOWPOINT,ESCHIGHPOINT);//,,ESCHIGHPOINT);
  pinMode(servoX1,OUTPUT); pinMode(servoX2,OUTPUT); pinMode(servoY1,OUTPUT); pinMode(servoY2,OUTPUT);
  analogWriteResolution(resBit);
  analogWriteFrequency(servoX1,333);
  analogWriteFrequency(servoX2,333);
  analogWriteFrequency(servoY1,333);
  analogWriteFrequency(servoY2,333);

  ESC.writeMicroseconds(round(ESCLOWPOINT*1.1));
  delay(1000);
  Serial.println(F("\nSERVO GO"));
  servoVaneCheck();
}

void initTelem(){
  TELEM.begin(57600); // 57600
}



void initVectornav() {
  Serial.println("CALIBRATING VECTORNAV");
  Vectornav.begin(115200);  // Initialize the SoftwareSerial port
  delay(200);
  Vectornav.println("$VNWRG,6,1*XX"); // write to 6 the name of the registry - wtf?
  Vectornav.print("$VNWRG,7,200*XX");  //
}





void servoVaneCheck(){
  analogWrite(servoX1,servoCalculator(90.0+MissSX1));
  analogWrite(servoX2,servoCalculator(90.0+MissSX2));
  analogWrite(servoY1,servoCalculator(90.0+MissSY1));
  analogWrite(servoY2,servoCalculator(90.0+MissSY2));

  Serial.println("START SWEEP");

  for(int i = 0; i < 3; i++){
    analogWrite(servoX1,servoCalculator(90.0+MissSX1+tvcMaxAngle));
    analogWrite(servoX2,servoCalculator(90.0+MissSX2-tvcMaxAngle));
    analogWrite(servoY1,servoCalculator(90.0+MissSY1+tvcMaxAngle));
    analogWrite(servoY2,servoCalculator(90.0+MissSY2-tvcMaxAngle));

    delay(400);
    analogWrite(servoX1,servoCalculator(90.0+MissSX1-tvcMaxAngle));
    analogWrite(servoX2,servoCalculator(90.0+MissSX2+tvcMaxAngle));
    analogWrite(servoY1,servoCalculator(90.0+MissSY1-tvcMaxAngle));
    analogWrite(servoY2,servoCalculator(90.0+MissSY2+tvcMaxAngle));
    delay(400);

  }
  Serial.println("DONE SWEEP");
}
