

void initSystem() {
  //initLidar();
   pinMode(motorCtrlPin, INPUT);
  initServos();
  initVectornav();
  initTelem();
  initRTK();
  pinMode(switchMain, INPUT);
  pinMode(mainBuzzer, OUTPUT);
  pinMode(LEDLR, OUTPUT);
  pinMode(LEDLG, OUTPUT);
  pinMode(LEDLB, OUTPUT);
  
  delay(1000);
}

void initLidar(){
  myLidarLite.begin(0, true);  
  myLidarLite.configure(0);  
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
  HC12.begin(57600); // 57600
}

void initVectornav() {
  Serial.println("CALIBRATING VECTORNAV");
  Vectornav.begin(115200);  // Initialize the SoftwareSerial port
  delay(1000);
  Vectornav.println("$VNWRG,6,1*XX"); // write to 6 the name of the registry - wtf?
  Vectornav.print("$VNWRG,7,200*XX");  //
}


void initRTK(){
  RTK.begin(57600);
}


void servoVaneCheck(){
  analogWrite(servoX1,servoCalculator(90.0));
  analogWrite(servoX2,servoCalculator(90.0));
  analogWrite(servoY1,servoCalculator(90.0));
  analogWrite(servoY2,servoCalculator(90.0));

  Serial.println("START SWEEP");

  for(int i = 0; i < 3; i++){

    analogWrite(servoX1,servoCalculator(90+tvcMaxAngle));
    analogWrite(servoX2,servoCalculator(90-tvcMaxAngle));
    analogWrite(servoY1,servoCalculator(90+tvcMaxAngle));
    analogWrite(servoY2,servoCalculator(90-tvcMaxAngle));
    delay(400);
    analogWrite(servoX1,servoCalculator(90-tvcMaxAngle));
    analogWrite(servoX2,servoCalculator(90+tvcMaxAngle));
    analogWrite(servoY1,servoCalculator(90-tvcMaxAngle));
    analogWrite(servoY2,servoCalculator(90+tvcMaxAngle));
    delay(400);

  }
  Serial.println("DONE SWEEP");
}
