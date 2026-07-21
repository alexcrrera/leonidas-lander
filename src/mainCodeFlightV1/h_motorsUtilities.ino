

void handleServos(){ // gets LiDAR data at appropriate rate , raises flag if NACK
  // uses the global SERVOFREQUENCY from a_userParameters.ino
  static unsigned long time = 0;
  if((micros()-time)*1.0>=1000.0*1000/SERVOFREQUENCY){
    
    time = micros();
    servoWrite();
  }
}    



void handleEDF(){ // gets LiDAR data at appropriate rate , raises flag if NACK
  const int EDFFREQUENCY = 25; // in Hz
  static unsigned long timeEdf = 0;

  if((micros()-timeEdf)*1.0>=1000.0*1000/EDFFREQUENCY){
    timeEdf = micros();
    handleThrustEDF();
  }
}


void handleThrustEDF(){

 if(!MOTORON){
    percentageMotor = 0.0;
  }

   if(MOTORTEST ){
    percentageMotor = 21.0;
  }

  if(MOTORTEST && spoolMotor){
    percentageMotor = 21.0;
  }
  const int outFinal = round(map(percentageMotor,0.0,100.0,ESCLOWPOINT,ESCHIGHPOINT));
  ESC.writeMicroseconds(outFinal);
}

void servoWrite() {
  if(!SERVOTEST){
  analogWrite(servoX1,servoCalculator(finalOutputX1pid+MissSX1));
  analogWrite(servoX2,servoCalculator(finalOutputX2pid+MissSX2));
  analogWrite(servoY1,servoCalculator(finalOutputY1pid+MissSY1));
  analogWrite(servoY2,servoCalculator(finalOutputY2pid+MissSY2));
  }
  else{ // TEST 
  analogWrite(servoX1,servoCalculator(MissSX1+tvcMaxAngle));
  analogWrite(servoX2,servoCalculator(MissSX2+tvcMaxAngle));
  analogWrite(servoY1,servoCalculator(MissSY1-tvcMaxAngle));
    
  }
}



// isBluebird moved to a_userParameters.ino (USER PARAMETER)
int servoCalculator2(float val){

  int multiFactor = 10; // 1> keep int part , 10 > keep decimal part

  int range = 0;
  float highTimeServo = 0.0;

  if(isBluebird){
    range = 60;
    highTimeServo = map(round(val*multiFactor),(90-range)*multiFactor,(90+range)*multiFactor,900,2100)/1000.0;
  }
  else{
    range = 50;
    highTimeServo = map(round(val*multiFactor),(90-range)*multiFactor,(90+range)*multiFactor,1000,2000)/1000.0;
  }

  float dutyCycleServo = highTimeServo/periodServo;
  int phi = round(dutyCycleServo*pow(2,resBit)); // 12 bit
  
  return(phi);
}



// CM509MG — angle (deg, 1 decimal) > PWM value for analogWrite
// 0° = center = 1500 µs
// Range: [-50.0 ; +50.0]
// PWM: 333 Hz, configurable resolution











// ================= SERVO CONFIGURATION =================
// Calibration constants (servoMinAngleDeg, servoMaxAngleDeg, servoMinPulseUs,
// servoCenterPulseUs, servoMaxPulseUs) moved to a_userParameters.ino (USER PARAMETERS).

uint32_t servoCalculator(float angleDeg)
{
    if (angleDeg < servoMinAngleDeg) angleDeg = servoMinAngleDeg;
    if (angleDeg > servoMaxAngleDeg) angleDeg = servoMaxAngleDeg;

    const float angleSpanDeg =
        servoMaxAngleDeg - servoMinAngleDeg;

    const float pulseSpanUs =
        servoMaxPulseUs - servoMinPulseUs;

    const float normalized =
        (angleDeg - servoMinAngleDeg) / angleSpanDeg;

    const float pulseUs =
        servoMinPulseUs + normalized * pulseSpanUs;

    const float pwmPeriodUs =
        1e6f / SERVOFREQUENCY;

    const uint32_t pwmMaxValue =
        (1UL << resBit) - 1;

    return (uint32_t)((pulseUs / pwmPeriodUs) * pwmMaxValue);
}



