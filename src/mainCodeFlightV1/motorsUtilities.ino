
unsigned long timeServos = 0;

void handleServos(){ // gets LiDAR data at appropriate rate , raises flag if NACK
  const int SERVOFREQUENCY = 50; // in Hz
  if((micros()-timeServos)*1.0>=1000.0*1000/SERVOFREQUENCY){
    handleServoPID();
    timeServos = micros();
    servoWrite();
  }
}    


unsigned long timeEdf = 0;

void handleEDF(){ // gets LiDAR data at appropriate rate , raises flag if NACK
  const int EDFFREQUENCY = 50; // in Hz
  if((micros()-timeEdf)*1.0>=1000.0*1000/EDFFREQUENCY){
    timeEdf = micros();
    handleMotorPID();
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
  analogWrite(servoX1,servoCalculator(finalOutputX1pid+90.0+MissSX1));
  analogWrite(servoX2,servoCalculator(finalOutputX2pid+90.0+MissSX2));
  analogWrite(servoY1,servoCalculator(finalOutputY1pid+90.0+MissSY1));
  analogWrite(servoY2,servoCalculator(finalOutputY2pid+90.0+MissSY2));
  }
  else{ // TEST 
  analogWrite(servoX1,servoCalculator(90.0+MissSX1+tvcMaxAngle));
  analogWrite(servoX2,servoCalculator(90.0+MissSX2+tvcMaxAngle));
  analogWrite(servoY1,servoCalculator(90.0+MissSY1-tvcMaxAngle));
  analogWrite(servoY2,servoCalculator(90.0+MissSY2-tvcMaxAngle));
  }
}



bool isBluebird = false;
int servoCalculator(float val){

  int range = 0;
  float highTimeServo = 0.0;

  if(isBluebird){
    range = 60;
    highTimeServo = map(round(val*10),(90-range)*10,(90+range)*10,900,2100)/1000.0;
  }
  else{
    range = 50;
    highTimeServo = map(round(val*10),(90-range)*10,(90+range)*10,1000,2000)/1000.0;
  
  }

 
  float dutyCycleServo = highTimeServo/periodServo;
  int phi = round(dutyCycleServo*pow(2,resBit));
  
  return(phi);
}






