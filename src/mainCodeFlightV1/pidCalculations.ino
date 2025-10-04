    void readPWM(){

 unsigned long pulseHigh = pulseIn(motorCtrlPin, HIGH);
  unsigned long pulseLow = pulseIn(motorCtrlPin, LOW);
  
  // Calculate the period and duty cycle
  unsigned long period = pulseHigh + pulseLow;
  float dutyCycle = (pulseHigh / (float)period) * 100.0;

  motorInputPWM = dutyCycle;
  //float mapped = map(pulseHigh,minPulse,maxPulse,0.0,100.0);
  // Print the measured values
  /*Serial.print("%: ");
  Serial.print(mapped);
  Serial.print("Pulse HIGH duration: ");
  Serial.print(pulseHigh);
  //Serial.print(" us, Pulse LOW duration: ");
  //Serial.print(pulseLow);
  //Serial.print(" us, Duty Cycle: ");
  //Serial.print(dutyCycle);
  Serial.println(" %");

  // Delay a bit before the next reading
*/

}

void pidAngles(){
  //readPWM();

  float dtAnglesPID = (micros() - dtAngles)/1000000.0;
  if(dtAnglesPID == 0){
    Serial.println("ERROR DT!"); // smth went really wrong here
    return;
  }
  float outputAngleXpid = 0.0, outputAngleYpid = 0.0, outputAngleZpid = 0.0;
  float Xp = 0.0, Xd = 0.0, Yp = 0.0,  Yd = 0.0, Zp = 0.0, Zd = 0.0; 

  // coder changement angles
  errorAngleX = AngleX - desiredAngleX;
  errorAngleY = AngleY - desiredAngleY;
  errorAngleZ = AngleZ - desiredAngleZ;

  Xp = pGainAngleX * errorAngleX;
  integralAngleX += iGainAngleX * dtAnglesPID * errorAngleX;
  Xd = dGainAngleX * ((errorAngleX - errorPreviousAngleX) / dtAnglesPID);

  Yp = pGainAngleY * errorAngleY;
  integralAngleY += iGainAngleY * dtAnglesPID * errorAngleY;
  Yd = dGainAngleY * ((errorAngleY - errorPreviousAngleY) / dtAnglesPID);

  Zp = pGainAngleZ * errorAngleZ;
  integralAngleZ += iGainAngleZ * dtAnglesPID*errorAngleZ;
  Zd = dGainAngleZ * ((errorAngleZ - errorPreviousAngleZ) / dtAnglesPID);


  integralAngleZ = wrapper(integralAngleZ,maxAngleZTVC); 
  integralAngleY =  wrapper(integralAngleY,maxAngleXYTVC); 
  integralAngleX =  wrapper(integralAngleX,maxAngleXYTVC);

  errorPreviousAngleX = errorAngleX; errorPreviousAngleY = errorAngleY; errorPreviousAngleZ = errorAngleZ;

  outputAngleXpid = Xp + integralAngleX + Xd;
   outputAngleYpid = Yp + integralAngleY + Yd; 
  

  outputAngleZpid = Zp +integralAngleZ + Zd+rollOffset;

  outputAngleXpid = wrapper(outputAngleXpid,maxAngleXYTVC);
  outputAngleYpid = wrapper(outputAngleYpid,maxAngleXYTVC);

  outputAngleZpid = wrapper(outputAngleZpid,maxAngleZTVC);

    //outputAngleZpid = (fabs(outputAngleZpid) > maxAngleZTVC) ? signeValeur(outputAngleZpid) * maxAngleZTVC : outputAngleZpid;
  //outputAngleYpid = 0.0;  outputAngleXpid = 0.0; // a degager

  //Serial.println(outputXpid);
  //outputAngleZpid = 0;

  finalOutputX1pid = outputAngleXpid + outputAngleZpid;
  finalOutputY1pid = -outputAngleYpid + outputAngleZpid;
  finalOutputX2pid = -outputAngleXpid + outputAngleZpid;
  finalOutputY2pid = outputAngleYpid + outputAngleZpid;

  finalOutputX1pid = round(finalOutputX1pid*10.0)/10.0; // resolution max 0.1º
  finalOutputY1pid = round(finalOutputY1pid*10.0)/10.0;
  finalOutputX2pid = round(finalOutputX2pid*10.0)/10.0;
  finalOutputY2pid = round(finalOutputY2pid*10.0)/10.0;

  finalOutputX1pid = wrapper(-finalOutputX1pid,tvcMaxAngle*1.0);
  finalOutputX2pid = wrapper(-finalOutputX2pid,tvcMaxAngle*1.0);
  finalOutputY1pid = wrapper(-finalOutputY1pid,tvcMaxAngle*1.0);
  finalOutputY2pid = wrapper(-finalOutputY2pid,tvcMaxAngle*1.0);
 dtAngles = micros();
 
}

void pidPosition(){
 /* errorPositionX = desiredPositionX - positionX;
  errorPositionY = desiredPositionY - positionY;
   float pGainXY = 1; float iGainXY = 0.02; float dGainXY = 0.5;

   float pX = pGainXY*errorPositionX;
    integralX += iGainXY*errorPositionX*dtPosition;
    float dX = 

*/
}

void pidMotor(){
  

  if(!MOTORON){
    percentageMotor = 0;
    return;
  }

  float pGainMotor = 11; float iGainMotor = 1.5; float dGainMotor = 7.0;

  float Mp = 0, Md = 0;
  errorM = desiredPositionZ - positionZ;
  if(buzzerOn && 1==2){

  int offsetFreq = 1000;
  int freq = round((errorM)*1000) + offsetFreq;

  tone(mainBuzzer, freq);

   tone(mainBuzzer, offsetFreq,0.2);
  }
  else{
    noTone(mainBuzzer);

  }
  float dtMotorPID = (micros() - dtMotor)/1000000.0;
  //Serial.print("\nTime: " + String(dtMotorPID));
  Mp = pGainMotor*errorM;
  integralMotor += iGainMotor * dtMotorPID * errorM;
  Md = dGainMotor * ((errorM - errorPreviousM) / dtMotorPID); 




  integralMotor =constrain(integralMotor,ESCOFFSET-MOTORMIN,MOTORMAX-ESCOFFSET);
  
 
  outputMpid = Mp + integralMotor + Md+ ESCOFFSET;
  
  finalOutputMpid = outputMpid;
    
  percentageMotor = constrain(finalOutputMpid,MOTORMIN,MOTORMAX);

  errorPreviousM = errorM;
 
  dtMotor = micros();
}



unsigned long timeMOTOR = 0;

void handleMotorPID(){ // gets MOTOR data at appropriate rate , raises flag if NACK
  const int MOTORFREQUENCY = 50; // in Hz
  if((millis()-timeMOTOR)*1.0>=1000.0/MOTORFREQUENCY){
    timeMOTOR = millis();
    pidMotor();
  }

}



unsigned long timeSERVOS = 0;

void handleServoPID(){ // gets SERVOS data at appropriate rate , raises flag if NACK
  const int SERVOSFREQUENCY = 100; // in Hz
  if((millis()-timeSERVOS)*1.0>=1000.0/SERVOSFREQUENCY){
    timeSERVOS = millis();
    pidAngles();
 
  }

}












