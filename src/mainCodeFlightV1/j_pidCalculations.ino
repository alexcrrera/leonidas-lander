

void pidAngles(){

  static unsigned long dtAngles = 0;
  
  float dtAnglesPID = (micros() - dtAngles)/1000000.0;
  if(fabs(dtAnglesPID) <= 0.000001){
    Serial.println("ERROR DT!"); // smth went really wrong here
    return;
  }




  // coder changement angles
  float errorAngleX = AngleX - desiredAngleX;
  float errorAngleY = AngleY - desiredAngleY;
  float errorAngleZ = AngleZ - desiredAngleZ;

  float Xp = pGainAngleX * errorAngleX;
  integralAngleX += iGainAngleX * dtAnglesPID * errorAngleX;
  float Xd = dGainAngleX * ((errorAngleX - errorPreviousAngleX) / dtAnglesPID);

  float Yp = pGainAngleY * errorAngleY;
  integralAngleY += iGainAngleY * dtAnglesPID * errorAngleY;
  float Yd = dGainAngleY * ((errorAngleY - errorPreviousAngleY) / dtAnglesPID);

  float Zp = pGainAngleZ * errorAngleZ;
  integralAngleZ += iGainAngleZ * dtAnglesPID*errorAngleZ;
  float Zd = dGainAngleZ * ((errorAngleZ - errorPreviousAngleZ) / dtAnglesPID);


  integralAngleZ = wrapper(integralAngleZ,maxAngleZTVC); integralAngleY =  wrapper(integralAngleY,maxAngleXYTVC); integralAngleX =  wrapper(integralAngleX,maxAngleXYTVC);

  errorPreviousAngleX = errorAngleX; errorPreviousAngleY = errorAngleY; errorPreviousAngleZ = errorAngleZ;


  float outputAngleXpid_temp = Xp + integralAngleX + Xd;
  float outputAngleYpid_temp = Yp + integralAngleY + Yd; 
  float outputAngleZpid_temp = Zp +integralAngleZ + Zd+rollOffset;

  outputAngleXpid_temp = wrapper(outputAngleXpid_temp,maxAngleXYTVC);
  outputAngleYpid_temp = wrapper(outputAngleYpid_temp,maxAngleXYTVC);
  outputAngleZpid_temp = wrapper(outputAngleZpid_temp,maxAngleZTVC);


  float finalOutputX1pid_temp = outputAngleXpid_temp + outputAngleZpid_temp;
  float finalOutputY1pid_temp = -outputAngleYpid_temp + outputAngleZpid_temp;
  float finalOutputX2pid_temp = -outputAngleXpid_temp + outputAngleZpid_temp;
  float finalOutputY2pid_temp = outputAngleYpid_temp + outputAngleZpid_temp;

  finalOutputX1pid_temp = round(finalOutputX1pid_temp*10.0)/10.0; // resolution max 0.1º
  finalOutputY1pid_temp = round(finalOutputY1pid_temp*10.0)/10.0;
  finalOutputX2pid_temp = round(finalOutputX2pid_temp*10.0)/10.0;
  finalOutputY2pid_temp = round(finalOutputY2pid_temp*10.0)/10.0;

  finalOutputX1pid_temp = wrapper(-finalOutputX1pid_temp,tvcMaxAngle*1.0);
  finalOutputX2pid_temp = wrapper(-finalOutputX2pid_temp,tvcMaxAngle*1.0);
  finalOutputY1pid_temp = wrapper(-finalOutputY1pid_temp,tvcMaxAngle*1.0);
  finalOutputY2pid_temp = wrapper(-finalOutputY2pid_temp,tvcMaxAngle*1.0);

  finalOutputX1pid = EWA(finalOutputX1pid,finalOutputX1pid_temp,EWMA_PID_ORIENTATION_SERVO);
  finalOutputY1pid = EWA(finalOutputY1pid,finalOutputY1pid_temp,EWMA_PID_ORIENTATION_SERVO);
  finalOutputX2pid = EWA(finalOutputX2pid,finalOutputX2pid_temp,EWMA_PID_ORIENTATION_SERVO);
  finalOutputY2pid = EWA(finalOutputY2pid,finalOutputY2pid_temp,EWMA_PID_ORIENTATION_SERVO);
  




 dtAngles = micros();
 
}







void pidPosition(){
  // ======== Compute PID position output ===============
  static float errX_body_previous = 0.0;
  static float errY_body_previous = 0.0;
  static unsigned long time = 0;
  
  float dt = (micros() - time)/1000000.0;
  if(fabs(dt) <= 0.000001){
    return;
  }

  float psi = radians(AngleZ);  // heading in ENU with yaw=0 meaning North

  float errX_ENU = desiredPositionX - positionX;
  float errY_ENU = desiredPositionY - positionY;


  float errX_body =  cos(psi) * errX_ENU + sin(psi) * errY_ENU;
  float errY_body = -sin(psi) * errX_ENU + cos(psi) * errY_ENU;


  float pGainXY = 15; float iGainXY = 0.075; float dGainXY = 10.0;

  float pX = pGainXY*errX_body;
  integralPositionX += iGainXY*errX_body*dt;
  integralPositionX = wrapper(integralPositionX,maxAngleXYTVC*1.0);
  float dX = dGainXY*(errX_body-errX_body_previous)/dt;


  float pY = pGainXY*errY_body;
  integralPositionY += iGainXY*errY_body*dt;
  integralPositionY = wrapper(integralPositionY,maxAngleXYTVC*1.0);
  float dY = dGainXY*(errY_body-errY_body_previous)/dt;



  float outputX = pX + integralPositionX + dX; 
  outputX = wrapper(outputX,maxAngleXYTVC*1.0);

  float outputY= pY + integralPositionY + dY; 
  outputY = wrapper(outputY,maxAngleXYTVC*1.0);


  desiredAngleX = outputX;
  desiredAngleY = outputY;
  desiredAngleZ = 0.0;

  errX_body_previous = errX_body;
  errY_body_previous = errY_body;

  time = micros();
}

void pidMotor(){
  if(!MOTORON){
    percentageMotor = 0;
    return;
  }

  float pGainMotor = 11; float iGainMotor = 1.5; float dGainMotor = 7.0;

  float Mp = 0, Md = 0;
  float errorM = desiredPositionZ - positionZ;

  static unsigned long dtMotor = 1;

  float dtMotorPID = (micros() - dtMotor)/1000000.0;
  //Serial.print("\nTime: " + String(dtMotorPID));
  Mp = pGainMotor*errorM;
  integralMotor += iGainMotor * dtMotorPID * errorM;
  Md = dGainMotor * ((errorM - errorPreviousM) / dtMotorPID); 

  integralMotor =constrain(integralMotor,ESCOFFSET-MOTORMIN,MOTORMAX-ESCOFFSET);

  
  finalOutputMpid = Mp + integralMotor + Md+ ESCOFFSET;
    
  percentageMotor = constrain(finalOutputMpid,MOTORMIN,MOTORMAX);

  errorPreviousM = errorM;
 
  dtMotor = micros();
}



void handleMotorPID(){ // gets MOTOR data at appropriate rate , raises flag if NACK
  const int MOTORFREQUENCY = 50; // in Hz
  static unsigned long timeTrack= 1;
  if((millis()-timeTrack)*1.0>=1000.0/MOTORFREQUENCY){
    timeTrack = millis();
    pidMotor();
  }

}


void handleOrientationPID(){ // gets SERVOS data at appropriate rate , raises flag if NACK
  static unsigned long timeTrack= 1;
  
  const int SERVOSFREQUENCY = 50; // in Hz
  if((millis()-timeTrack)*1.0>=1000.0/SERVOSFREQUENCY){
    timeTrack = millis();
    pidAngles();
 
  }

}





void handlePositionPID(){ // gets MOTOR data at appropriate rate , raises flag if NACK
  const int POSITIONPIDFREQUENCY = 10; // in Hz
  static unsigned long timeTrack= 1;
  if((millis()-timeTrack)*1.0>=1000.0/POSITIONPIDFREQUENCY){
    timeTrack = millis();
    pidPosition();
  }

}









