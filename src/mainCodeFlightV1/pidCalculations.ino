

void pidAngles(){

  static unsigned long dtAngles = 1;
  
  float dtAnglesPID = (micros() - dtAngles)/1000000.0;
  if(dtAnglesPID == 0){
    Serial.println("ERROR DT!"); // smth went really wrong here
    return;
  }
  float outputAngleXpid = 0.0, outputAngleYpid = 0.0, outputAngleZpid = 0.0;
  float Xp = 0.0, Xd = 0.0, Yp = 0.0,  Yd = 0.0, Zp = 0.0, Zd = 0.0; 

  // coder changement angles
  float errorAngleX = AngleX - desiredAngleX;
  float errorAngleY = AngleY - desiredAngleY;
  float errorAngleZ = AngleZ - desiredAngleZ;

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
  static float errX_body_previous = 0.0;
  static float errY_body_previous = 0.0;
  static unsigned long dtPosition = 0;
  
  float dt = (micros() - dtPosition)/1000000.0;
  if(dtPosition == 0){
    
    return;
  }

  float psi = radians(AngleZ);  // heading in ENU with yaw=0 meaning North

  float errX_ENU = desiredPositionX - positionX;
  float errY_ENU = desiredPositionY - positionY;


  float errX_body =  cos(psi) * errX_ENU + sin(psi) * errY_ENU;
  float errY_body = -sin(psi) * errX_ENU + cos(psi) * errY_ENU;


   float pGainXY = 15; float iGainXY = 0.075; float dGainXY = 10.0;

   float pX = pGainXY*errX_body;
    integralX += iGainXY*errX_body*dt;
    float dX = dGainXY*(errX_body-errX_body_previous)/dt;


  float pY = pGainXY*errY_body;
  integralY += iGainXY*errY_body*dt;
  float dY = dGainXY*(errY_body-errY_body_previous)/dt;

  errX_body_previous = errX_body;
  errY_body_previous = errY_body;

 float outputX = pX + integralX + dX; 
  outputX = wrapper(outputX,maxAngleXYTVC*1.0);
  float outputY= pY + integralY + dY; 
  outputY = wrapper(outputY,maxAngleXYTVC*1.0);


  desiredAngleX = outputX;
  desiredAngleY = outputY;
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
  static unsigned long timeMOTOR= 1;
  if((millis()-timeMOTOR)*1.0>=1000.0/MOTORFREQUENCY){
    timeMOTOR = millis();
    pidMotor();
  }

}



void handleOrientationPID(){ // gets SERVOS data at appropriate rate , raises flag if NACK
  static unsigned long timeTrack= 1;
  
  const int SERVOSFREQUENCY = 100; // in Hz
  if((millis()-timeTrack)*1.0>=1000.0/SERVOSFREQUENCY){
    timeTrack = millis();
    pidAngles();
 
  }

}





void handlePositionPID(){ // gets MOTOR data at appropriate rate , raises flag if NACK
  const int POSITIONPIDFREQUENCY = 15; // in Hz
  static unsigned long timeTrack= 1;
  if((millis()-timeTrack)*1.0>=1000.0/POSITIONPIDFREQUENCY){
    timeTrack = millis();
    pidMotor();
  }

}









