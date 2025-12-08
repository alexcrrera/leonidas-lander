


void forceLanding(){
  landingFlag = true;
   timeFlag = false;
      landingNow = true;
      rampLandingTime = 0;
          Serial.println("\n LANDING NOW - TIMEOUT");
}

elapsedMillis  timeTestTakeOff = 0;
float timeFlight = 21.0; // in sec


void testHelper(){

  if((timeTestTakeOff/1000.0 >= timeFlight) && timeFlag){
  forceLanding();
    
  }
}



void resetIntegralAngle(){
  integralAngleX = 0.0;
  integralAngleY = 0.0;
  integralAngleZ = 0.0;
  integralX = 0.0;
  integralY = 0.0;

}

void resetIntegralMotor(){
  integralMotor = 0.0;
}



float EWA(float average, float newSample, float alpha){ // alpha parameter for EWA filter , alpha -> 1 means we lower the weight of the new sample
  return(average * (1-alpha) + alpha*newSample);
}






int signeValeur(float val){
  return(val<0 ? -1 : 1);
}






float timeFlightCheck = 1.0;
void spoolMotorCheck(){
  if(spoolMotor){
    if(timerSpoolMotor/1000.0 >=timeFlightCheck){
       timeTestTakeOff = 0;
      spoolMotor = false;
      MOTORTEST =false;
      takeOffCommand = true;
       // resta
    }
    
  
      
  }
}





float wrapper(float val, float max){
  float inter = (fabs(val) > max) ? signeValeur(val) * max : val;
  return(inter);
}




bool checkMaxAngle() {
  if (fabs(AngleX) > maxAbortAngle || fabs(AngleY) > maxAbortAngle) {
     takeOff = false;
        MOTORARMED = false;
        MOTORON = false;
        Serial.println("MOTOR OFF!");
    spoolMotor = false;
      MOTORTEST =false;
      takeOffCommand = true;
      return (false);
  } else {

    return (true);
  }
}





void calculateOffsets(int offsetTool){
  switch(offsetTool){
    case 1: // vectornav angles
      angleOffsetX = vectornavAngleX;
      angleOffsetY = vectornavAngleY;
      angleOffsetZ = vectornavAngleZ;
    break;

    case 2: // pressure reset
    
    break;

    case 3: // reset lidar 

      lidarReadings[3] = lidarReadings[1];
      lidarOffset = lidarReadings[3];
    
      break;

    case 4:

      RTK_N_Offset = posN;
      RTK_E_Offset = posE;
      RTK_D_Offset = posD;

      lidarReadings[3] = lidarReadings[1];
      lidarOffset = lidarReadings[3];


    default:
      return;


  }

}





float RTK_2_LiDAR_RATIO = 0.7;

void updateData(){
  if(vectornavAnglesUpdate){

    vectornavAnglesUpdate = false; 

    AngleX =  vectornavAngleX - angleOffsetX;
    AngleY =  vectornavAngleY - angleOffsetY;
    AngleZ =  vectornavAngleZ - angleOffsetZ;

  }

  if(RTK_ENABLED){
  positionX = positionX_RTK;
  positionY = positionY_RTK;
  positionZ = RTK_2_LiDAR_RATIO*(positionZ_RTK) + (1.0-RTK_2_LiDAR_RATIO)*(lidarReadings[2]);//0.95 +random(-10,10)/100.0;

  }
  else{
    positionX = desiredPositionX; // error equal to 0
    positionY = desiredPositionY;
    positionZ = lidarReadings[2];
   // errorMessage = "2D POS. LOST";
  }
}


