


void forceLanding(){
  landingFlag = true;
   timeFlag = false;
      landingNow = true;
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
  integralAngleX = 0.0;
  integralAngleZ = 0.0;

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




unsigned long timeStep = 0;
int timeCursor = 0;

float stepData[] = {0.0,5.0,0.0,-5.0,-2,3};
int stepSim = sizeof(stepData)/sizeof(stepData[0]); // 3 sec
float timeStepper = 2.0; // sec


float timeFlightCheck = 1;
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


void stepFunction(){
  if((millis()-timeStep)/1000.0 >= timeStepper){
    timeStep = millis();
    timeCursor++;
    timeCursor = (abs(timeCursor) > (stepSim -1)) ? 0 : timeCursor;
    
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
    
      break;

    case 4:
      positionXOffset = positionX;
      positionYOffset = positionY;
      positionZOffset = lidarReadings[3];//positionZ; as long as we're not using rtk.-..


    default:
      return;


  }

}


