
elapsedMillis  rampLandingTime = 0;

bool checkDistanceWaypoint(){
  return(calculateDistance()<=distanceEpsilon);
}



bool checkVerticalDistance(float eps){
   return(fabs(desiredPositionZ-positionZ)<=eps);
}


float calculateDistance(){
  double dis = pow(positionX-desiredPositionX,2) + pow(positionY-desiredPositionY,2)+pow(positionZ-desiredPositionZ,2);
  return(pow(dis,0.5));
}

bool offsetFound = false;

float getHover(){
  float ESCINCREASER =4.0;
  if(offsetFound || positionZ>= verticalTol){
    offsetFound = true;
    return(ESCOFFSET);
  }

  else{

    return(constrain(MOTORMIN+ESCINCREASER*rampLandingTime/1000.0,MOTORMIN,MOTORMAX));
  
}
}


void handleFlightMode(){
    if(takeOff){ // Requires restart to take off
       ESCOFFSET = getHover();
      rampLand();
  }
 
  if(takeOffCommand){
    if(MOTORARMED){
      //calculateOffsets(1);
      //calculateOffsets(3);
      //calculateOffsets(4);
      resetIntegralMotor();
      resetIntegralAngle();
      Serial.println("\n\nTAKE OFF;");
       offsetFound = false;
      ESCOFFSET = MOTORMIN;
      MOTORON = true;
      takeOff = true; 
      timerFlight = 0;
      takeOffCommand = false; // resets flag
      timeFlag = true;
      rampLandingTime = 0;
        
      return;
  }
  else{
    takeOffCommand = false;
    return;
    }
  }


  if(landingNow){ // Requires restart to take off

    if(landingFlag){
      landingFlag = false;
      rampLandingTime = 0;
      flightMode = false;
      
    takeOff = false;
    }
    rampLand();
    
  }


  if(flightMode){
    flightMode = true;
    takeOff = false;
    setDesiredParameters();
    flightProcedure();
    return;
  }
  setDesiredParameters();
  flightProcedure();

}


float rampTime = LANDINGSPEED/TAKEOFFALTITUDE; // in sec

void rampLand(){
  if(landingNow){
    tone(mainBuzzer, 500);
  desiredPositionX = 0.0; desiredPositionY = 0.0; 
  desiredPositionZ = TAKEOFFALTITUDE - LANDINGSPEED*rampLandingTime/1000.0 ;
  desiredPositionZ = constrain(desiredPositionZ,0,TAKEOFFALTITUDE);
  return;
  }
  if(takeOff){
    tone(mainBuzzer, 1000);
     desiredPositionX = 0.0; desiredPositionY = 0.0; 
    desiredPositionZ =  LANDINGSPEED*rampLandingTime/1000.0 ;
   desiredPositionZ = constrain(desiredPositionZ,0,TAKEOFFALTITUDE);
  return;
  }


}


void setDesiredParameters(){
    if(flightMode){
       noTone(mainBuzzer);
      getCoordinates();
    }
  

}

void flightProcedure(){


 
  if(checkVerticalDistance(verticalTol) && (takeOff && desiredPositionZ == TAKEOFFALTITUDE)){
    flightMode = true;
    takeOff = false;
  }

  if(checkVerticalDistance(verticalTol) && (landingNow && desiredPositionZ <= verticalTol)){
    landingNow = false;
    MOTORON = false;
    takeOff = false;
    Serial.println("LANDED");
    desiredPositionX = 0;
    desiredPositionY = 0;
    desiredPositionZ = 0;
    
    MOTORARMED = false;
  
  }
}




void getCoordinates(){
  desiredPositionX = 0;
  desiredPositionY = 0;
  desiredPositionZ = TAKEOFFALTITUDE;
}



