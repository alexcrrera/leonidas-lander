unsigned long timeTELEMETRY = 0;


int dataIndexTelem = 0;
String incomingDataTelemString = "";

void handleTelemetry(){
   // in Hz
  if((micros()-timeTELEMETRY)*1.0>=1000.0*1000/TELEMETRYFREQUENCY){
    timeTELEMETRY = micros();
    sendTelem();
  }
}


void sendTelem(){

  String output = "";

  HC12.print("\n$LNDAS");
 // Serial.print("\n$LNDAS");
  output = "," + String(AngleX,1) +  "," +String(AngleY,1) +  "," +String(AngleZ,1);
  HC12.print(output);
 // Serial.print(output);
  output = "," +String(positionX) + ","+String(desiredPositionZ) + ","+String(positionZ) + ",48.864716,2.349014," +String(percentageMotor);
 // output += ",-1,-1,-1,";
   HC12.print(output);
  // Serial.print(output);
   output = "";
    if(buzzerOn){
  output += ",ON";
  }
  else{
    output += ",OFF";
  }
  if(MOTORARMED){
    output += ",ARMED";
  }
  else{
    output += ",DISARMED";
  }

  if(sdWrite==1 || sdWrite ==2){
    output += ",NO SD REC";
  }
  else{
    output += ",NO SD";
  }
    output += "," + String(ESCOFFSET,2);
   HC12.print(output);

   output = "";
  
   output += "," + String(vnGyro[0],5) + ","  + String(vnGyro[1],5) + "," + String(vnGyro[2],5);
     output += "," + String(desiredPositionX,2) + ","  +  String(desiredPositionY,2)  + ","  +String(desiredPositionZ,3);


    if(takeOff){
  output += ",TAKING OFF";
    }
  else{
    

    if(flightMode){
      output += ",FLIGHT MODE";
    }
    else{
      if(landingNow){
        output += ",LANDING NOW";
      }
      else{
        if(spoolMotor){
          output += ",SPOOLING UP";
        }
         output += ",STANDBY";
      }
    }
  }
  HC12.print(output);
   //Serial.print(output);
   HC12.print("*");
  // Serial.print("*");
  
}

void handleTelem() { // returns read output int.
  if (HC12.available() > 0) {
    char incomingChar = HC12.read(); 
   //Serial.print(incomingChar);
    if (incomingChar == '\n') {
      incomingDataTelem[dataIndexTelem] = '\0';
      processTelem();
      dataIndexTelem = 0;
    }

    else {
      incomingDataTelem[dataIndexTelem] = incomingChar;
      incomingDataTelemString=+incomingDataTelem;
      dataIndexTelem++;
      checkOverflowTelem();      
    }  
  }
  else{
      if (Serial.available() > 0) {
    char incomingChar = Serial.read(); 
   //Serial.print(incomingChar);
    if (incomingChar == '\n') {
      incomingDataTelem[dataIndexTelem] = '\0';
      processTelem();
      dataIndexTelem = 0;
    }

    else {
      incomingDataTelem[dataIndexTelem] = incomingChar;
      incomingDataTelemString=+incomingDataTelem;
      dataIndexTelem++;
      checkOverflowTelem();      
    }  
  } 

  }


}

void checkOverflowTelem(){
  if (dataIndexTelem >= bufferSize - 1) {
    Serial.println(F("overflowwww"));
    dataIndexTelem = 0;
    incomingDataTelemString = "";
    }
}

/*

bool flightMode = false;
bool landingNow = false;
bool takeOff = false;

*/
int checkHeaderTelem(){

  int TelemIdentity = -1;
  if (incomingDataTelemString.indexOf("RANGLES") != -1) {  // parameters
    Serial.println("CONFIGU");   
    resetIntegralAngle();
    TelemIdentity = 1;      
  }
      
  if (incomingDataTelemString.indexOf("RPOS") != -1) { // reset position
  TelemIdentity = 2;
  Serial.println("RESET POSITION");
  }
    
  if (incomingDataTelemString.indexOf("PID") != -1) {
    TelemIdentity = 3;
      }
  if (incomingDataTelemString.indexOf("ABORTANGLE") != -1) {
        TelemIdentity = 4;
        // LAND NOW!
          
      }
        if (incomingDataTelemString.indexOf("MAXANGLETVC") != -1) {
        TelemIdentity = 5;
        // LAND NOW!
          
      }

     if (incomingDataTelemString.indexOf("RLIDAR") != -1) {
           TelemIdentity = 6;
      
        // LAND NOW!
          
      }

     if (incomingDataTelemString.indexOf("DX") != -1) {
       resetIntegralAngle();
           TelemIdentity = 7;
      }
     if (incomingDataTelemString.indexOf("DY") != -1) {
           TelemIdentity = 8;
           resetIntegralAngle();
      }
     if (incomingDataTelemString.indexOf("DZ") != -1) {
          TelemIdentity = 9;
          resetIntegralAngle();
          }

      if (incomingDataTelemString.indexOf("PIDX") != -1) {
          TelemIdentity = 10;
          resetIntegralAngle();
      }
      if (incomingDataTelemString.indexOf("PIDY") != -1) {
           TelemIdentity = 11;
          resetIntegralAngle();
      }
                 if (incomingDataTelemString.indexOf("PIDZ") != -1) {
           TelemIdentity = 12;
          resetIntegralAngle();
      }

                      if (incomingDataTelemString.indexOf("TLM") != -1) {
           TelemIdentity = 13;
          
      }

        if (incomingDataTelemString.indexOf("ROLLOFF") != -1) {
           TelemIdentity = 14;
          
      }

       if (incomingDataTelemString.indexOf("TVCANGLEZ") != -1) {
           TelemIdentity = 15;
          
      }       if (incomingDataTelemString.indexOf("TVCANGLEXY") != -1) {
           TelemIdentity = 16;
          
      }

       if (incomingDataTelemString.indexOf("TSTEP") != -1) {
           TelemIdentity = 15;
          
      }

           if (incomingDataTelemString.indexOf("SDREC") != -1) {
            sdWrite = 2;
          
      }
           if (incomingDataTelemString.indexOf("SDSTOP") != -1) {
            sdWrite = -1;
          
      }
        if (incomingDataTelemString.indexOf("$VN300") != -1) {
          
        Vectornav.println(incomingDataTelemString);
        // LAND NOW!
          
      }
      if (incomingDataTelemString.indexOf("TAO") != -1) {
        MOTORTEST = true;
        spoolMotor = true;  // flag
        timerSpoolMotor = 0; // reset time
          Serial.println("\nTAKE OFF COMMAND SENT");
      }
 
      if (incomingDataTelemString.indexOf("Z+") != -1) {
     desiredAngleZ++;
      }
    
      if (incomingDataTelemString.indexOf("Z-") != -1) {
     desiredAngleZ--;
      }
       if (incomingDataTelemString.indexOf("O+") != -1) {
     rollOffset+=0.5;
      }
    
      if (incomingDataTelemString.indexOf("O-") != -1) {
     rollOffset+=-0.5;
      }

    
      if (incomingDataTelemString.indexOf("BTEST") != -1) {
        buzzerOn= !buzzerOn;
     
      }

      if (incomingDataTelemString.indexOf("PLT") != -1) {
        PLOTMODE = !PLOTMODE;
        TelemIdentity = 69;
      }
      if (incomingDataTelemString.indexOf("LND") != -1) {
     
       
        forceLanding();
     
        // LAND NOW!
      }
      if (incomingDataTelemString.indexOf("MARM") != -1) {
     
        takeOff = false;
        MOTORARMED = true;
        Serial.println("MOTOR ARMED!");
        // LAND NOW!
          
      }

      if (incomingDataTelemString.indexOf("SETHOME") != -1) {
 //calculateOffsets(1);
//calculateOffsets(3);
//calculateOffsets(4);
        Serial.println("SET HOME!");
        // LAND NOW!
            }
      
            if (incomingDataTelemString.indexOf("MOFF") != -1) {
        takeOff = false;
        MOTORARMED = false;
        MOTORON = false;
        Serial.println("MOTOR OFF!");
        // LAND NOW!
          
      }



      if (incomingDataTelemString.indexOf("DSRM") != -1) {
        
        MOTORARMED = false;
        
        Serial.println("MOTOR UNARMED!");
        // LAND NOW!
          
      }

 if (incomingDataTelemString.indexOf("MTEST") != -1) {
        
        MOTORTEST = !MOTORTEST;
        
        Serial.println("MOTOR test!");
        // LAND NOW!
          
      }


       if (incomingDataTelemString.indexOf("STEST") != -1) {
        
        SERVOTEST = !SERVOTEST;
        resetIntegralAngle();
        
        Serial.println("SERVO test!");
        // LAND NOW!
          
      }



  return(TelemIdentity); // none
}


void processTelem(){
  int TelemIdentity = checkHeaderTelem();

  char headerTelem[10];
  int offsetTool = -1;
float inter = 0.0;
  switch (TelemIdentity) {
    
    case -1:
    Serial.print("ERROR MESSAGE TELEM");
      return;
      break; // useless but meh
    case 1:
     offsetTool = 1;
      break;
    case 2:
      //ommaParser.parseLine(incomingDataTelem,headerTelem,Xangle,Yangle,Zangle);
      offsetTool = 4;
    case 3:

      break;
           case 4:
      //
      commaParser.parseLine(incomingDataTelem,headerTelem,abortAngle);
      //offsetTool = 4;

      break;
           case 5:
      //
      commaParser.parseLine(incomingDataTelem,headerTelem,tvcMaxAngle);
      //offsetTool = 4;

      break;

      case 6:
      offsetTool = 3;
    break;
      case 7:
       commaParser.parseLine(incomingDataTelem,headerTelem,desiredAngleX);
      break;
      case 8:
       commaParser.parseLine(incomingDataTelem,headerTelem,desiredAngleY);
      break;
            case 9:
       commaParser.parseLine(incomingDataTelem,headerTelem,desiredAngleZ);
      break;


        case 10:
         commaParser.parseLine(incomingDataTelem,headerTelem,pGainAngleX,iGainAngleX,dGainAngleX);
         break;
     case 11:
         commaParser.parseLine(incomingDataTelem,headerTelem,pGainAngleY,iGainAngleY,dGainAngleY);
         break;
             case 12:
         commaParser.parseLine(incomingDataTelem,headerTelem,pGainAngleZ,iGainAngleZ,dGainAngleZ);
         break;
          case 13:
             commaParser.parseLine(incomingDataTelem,headerTelem,TELEMETRYFREQUENCY);
         break;

            case 14:
             commaParser.parseLine(incomingDataTelem,headerTelem,rollOffset);
         break;

            case 15:
           
          
             commaParser.parseLine(incomingDataTelem,headerTelem,inter);

             maxAngleZTVC = (tvcMaxAngle -maxAngleXYTVC -  maxAngleZTVC < 0.0) ?  maxAngleZTVC : inter;

         break;
            case 16:

            
             commaParser.parseLine(incomingDataTelem,headerTelem,inter);

             maxAngleXYTVC = (tvcMaxAngle -maxAngleXYTVC -  maxAngleZTVC < 0.0) ?  maxAngleXYTVC : inter;

         break;            
        
        case 17:
             commaParser.parseLine(incomingDataTelem,headerTelem,timeStepper);
         break;

      case 69:
         commaParser.parseLine(incomingDataTelem,headerTelem,PLTRATE);
      if(PLTRATE==0){
        HC12.println("TELEM OFF");
      }
  }



  calculateOffsets(offsetTool);
}



