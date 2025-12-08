unsigned long timeTELEMETRY = 0;


int dataIndexTelem = 0;
String incomingDataTelemString = "";

void handleTelemetry(){
  
  readTelem(); // read incoming bytes
   // in Hz
  if((micros()-timeTELEMETRY)*1.0>=1000.0*1000/TELEMETRYFREQUENCY){
    timeTELEMETRY = micros();
    sendTelem(); // send periodically telem
  }
}


void sendTelem(){
  if(!START_SYSTEM){
    return;
  }

  String output = "";

  output +="\n$LNDAS";
 
 // Indexes 2-4
  output += "," + String(AngleX,1) +  "," +String(AngleY,1) +  "," +String(AngleZ,1);
 // TELEM.print(output);

  // Indexes 5-7
  output += "," +String(positionX,2) + ","+String(positionY,2) + ","+String(positionZ,2);

  // Indexes 8-10
  output += "," +String(latitudeDeg,8) + ","+String(longitudeDeg,8) + ","+String(altitudeM,2);

  // Index 11
  output +="," + String(percentageMotor);
 // output += ",-1,-1,-1,";
   //TELEM.print(output);
  // Serial.print(output);

  // Index 12: motor status
    if(MOTORON){
  output += ",ON";
  }
  else{
    output += ",OFF";
  }

  // Index 13 
  if(MOTORARMED){
    output += ",ARMED";
  }
  else{
    output += ",DISARMED";
  }

  // Index 14: SD status
  if(sdWrite==1 || sdWrite ==2){
    output += ",NO SD REC";
  }
  else{
    output += ",NO SD";
  }

 

   // Indexes 15-17
     output += "," + String(desiredPositionX,2) + ","  +  String(desiredPositionY,2)  + ","  +String(desiredPositionZ,3);

  // Index 18: flight mode
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
  // Index 19: nº sat.
  output += "," +String(round(numSV),0);

  // Index 20: RTK fix mode
  output +="," + String(fixTypeText);

  // Index 21: heading
  output +="," + String(headingDeg,2);

  // Index 22: Gr Speed
  output += "," + String(gSpeed,2);

  // Index 23: Time UTC
  output +="," + gnssTimestamp;
 
  // Index 24: TIME MISSION
   output +="," + gnssTimestamp;
  // Index 25: RTK accuracy
   output +="," + String(averageRTK_accuracy,2);
    // Index 26: error message 
   output +=","+ errorMessage;

  
  // End of message
  output += "*";

  //Serial.print(output);
 //  Serial.print(output);
   TELEM.print(output);


  

  
}

void readTelem() { // returns read output int.
  if (TELEM.available() > 0) {
    char incomingChar = TELEM.read(); 
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
    Serial.println(F("RADIO OVERFLOW"));
    dataIndexTelem = 0;
    incomingDataTelemString = "";
  }
}



void sendMessage(String message){
  // Send individual message via telem (no Serial Studio Formatting)
  TELEM.print("\n");
  TELEM.print(message);
  TELEM.print("*");
}



int checkHeaderTelem(){
  
    int TelemIdentity = -1;
    if(incomingDataTelemString.indexOf("GO") != -1) {  // parameters
      Serial.println("GO");   
      START_SYSTEM = true;
      resetIntegralAngle();
      }

    if(incomingDataTelemString.indexOf("RANGLES") != -1) {  // parameters
      Serial.println("CONFIGU");   

      resetIntegralAngle();
      TelemIdentity = 1;      
    }
      
    if(incomingDataTelemString.indexOf("RPOS") != -1) { // reset position
      TelemIdentity = 2;
      Serial.println("RESET POSITION");
    }
    
    if(incomingDataTelemString.indexOf("PID") != -1) {
      TelemIdentity = 3; 
    }
    
    if(incomingDataTelemString.indexOf("ABORTANGLE") != -1) {
      TelemIdentity = 4;
    }
    
    if(incomingDataTelemString.indexOf("MAXANGLETVC") != -1) {
      TelemIdentity = 5;  
    }

    if(incomingDataTelemString.indexOf("RLIDAR") != -1) {
      TelemIdentity = 6;
    }

    if(incomingDataTelemString.indexOf("TLM") != -1) {
      TelemIdentity = 13;
    }

    if(incomingDataTelemString.indexOf("SDREC") != -1) {
      sdWrite = 2;    
    }
    
    if(incomingDataTelemString.indexOf("SDSTOP") != -1) {
      sdWrite = -1;
    }
    
    if(incomingDataTelemString.indexOf("$VN300") != -1) {
      Vectornav.println(incomingDataTelemString);  // Allows to send commands directly to the VN300 sensor via telem.
    }

    if(incomingDataTelemString.indexOf("TAO") != -1) {
      MOTORTEST = true;
      spoolMotor = true;  // flag
      timerSpoolMotor = 0; // reset time
      Serial.println("\nTAKE OFF COMMAND SENT");
    }

    if(incomingDataTelemString.indexOf("PLT") != -1) {
      PLOTMODE = !PLOTMODE;
      TelemIdentity = 69;
      }
      if (incomingDataTelemString.indexOf("LND") != -1) {
     
       
        forceLanding();
     
        // LAND NOW!
      }
      if (incomingDataTelemString.indexOf("MARM") != -1) {
        // Arm motor
        takeOff = false;
        MOTORARMED = true;
        Serial.println("MOTOR ARMED!");
          
      }

      if (incomingDataTelemString.indexOf("SETHOME") != -1) {
 //calculateOffsets(1);
//calculateOffsets(3);
        calculateOffsets(4);
        Serial.println("SET HOME!");
     
            }
      
      if (incomingDataTelemString.indexOf("MOFF") != -1) {
        takeOff = false;
        MOTORARMED = false;
        MOTORON = false;
        Serial.println("MOTOR OFF!");
   
      }



      if (incomingDataTelemString.indexOf("DSRM") != -1) {
        // Disarm motor
        MOTORARMED = false;
        Serial.println("MOTOR UNARMED!");
      }

       if (incomingDataTelemString.indexOf("MTEST") != -1) {
        // Test motor
        MOTORTEST = !MOTORTEST;
        Serial.println("MOTOR test!");
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
        

      case 69:
         commaParser.parseLine(incomingDataTelem,headerTelem,PLTRATE);
      if(PLTRATE==0){
        TELEM.println("TELEM OFF");
      }
  }



  calculateOffsets(offsetTool);
}






