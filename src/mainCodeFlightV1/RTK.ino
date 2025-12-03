void initRTK(){
  RTK.begin(57600);
  myGNSS.begin(RTK, 1100, true);
  myGNSS.assumeAutoRELPOSNED(true, true);
  myGNSS.assumeAutoPVT(true);       // Tell library to expect it automatically
}

void initTelemRTK(){
  TELEMRTK.begin(57600);
}


void convertRTK(){
  // converts NED into XYZ


  // NED to ENU conversion
  positionX_RTK = 1.0*(posE-RTK_E_Offset) ;
  positionY_RTK = 1.0*(posN-RTK_N_Offset);
  positionZ_RTK = ((posD -RTK_D_Offset))*-1.0;

}




void checkRTKstatus(){

  if((millis() - lastGnssUpdate) > 1000){
   // RTK_ENABLED = false;  
    errorMessage = "TIMEOUT";
    return;
    }

  if(averageRTK_accuracy >= 0.5){
    RTK_ENABLED = false;
    errorMessage = "ACCURACY TOO LOW";
    return;
  }


  if(fixTypeRTK != 1 && fixTypeRTK != 2){
    RTK_ENABLED = false;
    errorMessage = "NOT IN FLOAT OR FIX MODE";
     return;
  
  }

  

  if(averageRTK_accuracy<0.0001){
    RTK_ENABLED = false;
    errorMessage = "ACC IS 0";
     return;
  }
    

  if(fabs(posN)<0.00001 || fabs(posE)<0.00001){
    RTK_ENABLED = false;
    errorMessage = "POS IS 0";
     return;
  }

  errorMessage = "NOMINAL";
  RTK_ENABLED = true;
}


void handleRTK() {
  
  // --------------------------
  // RELPOSNED (RTK)
  // --------------------------
  if (myGNSS.getRELPOSNED())
  {
    lastGnssUpdate = millis();
    posN = myGNSS.getRelPosN();

    posE = myGNSS.getRelPosE();
    posD = myGNSS.getRelPosD();



    accuracyN = myGNSS.getRelPosAccN();
    accuracyE = myGNSS.getRelPosAccE();
    accuracyD = myGNSS.getRelPosAccD();

    averageRTK_accuracy = (accuracyN + accuracyE) / 2.0;

    convertRTK();

  }

  // --------------------------
  // NAV-PVT (Fix type, position, velocity)
  // --------------------------
  if (myGNSS.getPVT())   // new PVT message arrived
  {
    // Fix type
    fixTypeRTK = myGNSS.getFixType();
    fixTypeRTK = myGNSS.getCarrierSolutionType();

    
    fixTypeText =fixTypeToText(fixTypeRTK);
    


    // Position (degrees)
    latitudeDeg  = 1.0*myGNSS.getLatitude()  / 1e7;  // convert int -> degrees
    longitudeDeg = 1.0*myGNSS.getLongitude() / 1e7;

    // Altitude
    altitudeM = myGNSS.getAltitudeMSL() / 1000.0; // mm -> meters

    // Number of satellites used
    numSV = myGNSS.getSIV();

    // Velocity
    gSpeed = myGNSS.getGroundSpeed() / 1000.0;  // mm/s -> m/s
    headingDeg = myGNSS.getHeading() / 1e5;     // heading in degrees
    int year   = myGNSS.getYear();
    int month  = myGNSS.getMonth();
    int day    = myGNSS.getDay();
    int hour   = myGNSS.getHour();
    int minute = myGNSS.getMinute();
    int second = myGNSS.getSecond();

    char buffer[26];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
          year, month, day, hour, minute, second);

   gnssTimestamp = String(buffer);

  }


  checkRTKstatus(); // CHECK THAT RTK IS OK

}

const char* fixTypeToText (int flagRTK){
    /*  switch (fixType)
    {
        case 0: return "NO FIX";
        case 1: return "DEAD RECKONING";
        case 2: return "2D FIX";
        case 3: return "3D FIX";
        case 4: return "GNSS + DEAD RECKONING";
        case 5: return "TIME FIX";
        default: return "UNKNOWN";
    }*/
    switch (flagRTK)
    {
      case 1: return "RTK FLOAT";
      case 2: return "RTK FIXED";
      default: return "NO RTK";
    }

}

void handleTelemRTK(){
  if( TELEMRTK.available()){
    byte received = TELEMRTK.read();
  
    RTK.write(received); // forward RTKTELEM

  }
}