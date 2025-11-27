void initRTK(){
  RTK.begin(57600);
  myGNSS.begin(RTK, 1100, true);
    myGNSS.assumeAutoRELPOSNED(true, true);
}

void initTelemRTK(){
  TELEMRTK.begin(57600);
}


void convertRTK(){
  // converts NED into XYZ
  float heading = radians(AngleZ);

  positionX_RTK = cos(heading)*(posN-RTK_N_Offset) + sin(heading)*(posE-RTK_E_Offset);
  positionY_RTK = sin(heading)*(posN-RTK_N_Offset) - cos(heading)*(posE-RTK_E_Offset);
  positionZ_RTK = ((posD-RTK_D_Offset))*-1.0;
}


void handleRTK(){
  if (myGNSS.getRELPOSNED())   // returns true once a full RELPOSNED message is parsed
  {
    posN  = myGNSS.getRelPosN();
    posE = myGNSS.getRelPosE();
    posD  = myGNSS.getRelPosD();

   

    // Optional: also display accuracies
    accuracyN = myGNSS.getRelPosAccN();
    accuracyE = myGNSS.getRelPosAccE();
    accuracyD = myGNSS.getRelPosAccD();

    averageRTK_accuracy = (accuracyN +accuracyE+accuracyD)/3.0;

    convertRTK();

    //Serial.print("Acc N/E/D: ");
   // Serial.print(accuracyN); Serial.print(" / ");
   // Serial.print(accuracyE); Serial.print(" / ");
  //  Serial.println(accuracyD);
  }
  else
  {
    // small delay to avoid spamming loop
   
    // tiny heartbeat
    static int counter = 0;
    if (++counter > 1000000)
    {
      Serial.print(".");
      counter = 0;
    }
  }
}



void handleTelemRTK(){
  if( TELEMRTK.available()){
    byte received = TELEMRTK.read();
  
    RTK.write(received); // forward RTKTELEM

  }
}