


void updateData(){
  if(vectornavAnglesUpdate){
    float dtV = (micros()-vnTime)/1000.0/1000.0;

    vnTime = micros();

    vectornavAnglesUpdate = false; 
    desiredAngleX = 0.0;  //stepData[timeCursor];
    AngleX =  vectornavAngleX - angleOffsetX;
    AngleY =  vectornavAngleY - angleOffsetY;
    AngleZ =  vectornavAngleZ - angleOffsetZ;



    vnGyro[0] = (AngleX - vnOld[0])/dtV;
    vnOld[0] = AngleX;

    vnGyro[1] = (AngleY - vnOld[1])/dtV;
    vnOld[1] = AngleY;

    vnGyro[2] = (AngleZ - vnOld[2])/dtV;
    vnOld[2] = AngleZ;

  }
  positionX = posN - positionXOffset;
  positionY = posE - positionYOffset;
  positionZ = lidarReadings[2]-positionZOffset;//0.95 +random(-10,10)/100.0;
  //positionZ = posD - positionZOffset;

}








int dataIndexVectornav = 0;
String incomingDataVectornavString = "";




void handleVectornav() { // returns read output int.
  
  if (Vectornav.available() > 0) {
    char incomingChar = Vectornav.read(); 
   //Serial.print(incomingChar);
    if (incomingChar == '\n') {
      incomingDataVectornav[dataIndexVectornav] = '\0';
      processVectornav();
      dataIndexVectornav = 0;
    }

    else {
      incomingDataVectornav[dataIndexVectornav] = incomingChar;
      incomingDataVectornavString=+incomingDataVectornav;
      dataIndexVectornav++;
      checkOverflowVectornav();
           
    }
    
  } 

}

void checkOverflowVectornav(){
  if (dataIndexVectornav >= bufferSize - 1) {
    Serial.println(F("overflowwww"));
    dataIndexVectornav = 0;
    incomingDataVectornavString = "";
    }
}


int checkHeaderVectornav(){

      int vectornavIdentity = -1;
      if (incomingDataVectornavString.indexOf("$VNYPR") != -1) {  
        // Serial.println("VNYPR");        
        vectornavIdentity = 1;      
      }
      
      if (incomingDataVectornavString.indexOf("$VNIMU") != -1) {
        vectornavIdentity = 19;
      //  Serial.println("VNIMU");
      }

      if (incomingDataVectornavString.indexOf("$VNG2E") != -1) {
        vectornavIdentity = 33;
        // Serial.println("VNG2E");
      }

      if (incomingDataVectornavString.indexOf("$VNYIA") != -1) {
        vectornavIdentity = 240;
         // Serial.println("VNYIA");
      }


  return(vectornavIdentity); // none
}


void processVectornav(){
  int vectornavIdentity = checkHeaderVectornav();
  char headerVectornav[10];

  switch (vectornavIdentity) {
    case 1:
      commaParser.parseLine(incomingDataVectornav,headerVectornav,vectornavAngleX,vectornavAngleY,vectornavAngleZ);

      float inter = vectornavAngleZ;

      vectornavAngleZ = - vectornavAngleX;
      vectornavAngleX = - inter;
      vectornavAngleY *= -1;




      vectornavAnglesUpdate = true;
      updateData();
      break;
  }
}



