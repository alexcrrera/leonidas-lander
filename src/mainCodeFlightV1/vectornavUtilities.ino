


void updateData(){

  AngleX =  vectornavAngleX - angleOffsetX;
  AngleY =  vectornavAngleY - angleOffsetY;
  AngleZ =  vectornavAngleZ - angleOffsetZ;
  
  positionX = posN - positionXOffset;
  positionY = posE - positionYOffset;
  positionZ = lidarReadings[2]-positionZOffset;//0.95 +random(-10,10)/100.0;
}


int dataIndexVectornav = 0;
String incomingDataVectornavString = "";

void handleVectornav() { 
  
  if (Vectornav.available() > 0) {
    char incomingChar = Vectornav.read(); 
 
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
        vectornavIdentity = 1;      
      }
      
      if (incomingDataVectornavString.indexOf("$VNIMU") != -1) {
        vectornavIdentity = 19;;
      }

      if (incomingDataVectornavString.indexOf("$VNG2E") != -1) {
        vectornavIdentity = 33;
      }

      if (incomingDataVectornavString.indexOf("$VNYIA") != -1) {
        vectornavIdentity = 240;
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
      // change of reference point
      vectornavAngleZ = - vectornavAngleX;
      vectornavAngleX = - inter;
      vectornavAngleY *= -1;

      updateData();
      break;
  }
}



