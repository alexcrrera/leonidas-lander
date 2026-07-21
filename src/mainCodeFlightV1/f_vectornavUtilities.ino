
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
  //  Serial.println(F("overflowwww"));
    dataIndexVectornav = 0;
    incomingDataVectornavString = "";
    }
}


int checkHeaderVectornav(){

      int vectornavIdentity = -1;
      if (incomingDataVectornavString.indexOf("$VNYPR") != -1) {  
       //  Serial.println("LALALAVNYPR");        
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

    // $VyNYPR,aw,pitch,roll,eec
      commaParser.parseLine(incomingDataVectornav,headerVectornav,vectornavAngleX_mes,vectornavAngleY_mes,vectornavAngleZ_mes);

      float inter = vectornavAngleZ_mes;

      vectornavAngleZ = EWA(vectornavAngleZ,-vectornavAngleX_mes, EMWA_VECTORNAV);
      vectornavAngleX = EWA(vectornavAngleX,- inter, EMWA_VECTORNAV);
      vectornavAngleY = EWA(vectornavAngleY,- vectornavAngleY_mes, EMWA_VECTORNAV);




      vectornavAnglesUpdate = true;

      break;
  }
}



