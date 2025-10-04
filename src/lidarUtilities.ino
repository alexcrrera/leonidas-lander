#define LIDAR_ADDR 0x62
#define REG_STATUS 0x01
#define REG_DISTANCE 0x8F





float lidarAlphaEWA = 0.21; 

bool LIDAROK = true;
unsigned long timeLiDAR = 0;







void handleLidar(){ // gets LiDAR data at appropriate rate , raises flag if NACK
   const int LIDARFREQUENCY = 10;

 
  if((millis()-timeLiDAR)*1.0>=1000.0/LIDARFREQUENCY){
    timeLiDAR = millis();
    getLidar();
  }

}


// is reading available?
int isLidarAvailable() {

 Wire.beginTransmission(LIDARLite_ADDR);
  Wire.write(0x00); // Register to read distance
  // Check for NACK or other error
  return((Wire.endTransmission() != 0)? -1 : 1);
  
  // Check for NACK or other error (Wire.endTransmission() != 0)? -1 : 1)
  //return(1);
}


  //float lidarReadings[4] = {0.0,0.0,0.0,0.0};// first value, av val, normalised, offset


float lidarNormalised() {
  float l = lidarReadings[1];
  float radAlpha = radians(AngleZ);
  float radBeta = radians(AngleY);

  // Calculate tangent of alpha and beta
  float tanAlpha = tan(radAlpha);
  float tanBeta = tan(radBeta);

  // Square the tangents
  float tanAlphaSquared = tanAlpha * tanAlpha;
  float tanBetaSquared = tanBeta * tanBeta;

  // Add the squared tangents and 1
  float sum = tanAlphaSquared + tanBetaSquared + 1;

  // Calculate the square root of the sum
  float sqrtSum = sqrt(sum);

  // Calculate x
  float x = l / sqrtSum;

  return x;

  // Calculate x

  //lidarReadings[2] = x;
  // Print the result
}

void getLidar() {
    //LiDAR.reading(float(myLidarLite.distance() - 5));
    if(isLidarAvailable()==-1){
   //  LIDAROK = false; 
    //Serial.println("LMAOOO");

    return;
}
 
  float distanceNow = (float)myLidarLite.distance()/100.0; // en m
  //Serial.print(distanceNow);
  lidarReadings[0] = distanceNow-lidarReadings[3];// -lidarReadings[4]; // raw value minus offset
  lidarReadings[1] = EWA(lidarReadings[1], lidarReadings[0],lidarAlphaEWA);
  lidarReadings[2] = lidarNormalised();
}
