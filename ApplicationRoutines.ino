


void DeviceCalibrate(void) {
  Vector3 vCumAccel;
  Vector3 vCumAccelSquared;
  Vector3 vCumGyroRaw;
  Vector3 vCumGyroRawSquared;
  Vector3 vAccelVariance;
  float cumRoll;
  float cumPitch;
  unsigned int  calibCounter;
  
  ReadGyroAccel();
  #if MONGOOSE == 1
  ReadTemperature();
  ReadAltitude();
  #endif
  
  Serial.print("Starting Calibration loop...");
  
  for (calibCounter = 0; calibCounter < NUM_SAMPLES; calibCounter++) {
    ReadGyroAccel();
    vCumAccel += vAccel;
    vCumAccelSquared += vAccel.squared();
    vCumGyroRaw += vGyroRaw;
    vCumGyroRawSquared += vGyroRaw.squared();
    gyroOffsetTemp += gyroTemp;
  }
  
  Serial.println("... finished.");
  
  //Calculate variances
  vAccelVariance = (vCumAccelSquared - (vCumAccel.squared() / NUM_SAMPLES)) / NUM_SAMPLES;
  vGyroVariance = (vCumGyroRawSquared - (vCumGyroRaw.squared() / NUM_SAMPLES)) / NUM_SAMPLES;
  vGyroVariance.x /= GYROGAIN_X * GYROGAIN_X;
  vGyroVariance.y /= GYROGAIN_Y * GYROGAIN_Y;
  vGyroVariance.z /= GYROGAIN_Z * GYROGAIN_Z;
  
  vGyroOffset = vCumGyroRaw / NUM_SAMPLES;
  gyroOffsetTemp /= NUM_SAMPLES;
  
  //either write values to EEPROM or read them from EEPROM
  //if the accel variance is sufficiently small then we were still enough
  //values to store are vGyroVariance, vGyroOffset, vGyroOffsetTemp
  Serial.print("Variance of acceleration * 10000 ");
  Serial.println(vAccelVariance.magnitude() * 1e4);
  if (vAccelVariance.magnitude() > 5.0e-4) {
    //moved during calibration so read values
    Serial.println("Detected motion during calibration. Using stored values.");
    EEPROM_readAnything(0, vGyroOffset);
    EEPROM_readAnything(12, vGyroVariance);
    EEPROM_readAnything(24, gyroOffsetTemp);
  } else {
    //still during calibration so write values
    Serial.println("No motion detected during calibration. Storing values.");
    EEPROM_writeAnything(0, vGyroOffset);
    EEPROM_writeAnything(12, vGyroVariance);
    EEPROM_writeAnything(24, gyroOffsetTemp);
  }
  
  //calculate current body angles
  roll  = -atan2(vCumAccel.y,  -vCumAccel.z); //when right side up, z-accel is -1.0
  pitch = asin(vCumAccel.x  / vCumAccel.magnitude());
  //if not reorienting to null current angles
  #if REORIENT == 0
  // Intermediate R to get level compass reading
  BuildDCM(&R, roll, pitch, 0.0);
  ReadCompass();
  headingAngle = atan2(- vMag.y, vMag.x);
  // Correct R including heading
  BuildDCM(&R, roll, pitch, headingAngle);
  #endif
  
  //null current roll/pitch body angles
  #if REORIENT == 1
  BuildDCM(&orientation, roll, pitch, 0.0);
  ReadCompass();
  headingAngle = atan2(- vMag.y, vMag.x);
  BuildDCM(&R, 0.0, 0.0, headingAngle);
  #endif
  
  slipAngle = 0.0;
  turnRate = 0.0;
  gLoad = 1.0;

  //initialize Kalman filter variables
  //don't know heading, do know stationary:
  P = Matrix2(1, 0, 0, 1);
  //setup transfer matrix: uv term set later as deltaT.
  F = Matrix2(1, 0, 0, 1);
  //initial state variable
  x_hat.u = headingAngle;
  x_hat.v = 0;
  //the other state variable is the heading angle
  //which is embedded in R as atan2(R.Y.x, R.X.x)
}

void StatusLEDToggle(void) {
  static char state = FALSE;
  if(state) {
    digitalWrite(STATUS_LED,LOW);
    state = FALSE;
  }
  else {
    digitalWrite(STATUS_LED,HIGH);
    state = TRUE;
  }
}

void ReadBattery(void) {
  voltage = 2 * 3.3 * analogRead(A0) / 1024;
  int batteryNew = 100 * (voltage - 3.4) / 0.7;
  batteryNew = constrain(batteryNew, 0, 100);
  battery = (batteryNew < battery) ? batteryNew : battery;
}

void InitGyroAccel(void) {
  
   #if MONGOOSE == 1
   InitITG3200();
   InitADXL345();
   #endif
   #if ARDIMU == 1
   InitMPU6000();
   #endif
}

void ReadGyroAccel(void) {
    float gyroTempDelta;
    
    #if MONGOOSE == 1
    ReadITG3200();
    ReadADXL345();
    #endif
    #if ARDIMU == 1
    ReadMPU6000();
    #endif
    
    //Gyroscope die temperature compensation
    if (gyroTemp == 0.0) {
      gyroTemp = gyroTempRaw;     }
    else {
      gyroTemp = gyroTemp * BETA + (1.0 - BETA) * gyroTempRaw;
    }
    gyroTempDelta = gyroTemp - gyroOffsetTemp; //temperature difference since calibration
    
    //GYROSCOPE:
    vGyro.x = (vGyroRaw.x - (GYRO_T_OFFSET_COEFFICIENT_X * gyroTempDelta + vGyroOffset.x)) / GYROGAIN_X;                                                                          
    vGyro.y = (vGyroRaw.y - (GYRO_T_OFFSET_COEFFICIENT_Y * gyroTempDelta + vGyroOffset.y)) / GYROGAIN_Y;
    vGyro.z = (vGyroRaw.z - (GYRO_T_OFFSET_COEFFICIENT_Z * gyroTempDelta + vGyroOffset.z)) / GYROGAIN_Z;
    
    //ACCELEROMETER:
    vAccel.x = (vAccelRaw.x - ACCELOFFSET_X) / ACCELGAIN_X;
    vAccel.y = (vAccelRaw.y - ACCELOFFSET_Y) / ACCELGAIN_Y;
    vAccel.z = (vAccelRaw.z - ACCELOFFSET_Z - baroTemp * ACCELOFFSET_Z_T_COEFFICIENT) / ACCELGAIN_Z;
    
    #if REORIENT == 1
    orientation.rotate(&vAccel);
    orientation.rotate(&vGyro);
    #endif
    
    gLoad = gLoad * BETA + (1.0 - BETA) * vAccel.magnitude();
    slipAngle = slipAngle * BETA + atan2(-vAccel.y, -vAccel.z) * (1.0 - BETA); //damped
    //note positive slip angle is ball-to-the-left
}


//to generate data to allow gyro and accelerometer calibration
//calculates and prints mean accceleration and gyro, and gyro die temp
void GyroAccelCal (void) {

  Vector3 vAvgAccel;
  Vector3 vAvgGyro;
  float avgGyroTemp = 0.0;
  unsigned int  calibCounter;
  
  for (calibCounter = 0; calibCounter < NUM_SAMPLES; calibCounter++) {
    // collect accelerations, gyros
    ReadGyroAccel();
    vAvgAccel += vAccelRaw;
    vAvgGyro += vGyroRaw;
    avgGyroTemp += gyroTemp;
  }
  ReadTemperature();
  vAvgGyro /= NUM_SAMPLES;
  vAvgAccel /= NUM_SAMPLES;
  avgGyroTemp /= NUM_SAMPLES;
  Serial.print(avgGyroTemp);
  Serial.print(", ");
  Serial.print(vAvgGyro.x,1);
  Serial.print(", ");
  Serial.print(vAvgGyro.y,1);
  Serial.print(", ");
  Serial.print(vAvgGyro.z,1);
  Serial.print(", ");
  Serial.print(vAvgAccel.x,1);
  Serial.print(", ");
  Serial.print(vAvgAccel.y,1);
  Serial.print(", ");
  Serial.print(vAvgAccel.z,1);
  Serial.print(", ");
  Serial.println(baroTemp,1);
}












