/* ******************************************************* */
/* I2C code for ADXL345 vAccelerometer                      */
/*                                                         */
/* ******************************************************* */

//I2C addresses 
int AccelAddress = 0x53;     //Write:0xA6  Read:0xA7

//============================================
// Accelerometer
//============================================
void InitADXL345(void) {
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x2D);  // power register
  Wire.write(0x08);  // measurement mode
  Wire.endTransmission();
  delay(20);
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x31);  // Data format register
  Wire.write(0x08);  // set to full resolution
  Wire.endTransmission();
  delay(20);	
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwith)
  //Wire.beginTransmission(AccelAddress);
  //Wire.write(0x2C);  // Rate
  //Wire.write(0x09);  // set to 50Hz, normal operation
  //Wire.endTransmission();
}


// Reads x,y and z vAccelerometer registers
void ReadADXL345(void) {
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(AccelAddress); 
  Wire.write(0x32);        //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.beginTransmission(AccelAddress); //start transmission to device
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device

  while(Wire.available()) {  // ((Wire.available())&&(i<6))
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  if (i==6) { // All bytes received?

    //get the raw data
    vAccelRaw.x = ( ((int)buff[1]) << 8) | buff[0]; // X axis
    vAccelRaw.y = ( ((int)buff[3]) << 8) | buff[2]; // Y axis 
    vAccelRaw.z = ( ((int)buff[5]) << 8) | buff[4]; // Z axis

  }
  else
    Serial.println("!ERR: Error reading accelerometer info!");
}



