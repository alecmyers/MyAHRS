/* ******************************************************* */
/* I2C code for ITG-3200 Gyro                              */
/*                                                         */
/* ******************************************************* */

//I2C addresses 
int GyroAddress = 0x68;      //Write:0xD0  Read:0xD1



//============================================
// Gyro
//============================================
void InitITG3200() {
  //Set the sample rate divider
  Wire.beginTransmission(GyroAddress);
  Wire.write(0x15);  // Sample rate divider register
  Wire.write((byte)0x0);  // Sample rate divider is 1 (register value + 1)
  Wire.endTransmission();
  delay(20);

  //Set the DLPF
  Wire.beginTransmission(GyroAddress);
  Wire.write(0x16);  // DLPF register
  Wire.write( (0x03<<3) | (0x04<<0) );  // Set the full-scale range to +/- 2000deg/sec, and the low pass filter to 20Hz
  Wire.endTransmission();
  delay(20);	

  //Setup the clock reference
  Wire.beginTransmission(GyroAddress);
  Wire.write(0x3E);  // Power and clock register
  Wire.write(1);  // Use the PLL with the X Gyro as the clock reference
  Wire.endTransmission();
  delay(20);	

  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwith)
  //Wire.beginTransmission(gyroAddress);
  //Wire.write(0x2C);  // Rate
  //Wire.write(0x09);  // set to 50Hz, normal operation
  //Wire.endTransmission();
}



// Reads the angular rates from the Gyro
void ReadITG3200() {
  int i = 0;
  byte buff[8];  //6 bytes of angular rate data, and 2 bytes of temperature data

  Wire.beginTransmission(GyroAddress); 
  Wire.write(0x1B);        //The temperature and gyro data starts at address 0x1B
  Wire.endTransmission(); //end transmission

    Wire.beginTransmission(GyroAddress); //start transmission to device
  Wire.requestFrom(GyroAddress, 8);    // request 8 bytes from device

  while(Wire.available()) {  // ((Wire.available())&&(i<6))
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission

    if (i==8) { // All bytes received?
    //get the raw data
    vGyroRaw.x = ((((int)buff[2]) << 8) | buff[3]);    // X axis
    vGyroRaw.y = ((((int)buff[4]) << 8) | buff[5]);    // Y axis
    vGyroRaw.z = ((((int)buff[6]) << 8) | buff[7]);   
    gyroTempRaw = ((((int)buff[0]) << 8) | buff[1]);    
  }

  else
    Serial.println("!ERR: Error reading Gyro info!");
}





