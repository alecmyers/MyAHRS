/* ******************************************************* */
/* I2C code for HMC5583 magnetometer                       */
/*                                                         */
/* ******************************************************* */


//I2C addresses 
int CompassAddress = 0x1E;   //Write:0x3C  Read:0x3D

HMC58X3 magn;

//============================================
// Magnetometer
//============================================
void InitCompass() {
  // no delay needed as we have already a delay(5) in HMC5843::init()
  magn.init(false); // Dont set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  magn.calibrate(2); // Use gain 1=default, valid 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
  magn.setMode(0);
  
  magGain.X = Vector3 (MAGGAIN_1, MAGGAIN_2, MAGGAIN_3);
  magGain.Y = Vector3 (MAGGAIN_4, MAGGAIN_5, MAGGAIN_6);
  magGain.Z = Vector3 (MAGGAIN_7, MAGGAIN_8, MAGGAIN_9);
  
}

void ReadCompass() {
  
  magn.getRaw(&magRaw.x, &magRaw.y, &magRaw.z);
  vMag.x = (magRaw.x - MAGOFFSET_X);
  vMag.y = (magRaw.y - MAGOFFSET_Y);
  vMag.z = (magRaw.z - MAGOFFSET_Z);
  magGain.rotate(&vMag); //field in sensor/PCB coordinates
  
  //reorient:
  #if REORIENT == 1
  orientation.rotate(&vMag); //field now in body coordinates
  #endif
  
  R.rotate(&vMag); //field in earth coordinates; should be parallel to earthBField
 
  magStrength = vMag.magnitude();
  vMag /= magStrength;
}






















