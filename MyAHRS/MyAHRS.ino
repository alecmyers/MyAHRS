
// Released under Creative Commons License 
// Modifications and additions by Cory Duce to allow it to work with Mongoose hardware
// Based on code by Doug Weibel and Jose Julio which was based on ArduIMU v1.5 by Jordi Munoz 
// and William Premerlani, Jose Julio and Doug Weibel
// Heavily rewritten by Alec Myers

// Axis definition: 
// X axis pointing forward (AWAY FROM the battery connector)
// Y axis pointing to the right 
// Z axis pointing down
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

/* Mongoose Hardware version - v1.0
 ATMega328@3.3V w/ external 16MHz resonator
 High Fuse DA
 Low Fuse FF
 ADXL345: vAccelerometer
 HMC5843: Magnetometer
 IGT-3200: vGyro
 BMP085: Barometric Pressure sensor
 Programmer : 3.3v FTDI
 Arduino IDE : Select board  "Arduino Pro or Pro mini (3.3V, 8MHz) w/ Atmega328"
 */
 
/* ARDIMU Hardware V3.0
 ATMega328@5V w/ external 16MHz resonator
 HMC5843: Magnetometer
 MPU6000 Accelerometer+Gyro
 Programmer : 5.0v FTDI
 Arduino IDE : Select board  "Arduino Pro or Pro mini (5V, 16MHz) w/ Atmega328"
 */

//========================================
// Output Data Configuration
#define PRINT_JSON              0
#define PRINT_LEVIL             0
#define PRINT_LEVILBARO         0
#define CALIBRATE_MAG           0
#define CALIBRATE_GYROACCEL     1

// Which platform
#define MONGOOSE                1
#define ARDIMU                  0

//use local coordinate frame measured at calibration time
//or use Mongoose frame
#define REORIENT                1
#define IGNORE_MAG              0

#include "Parameters.h"
#include "AHRSTypes.h"
#include "MPU6000.h"
#include <HMC58X3.h>
#include <EEPROM.h>
#include "EE.h"
#include <Wire.h>
#include <math.h>

#define DEGTORAD 0.01745329252 // *pi/180
#define RADTODEG 57.2957795131 // *180/pi
#define FALSE 0
#define TRUE 1
#define FAST 1
#define SLOW 0

#define SLEW_RATE 0.004

#define SIGMA_angle  0.1 // S.D. of compass measurement 0.1 radians
//don't apply g angle correction when g-load closer to 1 than this:
#define G_DEADBAND 0.05
//no of samples for initial calibration of gyro drift
#define NUM_SAMPLES 1000
//exponential decay for turn rate, etc
#define BETA 0.975
//decay for vertical speed
#define BETA2 0.8

#if ARDIMU == 1
#define SERIAL_MUX 7
#define RED_LED 5      // RED
#define STATUS_LED 6     // BLUE
#define YELLOW_LED 7   // not installed on v3 board?
#endif

#if MONGOOSE == 1
#define debugPin 6
#define STATUS_LED 4  //PD4 on the Atmega328. Red LED
#endif

float deltaT;    // Integration time (DCM algorithm)
long timer = 0;   //general purpuse timer
long timerOld;
long timer24 = 0; //Second timer used to print values 

Vector3 vGyro;
Vector3 vAccel;
Vector3 vMag;
Vector3 vGyroOffset;
float gyroOffsetTemp = 0.0; //the die temp when the vGyroscope zero-offset was measured
Vector3 vGyroVariance;

Vector3 vGyroRaw;
Vector3 vAccelRaw;
intVector magRaw;

//for calibrating gyros:
float angleTurned;

Matrix3 R;             //the DCM
Matrix3 orientation;   //sensor platform -> body orientation
Matrix3 magGain;       //hard and soft iron compensation

//Kalman variables
Matrix2 F;             //Transfer function
Matrix2 P;             //Covariance
Vector2 x_hat;         //state estimator

// Euler angles etc
float roll;
float pitch;
float headingAngle;
float gLoad;
float slipAngle;
float turnRate;
float rollRate;
float pitchRate;
float magStrength;
short baroTemp;
long baroPres;
int  battery = 100;
float voltage;
int  baroAlt; // remains 0.0 on ardIMU
int  verticalSpeed;
float gyroTemp = 0.0;
unsigned int gyroTempRaw;

//These counters allow us to sample some of the sensors at lower rates
unsigned int  compassCounter = 0;
unsigned int  baroCounter = 0;
unsigned int  powerCounter = 0;
unsigned int  printCounter = 0;
unsigned int  LEDCounter = 0;
unsigned int  powerOnCounter = 0;

void setup(void) { 
  
  Serial.begin(115200);
  pinMode (STATUS_LED, OUTPUT);  // Status LED
  //pinMode (debugPin, OUTPUT);  // debug LED

  Serial.println();
  Serial.println("AHRS firmware rewritten by Alec Myers.");
  delay(50);
  Wire.begin();    //Init the I2C
  delay(200);
  Serial.println("Init Gyros and Accelerometers...");
  InitGyroAccel();
  InitCompass();
  #if MONGOOSE == 1
  Serial.println("Init Baro...");
  InitBaro();
  # endif
  Serial.println("Calibrating device parameters...");
  DeviceCalibrate();
  
  timer = millis();
  delay(100);
  Serial.println("Beginning operation.");
}

void loop(void) { //Main Loop 

  #if CALIBRATE_GYROACCEL == 1
  GyroAccelCal();
  #else
  
  if((DIYmillis()-timer)>=5) { // Main loop runs at 200Hz

    timerOld = timer;
    timer=DIYmillis();
    // Real time of loop run.
    deltaT = (timer-timerOld)/1000.0; 
    if(deltaT > 1) {
      //keeps dt from blowing up
      deltaT = 0;        
    }
     
    powerOnCounter++;
    compassCounter++;
    baroCounter++;
    powerCounter++;
    printCounter++;
    LEDCounter++;

    ReadGyroAccel();
  
    if (compassCounter > 10) { //20Hz
      compassCounter = 0;
      ReadCompass();
      CorrectHeading();
    }
    
    #if MONGOOSE == 1
    if (baroCounter > 100) { //2Hz
      baroCounter = 0; 
      ReadTemperature();
      ReadAltitude();
      PrintBaro();
    }
    #endif 

    if (powerOnCounter < 400) { //2 seconds
      Update(FAST);
    } else {
      powerOnCounter = 600;
      Update(SLOW); 
    }
  
    if (powerCounter > 200) {  //1 Hz
      powerCounter = 0;
      ReadBattery();
      PrintPower(); 
    }
    // Output every 8 iterations
    if (printCounter > 7) {
      printCounter = 0;
      PrintData(); 
    }
    
    if (LEDCounter >  49) {
      LEDCounter = 0;
      StatusLEDToggle(); 
    }
  }
  #endif
}


