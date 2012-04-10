

void BuildDCM (Matrix3* Mat, float someRoll, float somePitch, float someHeadingAngle) {
  float sinPitch = sin(somePitch);
  float cosPitch = cos(somePitch);
  float sinRoll = sin(someRoll);
  float cosRoll = cos(someRoll);
  float sinHeading = sin(someHeadingAngle);
  float cosHeading = cos(someHeadingAngle);
  
  Mat->Y = Vector3(
          cosPitch * sinHeading,  
          sinRoll * sinPitch * sinHeading + cosRoll * cosHeading, 
          cosRoll * sinPitch * sinHeading - sinRoll * cosHeading);
  Mat->Z = Vector3( -sinPitch, sinRoll * cosPitch, cosRoll * cosPitch);
  
  /* Mat->X = Vector3(
          cosPitch * cosHeading,  
          sinRoll * sinPitch * cosHeading - cosRoll * sinHeading, 
          cosRoll * sinPitch * cosHeading + sinRoll * sinHeading);*/
  Mat->X = Mat->Y.cross(Mat->Z);
}


void Update(boolean rate) {
  //Compensation the Roll, Pitch and Yaw drift. 
  Vector3 vOmega;
  Vector3 vErrorGrav;
  Vector3 vEstAccel;
  float cosTheta, sinTheta;
  
  //Process noise matrix for heading
  //NEEDS MORE STUDY!
  Matrix2 Q;
  Q.vv = (deltaT * deltaT) / 0.01 ;
  Q.uv = Q.vv * deltaT / 2;
  Q.vu = Q.uv;
  Q.uu = Q.uv * Q.vu;
  Q.vv  /=  cos(pitch);
  //make last entry in transfer function
  F.uv = deltaT;
  //Update state
  x_hat = F * x_hat;
  //update a_priori estimate covariance
  P = F * P * F.T() + Q;
  
  //New turn-proof G- correction begins here:
  if (gLoad > (1.0 + G_DEADBAND)) {
    cosTheta = 1.0/(gLoad - G_DEADBAND);
  } 
  else if (gLoad > 0.5 && gLoad < (1.0 - G_DEADBAND)) {
    cosTheta = 2.0 - 1.0/(gLoad + G_DEADBAND);
  } 
  else {
    cosTheta = 1;
  }
  cosTheta = 1;

  sinTheta = sqrt(1 - cosTheta * cosTheta);
  //we also assume the aircraft is flying towards positive x
  //therefore if the rate of turn is +ve, theta is +ve
  if ( turnRate < 0) {
    sinTheta *=  - 1;
  } 

  //the new estimator for the true acceleration is the earth z-vector tilted by theta
  //about the heading-line of the aircraft.
  //The z-vector rotated by theta about this what we want.
  //g is negative to the z-vector
  vEstAccel = Vector3(- sinTheta * sin(headingAngle), sinTheta * cos(headingAngle),  -cosTheta);
  //Rotate to aircraft frame
  R.unRotate(&vEstAccel);
  
  //error is orthogonal to accel and estAccel,
  vErrorGrav = vAccel.cross(vEstAccel);
  //proportional and magnified for fast convergence
  if (rate == SLOW) {
    vErrorGrav = vErrorGrav.normalize() * SLEW_RATE;
  }
  
  //rotation = (gyro + Grav correction) times time
  vOmega = (vGyro + vErrorGrav) * deltaT;
  
  //only top two rows of matrix multiplication are needed
  R.X.x +=  R.X.y * vOmega.z + -R.X.z * vOmega.y;
  R.X.y += -R.X.x * vOmega.z +  R.X.z * vOmega.x;
  R.X.z +=  R.X.x * vOmega.y + -R.X.y * vOmega.x;
  R.Y.x +=  R.Y.y * vOmega.z + -R.Y.z * vOmega.y;
  R.Y.y += -R.Y.x * vOmega.z +  R.Y.z * vOmega.x;
  R.Y.z +=  R.Y.x * vOmega.y + -R.Y.y * vOmega.x;
  //third row via orthogonality condition
  R.Z = R.X.cross(R.Y);
  
  pitch = - asin(R.Z.x);
  roll = atan2(R.Z.y, R.Z.z);
  headingAngle = atan2(R.Y.x, R.X.x);
  //could use x_hat.u for heading angle but that is discontinuous at 2pi etc
  
  rollRate = rollRate * BETA + vGyro.x * (1-BETA);
  pitchRate = pitchRate * BETA + (vGyro.y * cos(roll) + vGyro.z * sin(roll)) * (1-BETA);
  turnRate = turnRate * BETA + x_hat.v * (1-BETA);
  
  angleTurned += vGyro.dot(R.Z) * deltaT;
  
}


//Kalman
void CorrectHeading(void) {
 
  //standard dev. of the magnetic heading
  Matrix2 Rk;
  Matrix2 I; //Identity
  Matrix2 S;
  Matrix2 K;
  Vector2 y_tilde;

  //Identity matrix
  I = Matrix2(1,0,0,1);
  
  //measurement noise covariance matrix
  //Make compass less trustworth at high pitch (speed rollover through pole
  Rk = Matrix2((SIGMA_angle * SIGMA_angle) / cos(pitch), 0, 0, vGyroVariance.z);

  //Calculate residual:
  //y_tilde = zed - x_hat
  //can do this componentwise size H is identity
  //angle resid. observed directly
  y_tilde.u = atan2(-vMag.y, vMag.x); 
  //this depends only on roll and pitch angles, not heading:
  y_tilde.v = vGyro.dot(R.Z) - x_hat.v;
  
  //residual covariance
  S = P + Rk;  //again, because H is identity
  
  //Kalman gain:
  K = P * S.I();
  
  //update estimator
  x_hat = x_hat + K * y_tilde;
  
  //update P
  //a posteriori estimate covariance
  P = (I - K) * P;
  
  //To implement new heading angle:
  //we rebuild R entirely -
  //removes need for normalization.
  #if IGNORE_MAG == 1
  BuildDCM(&R, roll, pitch, headingAngle);
  #else
  BuildDCM(&R, roll, pitch, x_hat.u);
  #endif
  //enforce x_hat.u between -pi and pi
  x_hat.u = atan2(R.Y.x, R.X.x);
}











