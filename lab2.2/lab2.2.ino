// Lab 2.2: Inverse Kinematics

#include <Sparki.h>
#include <math.h>

#define LOOP_TIME         100
#define AXLE_DIST         0.08586
#define RAD               0.025
#define VEL               0.0278551532

// ***************************************************************

float xI                = 0.0;
float yI                = 0.0;
float thetaR            = 0.0;

float xG                = 0.15; // 15cm
float yG                = 0.15;
float thetaG            = 0.0;

float n                 = 0.0;
float rho               = 0.0;
float alpha             = 0.0;

float xR                = 0.0;
float thetaRP           = 0.0;

float rVel;
float lVel;
float avgVel            = VEL;

unsigned long int startTime;

// ***************************************************************

void setup() {
  sparki.clearLCD();
}

// ***************************************************************

// 1. Calculate wheelspeeds to get desired behavior (line following, drive to goal etc.)
//   -> end result: left speed, right speeds
//    |                  ^
//    v                  |
//   wheelspeeds        Pose
// ----------------------------------------
// 2. Set those wheelspeeds (Sparki.Motor(CCW,rightspeed); Sparki.Motor(CW,leftspeed))
// 3. Do odometry using rightspeed and leftspeed
// 4. Make sure loop only takes 100ms

void loop() {
  startTime = millis();

  // Translate into real-world coordinate frame
  int xPoint = (xI / 10) + 50;
  int yPoint = (yI / 10) + 50;

  // Map out the path of Sparki
  sparki.drawPixel(xPoint, yPoint);
  sparki.updateLCD();

  // *******************************************************
  // * Feedback Controller

  rho = sqrt(pow((xI - xG), 2) + pow((yI - yG), 2));
  alpha = thetaR - atan2((yI - yG), (xI - xG)) - PI/2.0;
  n = thetaG - thetaR;

  Serial.print("rho: "); Serial.print(rho);
  Serial.print(" alpha: "); Serial.print(alpha / PI * 180.0);
  Serial.print(" n: "); Serial.println(n); Serial.println("- - - -");

  // Forward speed
  xR = 0.1 * rho;

  // Rotational speed
  thetaRP = 0.1 * (alpha); // + 0.01 * n;

  // *******************************************************
  // * Inverse Kinematics [Formula 3.64]

  lVel = (2.0 * xR / RAD - thetaRP * AXLE_DIST / RAD) / 2.0;
  rVel = (2.0 * xR / RAD + thetaRP * AXLE_DIST / RAD) / 2.0;

  // Serial.print("L: " + lVel + " "); Serial.print("R: " + rVel);

  sparki.motorRotate(MOTOR_LEFT, DIR_CCW, lVel / (VEL / RAD) * 100.0);
  sparki.motorRotate(MOTOR_RIGHT, DIR_CW, rVel / (VEL / RAD) * 100.0);

  // *******************************************************
  // * Odometry [Formula 3.40]

  // Average rotation speeds of the L & R wheels (phi * radius)
  avgVel = (rVel * RAD + lVel * RAD) * 0.5;

  xI += cos(thetaR) * avgVel * LOOP_TIME / 1000.0;
  yI += sin(thetaR) * avgVel * LOOP_TIME / 1000.0;
  thetaR += (rVel * RAD - lVel * RAD) / AXLE_DIST * LOOP_TIME / 1000.0;

  Serial.print("xI: "); Serial.print(xI);
  Serial.print(" yI: "); Serial.print(yI);
  Serial.print(" thetaR: "); Serial.println(thetaR/PI * 180.0);

  // Ensure every loop is exactly 100 ms
  while (millis() < startTime + LOOP_TIME) {}
}
