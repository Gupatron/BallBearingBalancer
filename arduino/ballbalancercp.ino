#include <math.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <stdint.h>
#include "TouchScreen.h"

#define pul1 2
#define dir1 3
#define pul2 6
#define dir2 5
#define pul3 8
#define dir3 1

//Enable Pins for Drivers
int ena1 = 32;
int multistepping = 31;
int ena3 = 30;


//Add instance of stepper motors
AccelStepper stepper1(1, pul1,dir1);
AccelStepper stepper2(1, pul2,dir2);
AccelStepper stepper3(1, pul3,dir3);
MultiStepper steppers;    

bool detected = 0; // Is ball on platform
float steptodeg = 3200.0/360.0; //number of steps in a degree
double kp =.3, ki =.0005, kd =75, ks = 20; //PID constants

long timeI;

long initialPos = 52.45; //Angle for platform with 0 pitch or roll

float phinaught = 34.71;// Angle of arm in resting position

//PID arrays
double previous_error[2], error[2] = {0,0};
double integral[2] = {0,0},  derivative[2] = {0,0}, out[2]; 
double speed[3] = {0,0,0}, speedPrev[3];     
long pos[3];

//IK Stuff
float pitch;
float roll;

double cpx;
double cpy;

//Platform Constants
double joint1 = 45;
double joint2 = 95;
float initialz = 95;
double r = 77.94228634; //Platform distance from center to mounting points
double e =  51.96152423; //Base platform distance from center to motors (r is 1.5x e)
float motangle1,motangle2,motangle3;

//Distance to center of touchpad from x and y direction
double distance_centerx = 500;
double distance_centery = 500;

TouchScreen ts = TouchScreen(A1, A2, A3, A0, 0);  //touch screen pins (XGND, YGND, X5V, Y5V, resistance of touchpad)

void setup() {
  Serial.begin(115200);

  pinMode(ena1, OUTPUT);
  pinMode(multistepping, OUTPUT);
  pinMode(ena3, OUTPUT);

  digitalWrite(ena1, HIGH);
  digitalWrite(multistepping, HIGH);
  digitalWrite(ena3, HIGH);
  //Turns drivers off initially
  delay(1000);
  digitalWrite(ena1, LOW);
  digitalWrite(ena3, LOW);
//Turns Drivers on

  //Sets stepper current position to 0
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);


  stepperPos(initialPos,initialPos,initialPos);
  steppers.runSpeedToPosition();

  Serial.println("Setup complete");
}

void loop() {
    if (Serial.available() > 0) {
        String cp = Serial.readStringUntil('\n');  // Read incoming string
        int commaIndex = cp.indexOf(',');         // Find the position of the comma
        if (commaIndex > 0) {
            String xStr = cp.substring(0, commaIndex);  // Extract x value
            String yStr = cp.substring(commaIndex + 1); // Extract y value
            cpx = xStr.toFloat();  // Convert x to float
            cpy = yStr.toFloat();  // Convert y to float

        }
    }


    PID(cpx, cpy);
    inverseKinematics(pitch, roll);
}



void PID(double setPointx, double setPointy){

TSPoint p = ts.getPoint();



if(p.x > 0){
  detected = 1;

  for(int i = 0; i < 2; i++){
      previous_error[i] = error[i];
      delay(2);
      if (i == 0) {
      error[i] = distance_centerx - p.x - setPointx;
      } 
    else if (i == 1) {
    error[i] = distance_centery - p.y - setPointy;
    }
      integral[i] += error[i] + previous_error[i];
      derivative[i] = error[i] - previous_error[i];
      derivative[i] = isnan(derivative[i]) || isinf(derivative[i]) ? 0 : derivative[i];
      out[i] = kp * error[i] + ki * integral[i] + kd * derivative[i];  
      out[i] = constrain(out[i], -500, 500);       
  }
      for (int i = 0; i < 3; i++) {
      speedPrev[i] = speed[i];                                                                                                           //sets previous speed
      speed[i] = (i == 1) * stepper1.currentPosition() + (i == 2) * stepper2.currentPosition() + (i == 3) * stepper3.currentPosition();  //sets current position
      speed[i] = abs(speed[i] - pos[i]) * ks;                                                                                            //calculates the error in the current position and target position
      speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200);                                                            //filters speed by preventing it from beign over 100 away from last speed
      speed[i] = constrain(speed[i], 0, 1500);                                                                                           //constrains sped from 0 to 1000
    }
}
else{
delay(20);
  if(p.x == 0){
    detected = 0;
  }
}

pitch = mapFloat(out[0], -500, 500, -25.0, 25.0);
roll = mapFloat(out[1], -500, 500, -25.0, 25.0);   

}

void inverseKinematics(float theta, float phi)
{
theta = theta * PI / 180.0; //Takes mapped value from PID and converts to radians
phi = phi * PI / 180.0;

//Vectors for Inverse Kinematics
float p[3] = {0, 0, initialz};//Height of Platform
//Platform Vectors
float ba[3] = {0, -r, 0};
float bb[3] = {r * sqrt(3) / 2, r / 2, 0};
float bc[3] = {-r * sqrt(3) / 2, r / 2, 0};
//Base Vectors
float aa[3] = {0, -e, 0};
float ab[3] = {e * sqrt(3) / 2, e / 2, 0};
float ac[3] = {-e * sqrt(3) / 2, e / 2, 0};

//Rotation Matrix and Leg Length Vectors
float s1[3], s2[3], s3[3];

float RotMatrix[3][3] = {
{cos(theta), sin(theta)*sin(phi), cos(phi)*sin(theta)},
{0.0, cos(phi), -sin(phi)},
{-sin(theta),cos(theta) * sin(phi),cos(phi) * cos(theta)}
};

for (int i = 0; i < 3; i++) {
float dotProduct_ba = 0.0;
float dotProduct_bb = 0.0;
float dotProduct_bc = 0.0;

for (int j = 0; j < 3; j++) {
dotProduct_ba += RotMatrix[i][j] * ba[j];
dotProduct_bb += RotMatrix[i][j] * bb[j];
dotProduct_bc += RotMatrix[i][j] * bc[j];
}

s1[i] = p[i] + dotProduct_ba - aa[i];
s2[i] = p[i] + dotProduct_bb - ab[i];
s3[i] = p[i] + dotProduct_bc - ac[i];
}
float leg1 = sqrt(pow(s1[0], 2) + pow(s1[1], 2) + pow(s1[2], 2));
float leg2 = sqrt(pow(s2[0], 2) + pow(s2[1], 2) + pow(s2[2], 2));
float leg3 = sqrt(pow(s3[0], 2) + pow(s3[1], 2) + pow(s3[2], 2));

//motor angle 1
float numerator1 = pow(leg1, 2) + pow(joint1, 2) - pow(joint2, 2);
float denominator1 = 2* leg1 * joint1;
 motangle1 = acos(numerator1/denominator1);
//motor angle 2
float numerator2 = pow(leg2, 2) + pow(joint1, 2) - pow(joint2, 2);
float denominator2 = 2* leg2 * joint1;
 motangle2 = acos(numerator2/denominator2);
//motor angle 3
float numerator3 = pow(leg3, 2) + pow(joint1, 2) - pow(joint2, 2);
float denominator3 = 2* leg3 * joint1;
motangle3 = acos(numerator3/denominator3);

motangle1 = motangle1 * 180/PI; //radians to degrees
motangle1 = 90 + phinaught - motangle1;

motangle2 = motangle2 * 180/PI;
motangle2 = 90 + phinaught - motangle2;

motangle3 = motangle3 * 180/PI;
motangle3 = 90 + phinaught - motangle3;

stepperPos(motangle1,motangle2,motangle3);

}

void stepperPos(float affectorangle1, float affectorangle2, float affectorangle3) {
    
    if(detected){


            pos[0] = (affectorangle1) * steptodeg;  // Steps for stepper1
            pos[1] = (affectorangle2) * steptodeg;  // Steps for stepper2
            pos[2] = (affectorangle3) * steptodeg;  // Steps for stepper3

  stepper1.setMaxSpeed(speed[0]);
  stepper2.setMaxSpeed(speed[1]);
  stepper3.setMaxSpeed(speed[2]);
 
  stepper1.setAcceleration(speed[0] * 30);
  stepper2.setAcceleration(speed[1] * 30);
  stepper3.setAcceleration(speed[2] * 30);

  steppers.moveTo(pos);
  steppers.run();
  }

  else{



  stepper1.setMaxSpeed(800);
  stepper2.setMaxSpeed(800);
  stepper3.setMaxSpeed(800);

  pos[0] = initialPos * steptodeg;
  pos[1] = initialPos * steptodeg;
  pos[2] = initialPos * steptodeg;

  steppers.moveTo(pos);
  steppers.run();

  }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }