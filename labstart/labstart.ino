#include <Wire.h>
#include <VL6180X.h>

#include "encoders.h"
#include "motor.h"
#include "pid.h"

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define SCALING 2

VL6180X sensor;


float Kp_left = 0.05; //Proportional gain 
float Kd_left = 0.0; //Derivative gain
float Ki_left = 0.0001; //Integral gain
PID_c left_PID(Kp_left, Ki_left, Kd_left); // controller for left wheel

float Kp_right = 0.05; //Proportional gain 
float Kd_right = 0.0; //Derivative gain
float Ki_right = 0.0001; //Integral gain
PID_c right_PID(Kp_right, Ki_right, Kd_right); // controller for right wheel

motor_c left_motor(L_DIR_PIN,L_PWM_PIN);
motor_c right_motor(R_DIR_PIN,R_PWM_PIN);

unsigned long timestamp;
unsigned long speedstamp;
int leftpre;
int rightpre;
int leftstart;
int rightstart;
int right;
int left;
float desspeed;
float demand;
float rightvel;
float leftvel;
float leftoutput;
float rightoutput;
float distance;
bool fin;

void speedcalc(unsigned long timechange){
  int leftchange = count_e0 - leftpre;
  int rightchange = count_e1 - rightpre;
  leftvel = (float)leftchange/(float)timechange;
  rightvel = (float)rightchange/(float)timechange;
  leftpre = count_e0;
  rightpre = count_e1;
  
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  left = 60;
  right = 60;
  setupEncoder0();
  setupEncoder1();
  fin = false;
  left_motor.speedchange(left);
  right_motor.speedchange(right);
  timestamp = millis();
  leftoutput = 0;
  rightoutput = 0;
  leftpre = count_e0;
  rightpre = count_e1;
  leftstart = count_e0;
  rightstart = count_e1;
  desspeed = 0.2;
  demand = (((desspeed/0.21991)*1440)/1000);
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);
}

void loop() {


  unsigned long timechange = millis() - timestamp;
  unsigned long distchange = millis() - speedstamp;
  if (sensor.readRangeSingleMillimeters() > 100 && !fin){
    if (timechange > 100){
      speedcalc(timechange);
      rightoutput = right_PID.update(demand, rightvel);
      leftoutput = left_PID.update(demand, leftvel);
      timestamp = millis();
      
    
      left_motor.speedchange(leftoutput*255);
      right_motor.speedchange(rightoutput*255);
      //Serial.print(leftoutput);
      //Serial.print(",");
      //Serial.println(rightoutput);
    }

  }
  
  else{
    fin = true;
    left_motor.speedchange(0);
    right_motor.speedchange(0);
    if (distchange > 1000){
      Serial.println(((float)count_e1-rightstart)*(0.219911/1440));
      speedstamp = millis();
    }
  }

//Serial.println(sensor.readRangeSingleMillimeters());
}
