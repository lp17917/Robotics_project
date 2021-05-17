#include <Wire.h>
#include <VL6180X.h>

#include "encoders.h"
#include "motor.h"
#include "pid.h"

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define SCALING 3

VL6180X sensor;


float Kp_left = 0.1; //Proportional gain
float Kd_left = 0.0; //Derivative gain
float Ki_left = 0.0001; //Integral gain
PID_c left_PID(Kp_left, Ki_left, Kd_left); // controller for left wheel

float Kp_right = 0.1; //Proportional gain
float Kd_right = 0.0; //Derivative gain
float Ki_right = 0.0001; //Integral gain
PID_c right_PID(Kp_right, Ki_right, Kd_right); // controller for right wheel

float Kp_dist = 1; //Proportional gain
float Kd_dist = 0.0; //Derivative gain
float Ki_dist = 0.0; //Integral gain
PID_c dist_PID(Kp_dist, Ki_dist, Kd_dist); // controller for left wheel


motor_c left_motor(L_DIR_PIN, L_PWM_PIN);
motor_c right_motor(R_DIR_PIN, R_PWM_PIN);

unsigned long timestamp;
unsigned long speedstamp;
int leftpre;
int rightpre;
int leftstart;
int rightstart;
int right;
int ja = 0;
int left;
float desspeed;
float demand;
float demanddist = 100;
float rightvel;
float leftvel;
float leftoutput;
float rightoutput;
float distance;
bool fin;
bool finmes = false;
float init_distance=500;
float boundary_distance=100;
//low pass filter
const float alpha=0.5;
const int n=1;
double data_filtered[]={0,0};
float pidout;
float rightsped;
const int q=10;
float data_velocity[100];
int sample;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  pidout = 1;
  left = 0;
  right = 0;
  setupEncoder0();
  setupEncoder1();
  fin = false;
  left_motor.speedchange(left);
  right_motor.speedchange(right);
  timestamp = millis();
  sample = 0;
  leftoutput = 0;
  rightoutput = 0;
  leftpre = count_e0;
  rightpre = count_e1;
  leftstart = count_e0;
  rightstart = count_e1;
  desspeed = 0.6  ;
  distance = 500;
  rightsped = 0;
  demand = (((desspeed / 0.21991) * 1440) / 1000);
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);
  sensor.startRangeContinuous(50);
}

void loop() {

  unsigned long timechange = millis() - timestamp;
  unsigned long distchange = millis() - speedstamp;

  
  if (!fin){
    
    if (timechange > 25) {
  
    
      speedcalc(timechange);
      //sample += 1;
      //if (sample == 2){
        data_velocity[ja] = rightvel * 100;
        Serial.print(distance);
        Serial.print(",");
        Serial.println(rightvel * 100);
        ja += 1;
      
        if (ja == 100){
     
          ja = 0;
        }
       // sample = 0;
      //}

      
      rightoutput = right_PID.update(demand, rightvel);
      leftoutput = left_PID.update(demand, leftvel);
      timestamp = millis();
      left_motor.speedchange(leftoutput * 255);
      right_motor.speedchange(rightoutput * 255);

      if ( distance >= init_distance){
        pidout = 1;

     }
      else if( distance < boundary_distance){
        pidout = 0;
        left_motor.speedchange(0);
        right_motor.speedchange(0);
        fin = true;

      }
      else {
      //update distance pid
        pidout = (dist_PID.update(demanddist, distance)/1000);
        pidout = -pidout;

      }
    demand = (((desspeed / 0.21991) * 1440) / 1000) * pidout;
    rightsped = (((rightvel * 1000) / 1440) * 0.21991);

    /*
    Serial.print(",");
    Serial.print("distance: ");
    Serial.print(distance/100);
    Serial.print(",");
    Serial.print("pidout: ");
    Serial.println(pidout);

*/


    }
    if (distchange > 50){
      distance = sensor.readRangeContinuousMillimeters();
      speedstamp = millis();
    }

  }
  else{
    if (!finmes){
      distance = sensor.readRangeSingleMillimeters();
      finmes = true;
    }
    Serial.print("count :");
    Serial.println(ja);
    for (int o=0;o<100;o++){
      Serial.print("data_velocity[i]:");
      Serial.print(",");
      Serial.print(o);
      Serial.print(",");
      Serial.println(data_velocity[o]);

    }
    
    Serial.print(distance);
    delay(10000);
  }


}



void speedcalc(unsigned long timechange) {
  int leftchange = count_e0 - leftpre;
  int rightchange = count_e1 - rightpre;
  leftvel = (float)leftchange / (float)timechange;
  rightvel = (float)rightchange / (float)timechange;
  leftpre = count_e0;
  rightpre = count_e1;

}
