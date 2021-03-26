#include <Wire.h>
#include <VL6180X.h>

#include "encoders.h"
#include "motor.h"


#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define SCALING 2

VL6180X sensor;

motor_c left_motor(L_DIR_PIN,L_PWM_PIN);
motor_c right_motor(R_DIR_PIN,R_PWM_PIN);

int right;
int left;
long leftstamp;
long rightstamp;
bool fin;

void setup() {
  Serial.begin(9600);
  Wire.begin();
 
  leftstamp = get_e0;
  rightstamp = get_e1;
  left = 30;
  fin = false;
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);
}

void loop() {

  if (sensor.readRangeSingleMillimeters() > 100 && !fin){
      left_motor.speedchange(left);
      right_motor.speedchange(right);
  }
  else{
    fin = true;
     left_motor.speedchange(0);
     right_motor.speedchange(0);
  }

}
