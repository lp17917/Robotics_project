#ifndef _MOTOR_H
#define _MOTOR_H


// A class to neatly contain commands for the 
// motors, to take care of +/- values, a min/max
// power value, & pin setup.


class motor_c {
  public:

    // This is a function prototype.
    // The actual function is written outside
    // of the class (see below).
    motor_c(int dirpin, int pwrpin);
    void speedchange( int inputspeed );
    bool fwd;
    int dpin;
    int ppin;
    int pwr;
};

// Constructor: when you create an instance
// of this class, your constructor will be
// called automatically.  You can treat it 
// a bit like the main setup().  What needs
// to be setup initially within this class?
motor_c::motor_c(int dirpin, int pwrpin) {

  dpin = dirpin;
  ppin = pwrpin;
  fwd = true;
  pwr = 0;

  pinMode( dpin, OUTPUT );
  pinMode( ppin, OUTPUT );

}

void motor_c::speedchange( int inputspeed ) {
  if (inputspeed < 0){
    fwd = false;
  }
  else{
    fwd = true;
  }
  if (fwd) {digitalWrite(dpin, LOW);}
  else if (!fwd) {digitalWrite(dpin, HIGH);}
  analogWrite(ppin, abs(inputspeed));
  
}


#endif
