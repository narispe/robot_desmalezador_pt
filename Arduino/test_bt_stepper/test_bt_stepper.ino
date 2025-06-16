#include <Ps3Controller.h>

// BT
#define MAC "90:34:fc:0f:69:75"

// STEPPER X
#define STEP_X 23
#define DIR_X 22
#define PLUS_X 1
#define MINUS_X 0
#define X_PER_STEP 1.8  //°

// STEPPER Y
#define STEP_Y 19
#define DIR_Y 18
#define PLUS_Y 0
#define MINUS_Y 1
#define Y_PER_STEP 0.2826 //mm

// STEPPER Z
#define STEP_Z 17
#define DIR_Z 16
#define PLUS_Z 1
#define MINUS_Z 0
#define Z_PER_STEP 0.015 //mm

// STEPPER
#define T_INIT 2000  //ms
#define T_STEP 1000  //us
#define T_DIR 200  //ms
#define STEP_PER_REV 200


float STATE[3];

void setup() {
  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.begin(MAC);
  pinMode(STEP_X, OUTPUT);
  pinMode(DIR_X, OUTPUT);
  pinMode(STEP_Y, OUTPUT);
  pinMode(DIR_Y, OUTPUT);
  pinMode(STEP_Z, OUTPUT);
  pinMode(DIR_Z, OUTPUT);
  digitalWrite(STEP_X, 0);
  digitalWrite(STEP_Y, 0);
  digitalWrite(STEP_Z, 0);
  digitalWrite(DIR_X, PLUS_X);
  digitalWrite(DIR_Y, PLUS_Y);
  digitalWrite(DIR_Z, PLUS_Z);
}


void loop() {

  // Rotate while pressing
  if ( Ps3.data.button.left )
    step_pulse(STEP_X);
  else if ( Ps3.data.button.right )
    step_pulse(STEP_X);
  else if ( Ps3.data.button.down )
    step_pulse(STEP_Y);
  else if ( Ps3.data.button.up )
    step_pulse(STEP_Y);
  else if ( Ps3.data.button.l2 )
    step_pulse(STEP_Z);
  else if ( Ps3.data.button.r2 )
    step_pulse(STEP_Z);
}


// FUNCTIONS

void step_pulse(int step_pin) {
  if ( step_pin == STEP_X ){
    if ( digitalRead(DIR_X) == PLUS_X ){
      STATE[0] += X_PER_STEP;
      Serial.println("MOVE X+");
    } else {
      STATE[0] -= X_PER_STEP;
      Serial.println("MOVE X-");
    }
  } 
  else if ( step_pin == STEP_Y ){
    if ( digitalRead(DIR_Y) == PLUS_Y ){
      STATE[1] += Y_PER_STEP;
      Serial.println("MOVE Y+");
    } else { 
      STATE[1] -= Y_PER_STEP;
      Serial.println("MOVE Y-");
    }
  } 
  if ( step_pin == STEP_Z ){
    if ( digitalRead(DIR_Z) == PLUS_Z ){
      STATE[2] += Z_PER_STEP;
      Serial.println("MOVE Z+");
    } else {
      STATE[2] -= Z_PER_STEP;
      Serial.println("MOVE Z-");
    }
  } 
  digitalWrite(step_pin, 1);
  delayMicroseconds(T_STEP/2);
  digitalWrite(step_pin, 0);
  delayMicroseconds(T_STEP/2);
}

void change_dir(int dir_pin, bool dir_val) {
  if ( digitalRead(dir_pin) == dir_val)
    return; 
  if ( dir_pin == DIR_X & dir_val == PLUS_X)
    Serial.println("CHANGE DIR_X TO +");
  else if ( dir_pin == DIR_X & dir_val == MINUS_X)
    Serial.println("CHANGE DIR_X TO -");
  else if ( dir_pin == DIR_Y & dir_val == PLUS_Y)
    Serial.println("CHANGE DIR_Y TO +");
  else if ( dir_pin == DIR_Y & dir_val == MINUS_Y)
    Serial.println("CHANGE DIR_Y TO -");
  else if ( dir_pin == DIR_Z & dir_val == PLUS_Z)
    Serial.println("CHANGE DIR_Z TO +");
  else if ( dir_pin == DIR_Z & dir_val == MINUS_Z)
    Serial.println("CHANGE DIR_Z TO -");
  delay(T_DIR/2);
  digitalWrite(dir_pin, dir_val);
  delay(T_DIR/2);
}


void notify(){

  // Change dir only when pressing opposite
  if ( Ps3.event.button_down.up ){
    change_dir(DIR_Y, PLUS_Y);
  }
  else if ( Ps3.event.button_down.right ){
    change_dir(DIR_X, PLUS_X);
  }
  else if ( Ps3.event.button_down.down ){
    change_dir(DIR_Y, MINUS_Y);
  }
  else if ( Ps3.event.button_down.left ){
    change_dir(DIR_X, MINUS_X);
  } 
  else if ( Ps3.event.button_down.l2 ){
    change_dir(DIR_Z, MINUS_Z);
  }
  else if ( Ps3.event.button_down.r2 ){
    change_dir(DIR_Z, PLUS_Z);
  }

  // Change dir and move 1 step only when pressing
  else if ( Ps3.event.button_down.circle ){
    change_dir(DIR_X, MINUS_X);
    step_pulse(STEP_X);
  }
  else if ( Ps3.event.button_down.square ){
    change_dir(DIR_X, PLUS_X);
    step_pulse(STEP_X);
  }
  else if ( Ps3.event.button_down.cross ){
    change_dir(DIR_Y, MINUS_Y);
    step_pulse(STEP_Y);
  }
  else if ( Ps3.event.button_down.triangle ){
    change_dir(DIR_Y, PLUS_Y);
    step_pulse(STEP_Y);
  }
  else if ( Ps3.event.button_down.l1 ){
    change_dir(DIR_Z, MINUS_Z);
    step_pulse(STEP_Z);
  }
  else if ( Ps3.event.button_down.r1 ){
    change_dir(DIR_Z, PLUS_Z);
    step_pulse(STEP_Z);
  }
}