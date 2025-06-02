#include <Ps3Controller.h>
#include <DRV8825.h>

// BT
#define MAC "90:34:fc:0f:69:75"

// STEPPER X
DRV8825 stepper_x;
#define STEP_X 23
#define DIR_X 22
#define PLUS_X 1
#define MINUS_X 0
#define X_PER_STEP 1.8  //°

// STEPPER Y
DRV8825 stepper_y;
#define STEP_Y 19
#define DIR_Y 18
#define PLUS_Y 0
#define MINUS_Y 1
#define Y_PER_STEP 0.2826 //mm

// STEPPER Z
DRV8825 stepper_z;
#define STEP_Z 17
#define DIR_Z 16
#define PLUS_Z 1
#define MINUS_Z 0
#define Z_PER_STEP 0.015 //mm

// STEPPER
#define T_INIT 2000  //ms
#define T_STEP 10  //ms
#define T_DIR 50  //ms
#define PULSE_LEN_MIN 2 //us
#define STEP_PER_REV 200


float STATE[3];


void setup() {
  Serial.begin(115200);

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin(MAC);

  stepper_x.begin(DIR_X, STEP_X);
  stepper_y.begin(DIR_Y, STEP_Y);
  stepper_z.begin(DIR_Z, STEP_Z);
  stepper_x.setStepPulseLength(PULSE_LEN_MIN);
  stepper_y.setStepPulseLength(PULSE_LEN_MIN);
  stepper_z.setStepPulseLength(PULSE_LEN_MIN);
  delay(T_INIT);
}


void loop() {

  if ( Ps3.data.button.left || Ps3.data.button.right){
    stepper_x.step();
    if ( stepper_x.getDirection() == PLUS_X ) {
      Serial.println("MOTOR X+");
      STATE[0] += X_PER_STEP;
    } else {
      Serial.println("MOTOR X-");
      STATE[0] -= X_PER_STEP;
    }
  }

  if ( Ps3.data.button.down || Ps3.data.button.up){
    stepper_y.step();
    if ( stepper_y.getDirection() == PLUS_Y ) {
      Serial.println("MOTOR Y+");
      STATE[1] += Y_PER_STEP;
    } else {
      Serial.println("MOTOR Y-");
      STATE[1] -= Y_PER_STEP;
    }
  }

  if ( Ps3.data.button.l1 || Ps3.data.button.r1){
    stepper_z.step();
    if ( stepper_z.getDirection() == PLUS_Z ) {
      Serial.println("MOTOR Z+");
      STATE[2] += Z_PER_STEP;
    } else {
      Serial.println("MOTOR Z-");
      STATE[2] -= Z_PER_STEP;
    }
  }

  delay(T_STEP);
}


void notify() {
  // ACTIVE CONTROL
  if( Ps3.event.button_down.up ) {
    Serial.println("Started pressing the up button");
    stepper_y.setDirection(PLUS_Y);
  }
  if( Ps3.event.button_down.down ) {
    Serial.println("Started pressing the down button");
    stepper_y.setDirection(MINUS_Y);
  }
  if( Ps3.event.button_down.left ) {
    Serial.println("Started pressing the left button");
    stepper_x.setDirection(PLUS_X);
  }
  if( Ps3.event.button_down.right ) {
    Serial.println("Started pressing the right button");
    stepper_x.setDirection(MINUS_X);
  }
  if( Ps3.event.button_down.l2 ) {
    Serial.println("Started pressing the left shoulder button");
    stepper_z.setDirection(MINUS_Z);
  }
  if( Ps3.event.button_down.r2 ) {
    Serial.println("Started pressing the right shoulder button");
    stepper_z.setDirection(PLUS_Z);
  }

  // CONTROL PER PRESS
  if( Ps3.event.button_down.triangle ) {
    Serial.println("Started pressing the triangle button");
    stepper_y.setDirection(PLUS_Y);
    stepper_y.step();
  }
  if( Ps3.event.button_down.cross ) {
    Serial.println("Started pressing the cross button");
    stepper_y.setDirection(MINUS_Y);
    stepper_y.step();
  }
  if( Ps3.event.button_down.square ) {
    Serial.println("Started pressing the square button");
    stepper_x.setDirection(PLUS_X);
    stepper_x.step();
  }
  if( Ps3.event.button_down.circle ) {
    Serial.println("Started pressing the circle button");
    stepper_x.setDirection(MINUS_X);
    stepper_x.step();
  }
  if( Ps3.event.button_down.l1 ) {
    Serial.println("Started pressing the left shoulder button");
    stepper_z.setDirection(MINUS_Z);
    stepper_z.step();
  }
  if( Ps3.event.button_down.r1 ) {
    Serial.println("Started pressing the right shoulder button");
    stepper_z.setDirection(PLUS_Z);
    stepper_z.step();
  }

  delay(T_DIR);
}

void onConnect() {
  Serial.println("Connected!.");
}