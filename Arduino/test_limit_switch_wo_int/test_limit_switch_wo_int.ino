#include <Ps3Controller.h>

//---------DEFINITIONS-----------

// SERIAL
#define SERIAL 115200
// BT
#define MAC "90:34:fc:0f:69:75"
// CNC SHIELD
#define EN_CNC 0
#define LS_X 32
#define LS_Y 33
#define LS_Z 25
#define STEP_X 19
#define DIR_X 18
#define STEP_Y 5
#define DIR_Y 17
#define STEP_Z 16
#define DIR_Z 4
// STEPPER DIRS
#define PLUS_X 1
#define MINUS_X 0
#define PLUS_Y 0
#define MINUS_Y 1
#define PLUS_Z 1
#define MINUS_Z 0
// STEPERS PITCH
#define X_PITCH 0.1286  // °
#define Y_PITCH 0.2826 // mm
#define Z_PITCH 0.015 // mm
// STEPERS VEL (T = RPM * 3e5)
#define T_STEP_X 3000 // us
#define T_STEP_Y 3000 // us
#define T_STEP_Z 1000 // us
#define T_DIR 250   // ms
#define T_CAL_DELAY 500 // ms
// PUENTE H
#define ENA 14
#define IN1 27
#define IN2 26
#define PWM_FREQ 1000 // hz
#define PWM_RES 8
// ENCODER
#define ENCA 35
#define ENCB 34
#define NFactor 400.0f
// PID
#define T_S 50  // ms
#define KP 1.2f
#define KI 1.5f
#define KD 0.0f

//---------VARIABLES-----------
// Limit Switch variables 
bool limit_x_plus_active = false;
bool limit_x_minus_active = false;
bool limit_y_plus_active = false;
bool limit_y_minus_active = false;
bool limit_z_plus_active = false;
bool limit_z_minus_active = false;

float STATE[3] = {0.0, 0.0, 0.0};
int range_x, range_y, range_z;
unsigned long t_counter_calib;


//---------SETUP-----------
void setup() {
  Serial.begin(SERIAL);
  init_cnc_shield();
  init_ps3();
}

//---------LOOP-----------
void loop() {
  check_switchs();
  control_Ps3();
}


//---------FUNCTIONS-----------
// STEPPERS
void step_pulse(int step_pin, int t_pulse){
  if ( !check_limit(step_pin) )
    return;
  switch ( step_pin ){
    case STEP_X:
      STATE[0] += (digitalRead(DIR_X) == PLUS_X) ? X_PITCH : -X_PITCH; break;
    case STEP_Y:
      STATE[1] += (digitalRead(DIR_Y) == PLUS_Y) ? Y_PITCH : -Y_PITCH; break;
    case STEP_Z:
      STATE[2] += (digitalRead(DIR_Z) == PLUS_Z) ? Z_PITCH : -Z_PITCH; break;
  }
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(t_pulse/2);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(t_pulse/2);
}
void change_dir(int dir_pin, bool dir_val){
  if ( digitalRead(dir_pin) != dir_val ){
    delay(T_DIR/2);
    switch ( dir_pin ){
      case DIR_X:
        digitalWrite(DIR_X, dir_val); break;
      case DIR_Y:
        digitalWrite(DIR_Y, dir_val); break;
      case DIR_Z:
        digitalWrite(DIR_Z, dir_val); break;
    }
    delay(T_DIR/2);
  }
}

//  LIMIT SWITCH
void check_switchs(){
  if ( digitalRead(LS_X) == LOW )
    if ( digitalRead(DIR_X) == PLUS_X )
      limit_x_plus_active = true;
    else
      limit_x_minus_active = true;
  else {
    limit_x_plus_active = false;
    limit_x_minus_active = false;
  }
  if ( digitalRead(LS_Y) == LOW )
    if ( digitalRead(DIR_Y) == PLUS_Y )
      limit_y_plus_active = true;
    else
      limit_y_minus_active = true;
  else {
    limit_y_plus_active = false;
    limit_y_minus_active = false;
  }
  if ( digitalRead(LS_Z) == LOW )
    if ( digitalRead(DIR_Z) == PLUS_Z )
      limit_z_plus_active = true;
    else
      limit_z_minus_active = true;
  else {
    limit_z_plus_active = false;
    limit_z_minus_active = false;
  }
}
bool check_limit(int step_pin){
  switch ( step_pin ){
    case STEP_X:
      if ( ((digitalRead(DIR_X) == PLUS_X) && limit_x_plus_active) || ((digitalRead(DIR_X) == MINUS_X) && limit_x_minus_active) )
        return false;
      break;
    case STEP_Y:
      if ( ((digitalRead(DIR_Y) == PLUS_Y) && limit_y_plus_active) || ((digitalRead(DIR_Y) == MINUS_Y) && limit_y_minus_active) )
        return false;
      break;
    case STEP_Z:
      if ( ((digitalRead(DIR_Z) == PLUS_Z) && limit_z_plus_active) || ((digitalRead(DIR_Z) == MINUS_Z) && limit_z_minus_active) )
        return false;
      break;
  } 
  return true;
}
int calibrate_range(int step_pin){
  int range;
  int dir_pin, plus_dir, minus_dir, ls_pin, t_step;
  switch ( step_pin ){
    case STEP_X:
      dir_pin = DIR_X;
      plus_dir = PLUS_X;
      minus_dir = MINUS_X;
      ls_pin = LS_X;
      t_step = T_STEP_X;
      break;
    case STEP_Y:
      dir_pin = DIR_Y;
      plus_dir = PLUS_Y;
      minus_dir = MINUS_Y;
      ls_pin = LS_Y;
      t_step = T_STEP_Y;
      break;
    case STEP_Z:
      dir_pin = DIR_Z;
      plus_dir = PLUS_Z;
      minus_dir = MINUS_Z;
      ls_pin = LS_Z;
      t_step = T_STEP_Z;
      break;
  }
  change_dir(dir_pin, plus_dir);
  while ( digitalRead(ls_pin) == HIGH )
    step_pulse(step_pin, t_step/10);
  delay(T_CAL_DELAY);
  change_dir(dir_pin, minus_dir);
  while ( digitalRead(ls_pin) == LOW)
    step_pulse(step_pin, t_step/10);
  delay(T_CAL_DELAY);
  while ( digitalRead(ls_pin) == HIGH){
    step_pulse(step_pin, t_step/10);
    range += 1;
  }
  delay(T_CAL_DELAY);
  change_dir(dir_pin, plus_dir);
  while ( digitalRead(ls_pin) == LOW){
    step_pulse(step_pin, t_step/10);
    range -= 1;
  }
  delay(T_CAL_DELAY*2);
  change_dir(dir_pin, minus_dir);
  for (int i; i<range/2; i++)
    step_pulse(step_pin, t_step/10);
  return range;
}

// CNC SHIELD
void init_cnc_shield(){
  pinMode(EN_CNC, OUTPUT);
  digitalWrite(EN_CNC, LOW);
  pinMode(STEP_X, OUTPUT);
  digitalWrite(STEP_X, LOW);
  pinMode(DIR_X, OUTPUT);
  digitalWrite(DIR_X, PLUS_X);
  pinMode(STEP_Y, OUTPUT);
  digitalWrite(STEP_Y, LOW);
  pinMode(DIR_Y, OUTPUT);
  digitalWrite(DIR_Y, PLUS_Y);
  pinMode(STEP_Z, OUTPUT);
  digitalWrite(STEP_Z, LOW);
  pinMode(DIR_Z, OUTPUT);
  digitalWrite(DIR_Z, PLUS_Z);
  pinMode(LS_X, INPUT_PULLUP);
  pinMode(LS_Y, INPUT_PULLUP);
  pinMode(LS_Z, INPUT_PULLUP);
}

// BT
void control_Ps3(){
  // Rotate while pressing
  if ( Ps3.data.button.left || Ps3.data.button.right )
    step_pulse(STEP_X, T_STEP_X);
  else if ( Ps3.data.button.down || Ps3.data.button.up )
    step_pulse(STEP_Y, T_STEP_Y);
  else if ( Ps3.data.button.l2 || Ps3.data.button.r2 )
    step_pulse(STEP_Z, T_STEP_Z);
}
void ISR_Ps3(){
  // Change dir only when pressing opposite
  if ( Ps3.event.button_down.up || Ps3.event.button_down.triangle )
    change_dir(DIR_Y, PLUS_Y);
  else if ( Ps3.event.button_down.right || Ps3.event.button_down.circle )
    change_dir(DIR_X, MINUS_X);
  else if ( Ps3.event.button_down.down || Ps3.event.button_down.cross )
    change_dir(DIR_Y, MINUS_Y);
  else if ( Ps3.event.button_down.left || Ps3.event.button_down.square )
    change_dir(DIR_X, PLUS_X);
  else if ( Ps3.event.button_down.l2 || Ps3.event.button_down.l1 )
    change_dir(DIR_Z, MINUS_Z);
  else if ( Ps3.event.button_down.r2 || Ps3.event.button_down.r1 )
    change_dir(DIR_Z, PLUS_Z);
  // Change dir and move 1 step only when pressing
  // if ( Ps3.event.button_down.circle || Ps3.event.button_down.square )
  //   step_pulse(STEP_X, T_STEP_X);
  // else if ( Ps3.event.button_down.triangle || Ps3.event.button_down.cross )
  //   step_pulse(STEP_Y, T_STEP_Y);
  // else if ( Ps3.event.button_down.l1 || Ps3.event.button_down.r1 )
  //   step_pulse(STEP_Z, T_STEP_Z);
  // pid control of rpm

  // on/off cnc shield
  if ( Ps3.event.button_down.ps ){ 
    digitalWrite(EN_CNC, !digitalRead(EN_CNC));
    t_counter_calib = millis();
  }

  if ( Ps3.event.button_up.ps ){
    if ( millis() - t_counter_calib > 5000){
      range_x = calibrate_range(STEP_X);
      range_y = calibrate_range(STEP_Y);
      range_z = calibrate_range(STEP_Z);
    }
    t_counter_calib = 0;
  }
}
void init_ps3(){
  Ps3.attach(ISR_Ps3);
  Ps3.begin(MAC);
}