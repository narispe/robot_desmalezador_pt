#include <Ps3Controller.h>

//---------DEFINITIONS-----------

// SERIAL
#define SERIAL 115200
// BT
#define MAC "90:34:fc:0f:69:75"
// LIMIT SWITCH
#define LS_X 32
#define LS_Y 33
#define LS_Z 25
// STEPPER X
#define STEP_X 19
#define DIR_X 18
#define PLUS_X 1
#define MINUS_X 0
#define X_PER_STEP 1.8  //°
// STEPPER Y
#define STEP_Y 5
#define DIR_Y 17
#define PLUS_Y 0
#define MINUS_Y 1
#define Y_PER_STEP 0.2826 //mm
// STEPPER Z
#define STEP_Z 16
#define DIR_Z 14
#define PLUS_Z 1
#define MINUS_Z 0
#define Z_PER_STEP 0.015 //mm
// STEPPER
#define T_INIT 2000  //ms
#define T_STEP 1000  //us
#define T_DIR 200  //ms
#define STEP_PER_REV 200
float STATE[3];
// Limit Switch variables 
bool limit_x_plus_active = false;
bool limit_x_minus_active = false;
bool limit_y_plus_active = false;
bool limit_y_minus_active = false;
bool limit_z_plus_active = false;
bool limit_z_minus_active = false;


//---------SETUP-----------
void setup() {
  Serial.begin(SERIAL);
  // cnc shield
  for (int DIR : {STEP_X, DIR_X, STEP_Y, DIR_Y, STEP_Z, DIR_Z}){
    pinMode(DIR, OUTPUT);
    digitalWrite(DIR, LOW);
  }
  digitalWrite(DIR_X, PLUS_X);
  digitalWrite(DIR_Y, PLUS_Y);
  digitalWrite(DIR_Z, PLUS_Z);
  // limit switch
  pinMode(LS_X, INPUT_PULLUP);
  pinMode(LS_Y, INPUT_PULLUP);
  pinMode(LS_Z, INPUT_PULLUP);
  // bt
  Ps3.attach(notify);
  Ps3.begin(MAC);
}


//---------LOOP-----------
void loop() {
  // Rotate while 
  
  check_switchs();


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

  // change_dir(DIR_X, PLUS_X);
  // for (int i; i<200; i++)
  //   step_pulse(DIR_X);
  // change_dir(DIR_Y, PLUS_Y);
  // for (int i; i<200; i++)
  //   step_pulse(DIR_Y);
  // change_dir(DIR_Z, PLUS_Z);
  // for (int i; i<200; i++)
  //   step_pulse(DIR_Z);
  // change_dir(DIR_X, MINUS_X);
  // for (int i; i<200; i++)
  //   step_pulse(DIR_X);
  // change_dir(DIR_Y, MINUS_Y);
  // for (int i; i<200; i++)
  //   step_pulse(DIR_Y);
  // change_dir(DIR_Z, MINUS_X);
  // for (int i; i<200; i++)
  //   step_pulse(DIR_Z);
}


//---------FUNCTIONS-----------
// STEPPER
void step_pulse(int step_DIR){
  // Checkeo si me puedo mover 
  if ( !check_limit(step_DIR) ){
    Serial.println("Movimiento bloqueado por limit switch");
    return;
  }
  if ( step_DIR == STEP_X )
    STATE[0] += (DIR_X == PLUS_X) ? X_PER_STEP : -X_PER_STEP;
  else if ( step_DIR == STEP_Y )
    STATE[1] += (DIR_Y == PLUS_Y) ? Y_PER_STEP : -Y_PER_STEP;
  else if ( step_DIR == STEP_Z )
    STATE[2] += (DIR_Z == PLUS_Z) ? Z_PER_STEP : -Z_PER_STEP;
  digitalWrite(step_DIR, HIGH);
  delayMicroseconds(T_STEP/2);
  digitalWrite(step_DIR, LOW);
  delayMicroseconds(T_STEP/2);
}
void change_dir(int dir_DIR, bool dir_val){
  if ( digitalRead(dir_DIR) != dir_val){
    delay(T_DIR/2);
    digitalWrite(dir_DIR, dir_val);
    delay(T_DIR/2);
  }
}
// BT
void notify(){
  // Change dir only when pressing opposite
  if ( Ps3.event.button_down.up )
    change_dir(DIR_Y, PLUS_Y);
  else if ( Ps3.event.button_down.right )
    change_dir(DIR_X, PLUS_X);
  else if ( Ps3.event.button_down.down )
    change_dir(DIR_Y, MINUS_Y);
  else if ( Ps3.event.button_down.left )
    change_dir(DIR_X, MINUS_X);
  else if ( Ps3.event.button_down.l2 )
    change_dir(DIR_Z, MINUS_Z);
  else if ( Ps3.event.button_down.r2 )
    change_dir(DIR_Z, PLUS_Z);
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
// LIMIT SWITCH
void check_switchs(){
  if ( digitalRead(LS_X) == LOW ){
    if ( digitalRead(DIR_X) == PLUS_X ){
      limit_x_plus_active = true;
      Serial.println("LIMIT SWITCH X+ ACTIVADO");
    } else {
      limit_x_minus_active = true;
      Serial.println("LIMIT SWITCH X- ACTIVADO");
    }
  } else {
    limit_x_plus_active = false;
    limit_x_minus_active = false;
    Serial.println("LIMIT SWITCH X DESACTIVADO");
  }
  if ( digitalRead(LS_Y) == LOW ){
    if ( digitalRead(DIR_Y) == PLUS_Y ) {
      limit_y_plus_active = true;
      Serial.println("LIMIT SWITCH Y+ ACTIVADO");
    } else {
      limit_y_minus_active = true;
      Serial.println("LIMIT SWITCH Y- ACTIVADO");
    }
  } else {
    limit_y_plus_active = false;
    limit_y_minus_active = false;
    Serial.println("LIMIT SWITCH Y DESACTIVADO");
  }
  if ( digitalRead(LS_Z) == LOW ){
    if ( digitalRead(DIR_Z) == PLUS_Z ){
      limit_z_plus_active = true;
      Serial.println("LIMIT SWITCH Z+ ACTIVADO");
    } else {
      limit_z_minus_active = true;
      Serial.println("LIMIT SWITCH Z- ACTIVADO");
    }
  } else {
    limit_z_plus_active = false;
    limit_z_minus_active = false;
    Serial.println("LIMIT SWITCH Z DESACTIVADO");
  }
}
bool check_limit(int step_DIR) {
  if (step_DIR == STEP_X) {
    if ( (digitalRead(DIR_X) == PLUS_X) && limit_x_plus_active ) {
      Serial.println("Bloqueado: X en dirección +");
      return false;
    }
    if ( (digitalRead(DIR_X) == MINUS_X) && limit_x_minus_active ) {
      Serial.println("Bloqueado: X en dirección -");
      return false;
    }
  } else if (step_DIR == STEP_Y) {
    if ( (digitalRead(DIR_Y) == PLUS_Y) && limit_y_plus_active ) {
      Serial.println("Bloqueado: Y en dirección +");
      return false;
    }
    if ( (digitalRead(DIR_Y) == MINUS_Y) && limit_y_minus_active ) {
      Serial.println("Bloqueado: Y en dirección -");
      return false;
    }
  } else if (step_DIR == STEP_Z) {
    if ( (digitalRead(DIR_Z) == PLUS_Z) && limit_z_plus_active ) {
      Serial.println("Bloqueado: Z en dirección +");
      return false;
    }
    if ( (digitalRead(DIR_Z) == MINUS_Z) && limit_z_minus_active ) {
      Serial.println("Bloqueado: Z en dirección -");
      return false;
    }
  }
  return true;
}
