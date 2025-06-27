#include <Ps3Controller.h>

//---------DEFINITIONS-----------
// SERIAL
#define SERIAL 115200
// BT
#define MAC "90:34:fc:0f:69:75"
// CNC SHIELD
#define EN_CNC 0
// LIMIT SWITCH
#define LS_X 32
#define LS_Y 33
#define LS_Z 25
// STEPPER X
#define STEP_X 19
#define DIR_X 18
#define PLUS_X 1
#define MINUS_X 0
#define X_PER_STEP 0.1286  //°
#define T_STEP_X 1000 //us
// STEPPER Y
#define STEP_Y 5
#define DIR_Y 17
#define PLUS_Y 0
#define MINUS_Y 1
#define Y_PER_STEP 0.2826 //mm
#define T_STEP_Y 1000 //us
// STEPPER Z
#define STEP_Z 16
#define DIR_Z 14
#define PLUS_Z 1
#define MINUS_Z 0
#define Z_PER_STEP 0.015 //mm
#define T_STEP_Z 1000 //us
// STEPPER
#define T_STEP 1000 //us
#define T_DIR 200   //ms
#define STEP_PER_REV 200
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
#define KP 0.9f
#define KI 0.4f
#define KD 0.0f

//---------VARIABLES-----------
// Limit Switch variables 
bool limit_x_plus_active = false;
bool limit_x_minus_active = false;
bool limit_y_plus_active = false;
bool limit_y_minus_active = false;
bool limit_z_plus_active = false;
bool limit_z_minus_active = false;
volatile long encoder_count;
long theta, theta_prev;
float rpm;
float e, e_prev, inte, deriv;
unsigned long t, t_prev;
float dt;
int pwm_val;
int rpm_values[3] = {0, 200, 400};
int rpm_val_index;
int rpm_ref = rpm_values[0];
float STATE[3] = {0.0, 0.0, 0.0};

//---------SETUP-----------
void setup() {
  Serial.begin(SERIAL);
  init_cnc_shield();
  init_dc_motor();
  init_ps3();
  t_prev = millis();
}

//---------LOOP-----------
void loop() {
  check_switches();

  control_Ps3();
  
  t = millis();
  if ( t - t_prev >= T_S ){
    noInterrupts();
    theta = encoder_count;
    interrupts();
    dt = t - t_prev;
    rpm = (theta - theta_prev) * (60000/dt) / NFactor;
    e = rpm_ref - rpm;
    inte += (e + e_prev) * (dt/2000);
    deriv = (e - e_prev) * (1000/dt);
    pwm_val = int(KP*e + KI*inte + KD*deriv);
    pwm_val = constrain(pwm_val, -255, 255);
    WriteDriver(pwm_val);
    theta_prev = theta;
    e_prev = e;
    t_prev = t;
  }
}


//---------FUNCTIONS-----------
// STEPPERS
void step_pulse(int step_pin, int t_pulse){
  if ( !check_limit(step_pin) )
    return;
  switch ( step_pin ){
    case STEP_X:
      STATE[0] += (digitalRead(DIR_X) == PLUS_X) ? X_PER_STEP : -X_PER_STEP; break;
    case STEP_Y:
      STATE[1] += (digitalRead(DIR_Y) == PLUS_Y) ? Y_PER_STEP : -Y_PER_STEP; break;
    case STEP_Z:
      STATE[2] += (digitalRead(DIR_Z) == PLUS_Z) ? Z_PER_STEP : -Z_PER_STEP; break;
  }
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(t_pulse/2);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(t_pulse/2);
}
void change_dir(int dir_pin, bool dir_val){
  if ( digitalRead(dir_pin) != dir_val ){
    switch ( dir_pin ){
      case DIR_X:
        digitalWrite(DIR_X, dir_val); break;
      case DIR_Y:
        digitalWrite(DIR_Y, dir_val); break;
      case DIR_Z:
        digitalWrite(DIR_Z, dir_val); break;
    }
    delay(T_DIR);
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
  if ( Ps3.event.button_down.circle || Ps3.event.button_down.square )
    step_pulse(STEP_X, T_STEP_X);
  else if ( Ps3.event.button_down.triangle || Ps3.event.button_down.cross )
    step_pulse(STEP_Y, T_STEP_Y);
  else if ( Ps3.event.button_down.l1 || Ps3.event.button_down.r1 )
    step_pulse(STEP_Z, T_STEP_Z);
  // pid control of rpm
  if ( Ps3.event.button_down.start ) {
    rpm_val_index = (rpm_val_index + 1) % 3;
    rpm_ref = rpm_values[rpm_val_index];
    inte = 0;
  }
  else if ( Ps3.event.button_down.select ) {
    rpm_val_index = 0;
    rpm_ref = rpm_values[rpm_val_index];
    inte = 0;
  }
  // on/off cnc shield
  if ( Ps3.event.button_down.ps )
    digitalWrite(EN_CNC, !digitalRead(EN_CNC));
}
void init_ps3(){
  Ps3.attach(ISR_Ps3);
  Ps3.begin(MAC);
}

// DC MOTOR
void ISR_Encoder1(){
  bool PinA = digitalRead(ENCA);
  bool PinB = digitalRead(ENCB);
  encoder_count += (PinA == PinB) ? 1 : -1; 
}
void ISR_Encoder2(){
  bool PinA = digitalRead(ENCA);
  bool PinB = digitalRead(ENCB);
  encoder_count += (PinA == PinB) ? -1 : 1;
}
void WriteDriver(int PWM_val){
  int duty = min( abs(PWM_val), 255);
  if ( duty > 0) {
    digitalWrite(IN1, PWM_val < 0);
    digitalWrite(IN2, PWM_val > 0);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(ENA, duty);
}
void init_dc_motor(){
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  pinMode(IN1, OUTPUT);
  digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN2, LOW);
  pinMode(ENCA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), ISR_Encoder1, CHANGE);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCB), ISR_Encoder2, CHANGE);
}