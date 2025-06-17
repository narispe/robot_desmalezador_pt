#include <Ps3Controller.h>

//---------DEFINITIONS-----------

// SERIAL
#define SERIAL 115200
// BT
#define MAC "90:34:fc:0f:69:75"
// STEPPER X
#define STEP_X 23
#define DIR_X 22
#define PLUS_X 1
#define MINUS_X 0
#define X_PER_STEP 0.1286 //°
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
#define T_INIT 2000 //ms
#define T_STEP 1000 //us
#define T_DIR 200   //ms
#define STEP_PER_REV 200
// LIMIT SWITCH
#define LS_X 32
#define LS_Y 33
#define LS_Z 25
// PUENTE H
#define ENA 14
#define IN1 27
#define IN2 26
#define PWM_FREQ 1000 // hz
#define PWM_RES 8
// ENCODER
#define V3 12
#define ENCA 35
#define ENCB 34
#define NFactor 400.0f
// PID
#define T_S 50  // ms
#define KP 0.9f
#define KI 0.4f
#define KD 0.0f


//---------VARIABLES-----------
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
  // cnc shield
  for ( int pin : {STEP_X, DIR_X, STEP_Y, DIR_Y, STEP_Z, DIR_Z} ){
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  digitalWrite(DIR_X, PLUS_X);
  digitalWrite(DIR_Y, PLUS_Y);
  digitalWrite(DIR_Z, PLUS_Z);
  // limit switch
  pinMode(LS_X, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LS_X), ISR_LimitSwitchX, CHANGE);
  pinMode(LS_Y, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LS_Y), ISR_LimitSwitchY, CHANGE);
  pinMode(LS_Z, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LS_Z), ISR_LimitSwitchZ, CHANGE);
  // puente h
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  for ( int pin : {IN1, IN2} ){
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  // encoder
  pinMode(V3, OUTPUT);
  digitalWrite(V3, HIGH);
  pinMode(ENCA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), ISR_Encoder1, CHANGE);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCB), ISR_Encoder2, CHANGE);
  // bt
  Ps3.attach(ISR_Ps3);
  Ps3.begin(MAC);
  t_prev = millis();
}


//---------LOOP-----------
void loop() {
  // Rotate while pressing
  if ( Ps3.data.button.left || Ps3.data.button.right )
    step_pulse(STEP_X);
  else if ( Ps3.data.button.down || Ps3.data.button.up )
    step_pulse(STEP_Y);
  else if ( Ps3.data.button.l2 || Ps3.data.button.r2 )
    step_pulse(STEP_Z);

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
// STEPPER
void step_pulse(int step_pin){
  if ( step_pin == STEP_X )
    STATE[0] += (DIR_X == PLUS_X) ? X_PER_STEP : -X_PER_STEP;
  else if ( step_pin == STEP_Y )
    STATE[1] += (DIR_Y == PLUS_Y) ? Y_PER_STEP : -Y_PER_STEP;
  else if ( step_pin == STEP_Z )
    STATE[2] += (DIR_Z == PLUS_Z) ? Z_PER_STEP : -Z_PER_STEP;
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(T_STEP/2);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(T_STEP/2);
}
void change_dir(int dir_pin, bool dir_val){
  if ( digitalRead(dir_pin) != dir_val){
    delay(T_DIR/2);
    digitalWrite(dir_pin, dir_val);
    delay(T_DIR/2);
  }
}
// BT
void ISR_Ps3(){
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
  // pid control of rpm
  else if ( Ps3.event.button_down.start ) {
    rpm_val_index = (rpm_val_index + 1) % 3;
    rpm_ref = rpm_values[rpm_val_index];
    inte = 0;
  }
  else if ( Ps3.event.button_down.select ) {
    rpm_val_index = 0;
    rpm_ref = rpm_values[rpm_val_index];
    inte = 0;
  }
}
// ENCODER
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
    digitalWrite(IN1, PWM_val > 0);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(ENA, duty);
}
void ISR_LimitSwitchX(){
  if (digitalRead(LS_X) == LOW){
    Serial.println("LIMIT SWITCH X ACTIVADO");
  } else {
    Serial.println("LIMIT SWITCH X DESACTIVADO");
  }
}
void ISR_LimitSwitchY(){
  if (digitalRead(LS_Y) == LOW){
    Serial.println("LIMIT SWITCH Y ACTIVADO");
  } else {
    Serial.println("LIMIT SWITCH Y DESACTIVADO");
  }
}
void ISR_LimitSwitchZ(){
  if (digitalRead(LS_Z) == LOW){
    Serial.println("LIMIT SWITCH Z ACTIVADO");
  } else {
    Serial.println("LIMIT SWITCH Y DESACTIVADO");
  }
}