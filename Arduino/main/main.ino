//---------LIBRARIES-----------
#include <Ps3Controller.h>
#include "MPU9250.h"
#include <BluetoothSerial.h>

//---------DEFINITIONS-----------
// SERIAL BT
#define SERIAL 115200
#define DEVICE_NAME "ROBOTITO"
#define BOOT_MSG "HOLA"
// CONTROLLER
#define MAC "90:34:fc:0f:69:75"
#define T_VIB 500 // ms
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
#define X_PITCH 0.1286  //°
#define Y_PITCH 0.2826 //mm
#define Z_PITCH 0.015 //mm
// STEPERS VEL (T = RPM * 3e5)
#define T_STEP_X 5000 //us
#define T_STEP_Y 3000 //us
#define T_STEP_Z 1000 //us
#define T_DIR 500   //ms
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
#define TS 50  // ms
#define KP 1.2f
#define KI 1.5f
#define KD 0.0f

//---------OBJECTS-----------
MPU9250 Mpu;
BluetoothSerial SerialBT;

//---------VARIABLES-----------
// Limit Switch variables 
bool limit_x_plus_on = false;
bool limit_x_minus_on = false;
bool limit_y_plus_on = false;
bool limit_y_minus_on = false;
bool limit_z_plus_on = false;
bool limit_z_minus_on = false;
// PID CONTROL
volatile long encoder_count;
long theta, theta_prev;
float rpm;
float e, e_prev, e_prev_prev;
unsigned long t, t_prev;
float dt;
int pwm_val, pwm_val_prev;
int rpm_values[3] = {0, 400, -400};
int rpm_val_index;
int rpm_ref = rpm_values[0];
// Stepers
float state[3] = {0.0, 0.0, 0.0};
int range_x, range_y, range_z;
unsigned long t_counter_calib;

//---------SETUP-----------
void setup() {
  init_bt();
  init_cnc_shield();
  init_dc_motor();
  init_imu();
  init_ps3();
  t_prev = millis();
}

//---------LOOP-----------
void loop() {
  check_limit_switch();

  control_ps3();
  
  t = millis();
  if ( t - t_prev >= TS ){
    noInterrupts();
    theta = encoder_count;
    interrupts();
    dt = t - t_prev;
    rpm = (theta - theta_prev) * (60000/dt) / NFactor;
    e = rpm_ref - rpm;
    pwm_val = int(pwm_val_prev
                  + (KP + TS*KI/1000 + 1000*KD/TS) * e
                  + (-KP-2000*KD/TS) * e_prev
                  + (1000*KD/TS) * e_prev_prev);
    pwm_val = constrain(pwm_val, -255, 255);
    write_hbridge(pwm_val);
    theta_prev = theta;
    e_prev = e;
    e_prev_prev = e_prev;
    pwm_val_prev = pwm_val;
    t_prev = t;
  }
}


//---------FUNCTIONS-----------
// STEPPERS
void step_pulse(int step_pin, int t_pulse){
  if ( is_limit_on(step_pin) )
    return;
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(t_pulse/2);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(t_pulse/2);
  switch ( step_pin ){
    case STEP_X:
      state[0] += (digitalRead(DIR_X) == PLUS_X) ? X_PITCH : -X_PITCH;
      SerialBT.println("MOVING STEPPER X");
      break;
    case STEP_Y:
      state[1] += (digitalRead(DIR_Y) == PLUS_Y) ? Y_PITCH : -Y_PITCH;
      SerialBT.println("MOVING STEPPER Y");
      break;
    case STEP_Z:
      state[2] += (digitalRead(DIR_Z) == PLUS_Z) ? Z_PITCH : -Z_PITCH;
      SerialBT.println("MOVING STEPPER Z");
      break;
  }
}
void change_dir(int dir_pin, bool dir_val){
  if ( digitalRead(dir_pin) != dir_val ){
    delay(T_DIR/2);
    switch ( dir_pin ){
      case DIR_X:
        if ( dir_val ==  PLUS_X)
          SerialBT.println("CHANGE DIR_X TO X+");
        else
          SerialBT.println("CHANGE DIR_X TO X-");
        break;
      case DIR_Y:
        if ( dir_val ==  PLUS_Y)
          SerialBT.println("CHANGE DIR_Y TO Y+");
        else
          SerialBT.println("CHANGE DIR_Y TO Y-");
        break;
      case DIR_Z:
        if ( dir_val ==  PLUS_Z)
          SerialBT.println("CHANGE DIR_Z TO Z+");
        else
          SerialBT.println("CHANGE DIR_Z TO Z-");
        break;
    }
    digitalWrite(dir_pin, dir_val);
    delay(T_DIR/2);
  }
}

//  LIMIT SWITCH
void check_limit_switch(){
  limit_x_plus_on = false;
  limit_x_minus_on = false;
  limit_y_plus_on = false;
  limit_y_minus_on = false;
  limit_z_plus_on = false;
  limit_z_minus_on = false;
  if ( digitalRead(DIR_X) == PLUS_X && digitalRead(LS_X) == LOW ){
    limit_x_plus_on = true;
    SerialBT.println("LIMIT SWITCH TRIGGERED: X+");
  }
  if ( digitalRead(DIR_X) == MINUS_X && digitalRead(LS_X) == LOW ){
    limit_x_minus_on = true;
    SerialBT.println("LIMIT SWITCH TRIGGERED: X-");
  }
  if ( digitalRead(DIR_Y) == PLUS_Y && digitalRead(LS_Y) == LOW ){
    limit_y_plus_on = true;
    SerialBT.println("LIMIT SWITCH TRIGGERED: Y+");
  }
  if ( digitalRead(DIR_Y) == MINUS_Y && digitalRead(LS_Y) == LOW ){
    limit_y_minus_on = true;
    SerialBT.println("LIMIT SWITCH TRIGGERED: Y-");
  }
  if ( digitalRead(DIR_Z) == PLUS_Z && digitalRead(LS_Z) == LOW ){
    limit_z_plus_on = true;
    SerialBT.println("LIMIT SWITCH TRIGGERED: Z+");
  }
  if ( digitalRead(DIR_Z) == MINUS_Z && digitalRead(LS_Z) == LOW ){
    limit_z_minus_on = true;
    SerialBT.println("LIMIT SWITCH TRIGGERED: Z-");
  }
}
bool is_limit_on(int step_pin) {
  int dir_pin, dir_val, dir_plus, dir_minus;
  bool limit_plus, limit_minus;
  switch ( step_pin ){
    case STEP_X:
      dir_pin = DIR_X;
      dir_plus = PLUS_X;
      dir_minus = MINUS_X;
      limit_plus = limit_x_plus_on;
      limit_minus = limit_x_minus_on;
      break;
    case STEP_Y:
      dir_pin = DIR_Y;
      dir_plus = PLUS_Y;
      dir_minus = MINUS_Y;
      limit_plus = limit_y_plus_on;
      limit_minus = limit_y_minus_on;
      break;
    case STEP_Z:
      dir_pin = DIR_Z;
      dir_plus = PLUS_Z;
      dir_minus = MINUS_Z;
      limit_plus = limit_z_plus_on;
      limit_minus = limit_z_minus_on;
      break;
  }
  dir_val = digitalRead(dir_pin);
  if ( (dir_val == dir_plus && limit_plus) || (dir_val == dir_minus && limit_minus) )
    return true;
  return false;
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
  delay(T_CAL_DELAY);
  return range;
}

// CNC SHIELD
void turn_power_cnc_shield(){
  digitalWrite(EN_CNC, !digitalRead(EN_CNC));
  if ( digitalRead(EN_CNC) == LOW )
    SerialBT.println("CNC SHIELD POWERED ON");
  else 
    SerialBT.println("CNC SHIELD POWERED OFF");
}
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
  SerialBT.println("CNC SHIELD INITIATED");
}

// CONTROLLER
void control_ps3(){
  // Rotate while pressing
  if ( Ps3.data.button.left || Ps3.data.button.right )
    step_pulse(STEP_X, T_STEP_X);
  else if ( Ps3.data.button.down || Ps3.data.button.up )
    step_pulse(STEP_Y, T_STEP_Y);
  else if ( Ps3.data.button.l2 || Ps3.data.button.r2 )
    step_pulse(STEP_Z, T_STEP_Z);
}
void isr_ps3(){
  // Change dir only when pressing opposite
  if ( Ps3.event.button_down.up )
    change_dir(DIR_Y, PLUS_Y);
  else if ( Ps3.event.button_down.right )
    change_dir(DIR_X, MINUS_X);
  else if ( Ps3.event.button_down.down )
    change_dir(DIR_Y, MINUS_Y);
  else if ( Ps3.event.button_down.left )
    change_dir(DIR_X, PLUS_X);
  else if ( Ps3.event.button_down.l2 )
    change_dir(DIR_Z, MINUS_Z);
  else if ( Ps3.event.button_down.r2 )
    change_dir(DIR_Z, PLUS_Z);

  // Change dir and move 1 step only when pressing
  // if ( Ps3.event.button_down.circle || Ps3.event.button_down.square )
  //   step_pulse(STEP_X, T_STEP_X);
  // else if ( Ps3.event.button_down.triangle || Ps3.event.button_down.cross )
  //   step_pulse(STEP_Y, T_STEP_Y);
  // else if ( Ps3.event.button_down.l1 || Ps3.event.button_down.r1 )
  //   step_pulse(STEP_Z, T_STEP_Z);

  // pid control of rpm
  if ( Ps3.event.button_down.start )
    update_rpm_ref();
  else if ( Ps3.event.button_down.select )
    reset_rpm_ref();

  // on/off cnc shield
  if ( Ps3.event.button_down.ps ){
    turn_power_cnc_shield();
    t_counter_calib = 0;
  }
  // calibrate if hold & release 5s
  if ( Ps3.event.button_up.ps ){
    if ( millis() - t_counter_calib > 5000){
      range_x = calibrate_range(STEP_X);
      range_y = calibrate_range(STEP_Y);
      range_z = calibrate_range(STEP_Z);
    }
    t_counter_calib = 0;
  }
}
void ps3_on_connect(){
  Ps3.setRumble(100.0, T_VIB);
  delay(T_VIB);
  Ps3.setRumble(0.0);
  SerialBT.println("CONTROLLER CONNECTED");
}
void init_ps3(){
  Ps3.attach(isr_ps3);
  Ps3.attachOnConnect(ps3_on_connect);
  Ps3.begin(MAC);
  SerialBT.println("CONTROLLER INITIATED");
}

// DC MOTOR
void isr_encoder1(){
  bool PinA = digitalRead(ENCA);
  bool PinB = digitalRead(ENCB);
  encoder_count += (PinA == PinB) ? 1 : -1; 
}
void isr_encoder2(){
  bool PinA = digitalRead(ENCA);
  bool PinB = digitalRead(ENCB);
  encoder_count += (PinA == PinB) ? -1 : 1;
}
void write_hbridge(int PWM_val){
  int duty = abs(PWM_val);
  if ( duty > 0 ){
    digitalWrite(IN1, PWM_val > 0);
    digitalWrite(IN2, PWM_val < 0);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(ENA, duty);
  SerialBT.print("H-BRIDGE PWM WRITED: ");
  SerialBT.println(PWM_VAL);
}
void update_rpm_ref(){
  rpm_val_index = (rpm_val_index + 1) % 3;
  rpm_ref = rpm_values[rpm_val_index];
  SerialBT.print("DRILL RPM SETTED TO: ");
  SerialBT.println(rpm_ref);
  // if ( rpm_ref == rpm_values[0])
  //   write_hbridge(0);
  // if ( rpm_ref == rpm_values[1])
  //   write_hbridge(150);
  // if ( rpm_ref == rpm_values[2])
  //   write_hbridge(255);
}
void reset_rpm_ref(){
  rpm_val_index = 0;
  rpm_ref = rpm_values[rpm_val_index];
  SerialBT.println("DRILL RPM RESETED");
}
void init_dc_motor(){
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  pinMode(IN1, OUTPUT);
  digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN2, LOW);
  pinMode(ENCA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), isr_encoder1, CHANGE);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCB), isr_encoder2, CHANGE);
  SerialBT.println("DC MOTOR INITIATED");
}

// IMU
void calibrate_imu(){
  Mpu.verbose(true);
  delay(1000);
  Mpu.calibrateAccelGyro();
  delay(1000);
  Mpu.calibrateMag();
  print_calibration();
  Mpu.verbose(false);
}
void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(Mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(Mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(Mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(Mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(Mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(Mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(Mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(Mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(Mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(Mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(Mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(Mpu.getMagScaleZ());
    Serial.println();
}
void print_data(){
  if ( Mpu.update() ){
    Serial.print("Accx:");
    Serial.print(Mpu.getAccX());
    Serial.print(",");
    Serial.print("Accy:");
    Serial.print(",");
    Serial.print(Mpu.getAccY());
    Serial.print(",");
    Serial.print("Accz:");
    Serial.print(Mpu.getAccZ());
    Serial.print(",");
    Serial.print("Gyrox:");
    Serial.print(Mpu.getGyroX());
    Serial.print(",");
    Serial.print("Gyroy:");
    Serial.print(Mpu.getGyroY());
    Serial.print(",");
    Serial.print("Gyroz:");
    Serial.print(Mpu.getGyroZ());
    Serial.println(",");
  }
}
void init_imu(){
  Wire.begin();
  if (!Mpu.setup(0x68)) { 
    SerialBT.println("MPU CONNECTION FAILED");
    return;
  }
  SerialBT.println("MPU INITIATED");
}

// BT
void init_bt(){
  Serial.begin(SERIAL);
  SerialBT.begin(DEVICE_NAME);
  SerialBT.println(BOOT_MSG);
}