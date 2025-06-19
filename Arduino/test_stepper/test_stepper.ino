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


#define T_STEP_X 10
#define T_DIR_X 1000
#define T_STEP_Y 10
#define T_DIR_Y 1000
#define T_STEP_Z 10
#define T_DIR_Z 1000

void setup() {
  Serial.begin(115200);
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
  test_x();
  test_y();
  test_z();
}


void test_x() { 
  for (int i=0; i<200; i++){
    Serial.println("moviendo motor x+");
    digitalWrite(STEP_X, 1);
    delay(T_STEP_X/2);
    digitalWrite(STEP_X, 0);
    delay(T_STEP_X/2);
  };
  delay(T_DIR_X/2);
  digitalWrite(DIR_X, MINUS_X);
  delay(T_DIR_X/2);
  for (int i=0; i<200; i++){
    Serial.println("moviendo motor x-");
    digitalWrite(STEP_X, 1);
    delay(T_STEP_X/2);
    digitalWrite(STEP_X, 0);
    delay(T_STEP_X/2);
  };
}

void test_y() { 
  for (int i=0; i<200; i++){
    Serial.println("moviendo motor y+");
    digitalWrite(STEP_Y, 1);
    delay(T_STEP_Y/2);
    digitalWrite(STEP_Y, 0);
    delay(T_STEP_Y/2);
  };
  delay(T_DIR_Y/2);
  digitalWrite(DIR_Y, MINUS_Y);
  delay(T_DIR_Y/2);
  for (int i=0; i<200; i++){
    Serial.println("moviendo motor y-");
    digitalWrite(STEP_Y, 1);
    delay(T_STEP_Y/2);
    digitalWrite(STEP_Y, 0);
    delay(T_STEP_Y/2);
  };
}

void test_z() { 
  for (int i=0; i<200; i++){
    digitalWrite(STEP_Z, 1);
    delay(T_STEP_Z/2);
    digitalWrite(STEP_Z, 0);
    delay(T_STEP_Z/2);
  };
  delay(T_DIR_Z/2);
  digitalWrite(DIR_Z, MINUS_Z);
  delay(T_DIR_Z/2);
  for (int i=0; i<200; i++){
    digitalWrite(STEP_Z, 1);
    delay(T_STEP_Z/2);
    digitalWrite(STEP_Z, 0);
    delay(T_STEP_Z/2);
  };
}