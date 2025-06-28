#define ENA 14
#define IN1 27
#define IN2 26
#define PWM_FREQ 1000 // hz
#define PWM_RES 8

#define ENCA 35
#define ENCB 34
#define NFactor 400.0f
#define T_S 50  // ms
#define KP 1.2f
#define KI 1.5f
#define KD 0.0f

volatile long EncoderCount = 0;

long Theta, Theta_prev;
float RPM;
int PWM_val, PWM_val_prev;

float e, e_prev, e_prev_prev;
unsigned long t, t_prev;
float dt;

int RPM_ref;



void setup() {
  Serial.begin(115200);
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), ISR_EncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), ISR_EncoderA2, CHANGE);
  t_prev = millis();
  RPM_ref = -400;
}
unsigned long t_last_ref_update = 0;  // global
const unsigned long REF_UPDATE_INTERVAL = 5000; // 10 s

void loop() {
    t = millis();

  // Aumenta RPM_ref cada 10 segundos
  if (t - t_last_ref_update >= REF_UPDATE_INTERVAL) {
    RPM_ref += 100;
    t_last_ref_update = t;
  }
  if ( t - t_prev >= T_S){
    dt = t - t_prev;
    noInterrupts();
    Theta = EncoderCount;
    interrupts();
    RPM = (Theta - Theta_prev) * (60000/dt) / NFactor;
    e = RPM_ref - RPM;

    PWM_val = int(PWM_val_prev + (KP + T_S*KI/1000 + 1000*KD/T_S) * e + (-KP-2000*KD/T_S)*e_prev + (1000*KD/T_S)*e_prev_prev);
    if (PWM_val > 255) PWM_val = 255;
    else if (PWM_val < -255) PWM_val = -255;

    WriteDriver(PWM_val);

    Theta_prev = Theta;
    e_prev = e;
    e_prev_prev = e_prev;
    PWM_val_prev = PWM_val;
    t_prev = t;
    Serial.print("Theta:");
    Serial.print(Theta);
    Serial.print(",RPM_REF:");
    Serial.print(RPM_ref);
    Serial.print(",RPM:");
    Serial.print(RPM);
    Serial.print(",PWM:");
    Serial.println(PWM_val);
  }
  // WriteDriver(100);


}


void ISR_EncoderA2(){
  bool PinB = digitalRead(ENCB);
  bool PinA = digitalRead(ENCA);
  if ( PinB == LOW )
    if ( PinA == HIGH )
      EncoderCount++;
    else
      EncoderCount--;
  else
    if ( PinA == HIGH )
      EncoderCount--;
    else
      EncoderCount++;
}
void ISR_EncoderA1(){
  bool PinB = digitalRead(ENCB);
  bool PinA = digitalRead(ENCA);
  if ( PinA == LOW )
    if ( PinB == HIGH )
      EncoderCount--;
    else
      EncoderCount++;
  else
    if ( PinB == HIGH )
      EncoderCount++;
    else
      EncoderCount--; 
}
void WriteDriver(int PWM_val){
  int duty = min( abs(PWM_val), 255);
  if ( PWM_val > 0 ){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
  } else if ( PWM_val < 0 ){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
  } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
  }
  ledcWrite(ENA, duty);
}