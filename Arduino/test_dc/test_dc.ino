#define ENA 14
#define IN1 27
#define IN2 27
#define PWM_FREQ 1000 // hz
#define PWM_RES 8

#define ENCA 35
#define ENCB 34
#define NFactor 1
#define T_S 100  // ms
#define KP 1.0f
#define KI 0.5f
#define KD 0.0f

volatile long EncoderCount = 0;

long Theta, Theta_prev;
float RPM;
int PWM_val;

float e, e_prev, inte, deriv;
unsigned long dt, t, t_prev;

int RPM_ref = 0;


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
}

void loop() {
  t = millis();
  if ( t - t_prev >= T_S){
    dt = t - t_prev;
    noInterrupts();
    Theta = EncoderCount;
    interrupts();
    RPM = (Theta - Theta_prev) * (60000/dt) / NFactor;
    e = RPM_ref - RPM;
    inte += (e + e_prev) * (dt/2000);
    deriv = (e - e_prev) * (1000/dt);
    PWM_val = int(KP*e + KI*inte + KD*deriv);
    if (PWM_val > 255) PWM_val = 255;
    else if (PWM_val < -255) PWM_val = -255;
    WriteDriverV(PWM_val);
    Theta_prev = Theta;
    e_prev = e;
    t_prev = dt;
    Serial.print("RPM_REF: ");
    Serial.print(RPM_ref);
    Serial.print("  | RPM: ");
    Serial.print(RPM);
    Serial.print("  | PWM: ");
    Serial.println(PWM_val);
  }
  noInterrupts();
  Serial.println(EncoderCount);
  interrupts();
  delay(100);

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
void WriteDriverV(int PWM_val){
  int duty = min( abs(PWM_val), 250);
  if ( PWM_val > 0 ){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
  } else if ( PWM_val < 0 ){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
  } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
  }
  ledcWrite(ENA, duty);
}