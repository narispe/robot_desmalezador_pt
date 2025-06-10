#define ENA 14
#define IN1 27
#define IN2 27
#define ENCA 35
#define ENCB 34
#define T_S 1000  // us
#define NFactor 1

volatile long EncoderCount = 0;
float Theta, Theta_prev;
float RPM;
int PWM_val;

float e, e_prev;
float inte, inte_prev, deriv;
unsigned long dt, t, t_prev;

int RPM_ref = 0;
float kp = 1;
float ki = 0.5;
float kd = 10;

void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), ISR_EncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), ISR_EncoderA2, CHANGE);
  t_prev = micros();
}

void loop() {
  if ( (micros() - t_prev) >= T_S){
    t = micros();

    Theta = EncoderCount;
    dt = t - t_prev;
    RPM = (Theta - Theta_prev) * (1000000/dt) * 60 / NFactor;

    e = RPM_ref - RPM;
    inte = inte_prev + (dt/1000000) * (e + e_prev) / 2;
    deriv = (e - e_prev) * (1000000/dt);

    PWM_val = int(kp*e + ki*inte + kd*deriv);

    WriteDriverVoltage(PWM_val);

    Theta_prev = Theta;
    t_prev = dt;
  }

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
void WriteDriverVoltage(int PWM_val){
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
    analogWrite(ENA, min(abs(PWM_val), 250));
}