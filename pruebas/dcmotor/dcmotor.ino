#include <util/atomic.h>

#define ENCA 2
#define ENCB 3
#define IDX 4
#define IN1 13
#define IN2 12
#define ENA 5

volatile int posi = 0;
long prevT = 0;
float eprev = 0.0;
float eintegral = 0.0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(IDX, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(0, readEncoder, RISING);
  Serial.println("target pos targetprev");
}
void loop() {
  // put your main code here, to run repeatedly:
  // constantes PID
  int target = analogRead(0);
  float kp = 1.25;
  float kd = 0.125;
  float ki = 0.0;
  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  int e = pos - target;
  float dedt = (e-eprev)/deltaT;
  eintegral = eintegral + e*deltaT;
  float u = kp*e +kd*dedt + ki*eintegral;
  float pwm = fabs(u);
  if(pwm >255){
    pwm = 255;
  }
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  setMotor(dir,pwm, ENA, IN1, IN2);
  eprev = e;
  Serial.print(1200);
  Serial.print(" ");
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}
void readEncoder(){
  int b = digitalRead(ENCB);
  if (b>0){
    posi--; // CW
  }
  else{
    posi++; // CCW
  }
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  // dir: rotation direction
  // pwmVal: pwm speed
  // pwm: pwm pin
  // in1, in2: in1,in2 pin
  analogWrite(pwm, pwmVal);
  if(dir == 1){ // CW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
