#include <util/atomic.h>


#define ENA 5
#define IN1 13
#define IN2 12
#define SA 2
#define SB 3
#define button 4

float resolution = 300.0;
float lastInput = 0.0;
float lastTime;
float prevT;
float outMin = 0;
float outMax = 462;
volatile int pulses = 0;
volatile int theta = 0;
int pos = 0;
float eprev = 0.0;
float eintegral = 0.0;
float target;
float kp,kd,ki;
float aggkp = 10.8, aggkd = 0.5714, aggki = 0.831;
float conskp = 10.8, conskd = 0.7714, conski = 0.971;
void setTunings(float p, float i ,float d){
  kp = p;
  ki = i;
  kd = d;
}
void setMotor(int dir, int pwmVal) {
  analogWrite(ENA, pwmVal);
  if(dir == 1){ // CW
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if(dir == -1){ // CCW
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
void readEncoder() {
  pulses++;
  int b = digitalRead(SB);
  if(b>0){ //CW
    theta = 1;
  }
  else{ //CCW
    theta = -1;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(SA, INPUT);
  pinMode(SB, INPUT);
  pinMode(button, INPUT);
  attachInterrupt(0, readEncoder, RISING);
  target = analogRead(0)*(2*digitalRead(4)-1);
  prevT = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  target = map(analogRead(0),0,1023,outMin,outMax)*(2*digitalRead(4)-1);
  float rpm;
  float currT = micros();
  float deltaT = (currT-prevT);
  int dir;
  if(deltaT>=2.0e5){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      rpm = (float)pulses/deltaT/resolution*60*1.0e6;
      pulses = 0;
      dir = theta;
    }
    prevT = micros();
    float e = target - rpm*dir;
    float gap = fabs(e);
    if (gap<10)
      setTunings(conskp,conski,conskd);
    else{
      setTunings(aggkp,aggki,aggkd);
    }
    float dedt = -(rpm-lastInput)/(deltaT/1.0e6);
    eintegral = eintegral + e*(deltaT/1.0e6);
    if (eintegral>255) eintegral = 255;
    else if (eintegral < -255) eintegral = -255;
    float u = kp*e + kd*dedt + ki*eintegral;
    float pwm = fabs(u);
    if(pwm>255) pwm = 255;
    lastInput = rpm; 
    setMotor(2*(u>0)-1, pwm);
    Serial.print(rpm*dir);
    Serial.print(" ");
    Serial.print(target);
    Serial.println();
  }
  
}
