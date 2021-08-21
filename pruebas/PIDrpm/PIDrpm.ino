#include <util/atomic.h>


#define ENA 5
#define IN1 13
#define IN2 12
#define SA 2
#define SB 3

float resolution = 300.0;
float prevT;
volatile int pulses = 0;
volatile int theta = 0;
int pos = 0;
float eprev = 0.0;
float eintegral = 0.0;
float target = 60.0;
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
  
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(SA, INPUT);
  pinMode(SB, INPUT);
  attachInterrupt(0, readEncoder, RISING);
  prevT = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  float rpm;
  float pwm = map(analogRead(0),0,1023,0,255);
  float currT = micros();
  float deltaT = (currT-prevT);
  if(deltaT>=2.0e5){
    detachInterrupt(0);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      rpm = (float)pulses/deltaT/resolution*60*1.0e6;
      pulses = 0;
    }
    prevT = micros();
    attachInterrupt(0, readEncoder, RISING);
    float kp = 5.25;
    float kd = 2.125;
    float ki = 1.025;
    float e = target - rpm;
    float dedt = (e-eprev)/(deltaT/1.0e6);
    eintegral = eintegral + e*(deltaT/1.0e6);
    float u = kp*e + kd*dedt + ki*eintegral;
    float pwm = fabs(u);
    if(pwm >255){
      pwm = 255;
    }
    setMotor(1, pwm);
    eprev = e;
    Serial.print(rpm);
    Serial.print(" ");
    Serial.print(target);
    Serial.println();
  }
  
}
