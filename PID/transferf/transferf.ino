#include <util/atomic.h>

#define IN1 13
#define IN2 12
#define EN1 5
#define SA 3
#define SB 2

float prevT;
float pwm;
float resolution = 300.0;
volatile int pulses = 0;
void readEncoder(){
  pulses++;
}

void setMotor(int dir, int pwm){
  analogWrite(EN1, pwm);
  if(dir>0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if(dir==0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(SA, INPUT);
  pinMode(SB, INPUT);
  attachInterrupt(1, readEncoder, RISING);
  Serial.println("CLEARSHEET");
  Serial.println("LABEL, Hora,Segundos,pwm, rpm");
  Serial.println("RESETTIMER");
  prevT = micros();
}
void loop() {
  // put your main code here, to run repeatedly:
  pwm = map(analogRead(0),0,1023,0,255);
  setMotor(1,pwm);
  float rpm;
  float currT = micros();
  float deltaT = currT-prevT;
  if(deltaT>=2.0e5){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      rpm = (float)pulses/deltaT/resolution*60*1.0e6;
      pulses = 0;
    }
    prevT = micros();
    Serial.print("DATA,TIME,TIMER,");
    Serial.print(pwm);
    Serial.print(",");
    Serial.print(rpm);
    Serial.println("");
  }
}
