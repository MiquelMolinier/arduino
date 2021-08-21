#include <util/atomic.h>

#define ENA 5
#define IN1 7
#define IN2 6
#define SA 2
#define SB 3

float resolution = 300.0;
long prevT = 0;
volatile int pulses = 0;
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
  Serial.println("A B");
}

void loop() {
  // put your main code here, to run repeatedly:
  float rpm;
  float target = 285.0;
  if((millis()-prevT)>=500){
    detachInterrupt(0);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      rpm = (float)pulses/(millis()-prevT)/resolution*1000*60;
      pulses = 0;
    }
    prevT = millis();
    attachInterrupt(0, readEncoder, RISING);
  }
  Serial.print(rpm, 3);
  Serial.println();
}
