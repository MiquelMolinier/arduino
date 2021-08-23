#include <util/atomic.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(13,12,11,10,9,8);
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
  prevT = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  lcd.setCursor(0,0);
  lcd.print("RPM:");
  float pwm = map(analogRead(0),0,1023,0,255);
  setMotor(1, pwm);
  float rpm;
  float currT = micros();
  float deltaT = currT-prevT;
  if(deltaT>=2.0e5){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      rpm = (float)pulses/deltaT/resolution*60*1.0e6;
      pulses = 0;
    }
    prevT = micros();
    attachInterrupt(0, readEncoder, RISING);
    lcd.print(rpm);
    lcd.print("    ");
    lcd.setCursor(0,1);
  }
  Serial.print(rpm, 3);
  Serial.println();
}
