/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <util/atomic.h>

#define ENCA 2
#define ENCB 3
#define IDX 4
#define IN1 13
#define IN2 12
#define ENA 5

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
volatile int posi = 0;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.0, consKd=0.25;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Serial.begin(57600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(IDX, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  myPID.SetSampleTime(1);
  Input = 0;
  attachInterrupt(0, readEncoder, RISING);
  Serial.println("target pos targetprev");
  Setpoint = 200;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255,255);
}

void loop()
{
  Setpoint = analogRead(0);
  Input = posi;
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 5)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  myPID.Compute();
  int dir = -1;
  if(Output<0){
    dir = 1;
  }
  double pwm = fabs(Output);
  setMotor(dir, (int)pwm);
  Serial.print(Input);
  Serial.print(" ");
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.print(500);
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
void setMotor(int dir, int pwmVal){
  analogWrite(ENA, pwmVal);
  if(dir == 1){ // CW
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if(dir == -1){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
