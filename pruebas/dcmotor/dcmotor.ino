#define ENCA 2
#define ENCB 3
#define IDX 4
#define IN1 13
#define IN2 12
#define ENA 5
int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(IDX, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(0, readEncoder, RISING);
  
}
void loop() {
  // put your main code here, to run repeatedly:
  setMotor(1, 127, ENA, IN1, IN2);
  delay(200);
  Serial.println(pos);
  setMotor(-1, 127, ENA, IN1, IN2);
  delay(200);
  Serial.println(pos);
  setMotor(0, 127, ENA, IN1, IN2);
  delay(200);
  Serial.println(pos);
}
void readEncoder(){
  int b = digitalRead(ENCB);
  if (b>0){
    pos--; // CW rotate one way
  }
  else{
    pos++; // CCW rotate other way
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
