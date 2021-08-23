#define SB 3
#define SA 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(SA,  INPUT);
  pinMode(SB, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int a = digitalRead(SA);
  int b = digitalRead(SB);
  Serial.print(a*5);
  Serial.print(" ");
  Serial.print(b*5);
  Serial.println();
}
