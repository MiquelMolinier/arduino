char tipo_serial;
int dato;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    tipo_serial = Serial.read();
    switch(tipo_serial){
      case 'a':
        digitalWrite(13, LOW);
        break;
      case 'b':
        digitalWrite(13, HIGH);
        break;
    }  
  }
}
