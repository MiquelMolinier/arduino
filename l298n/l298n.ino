int dir1PinA = 6;
int dir2PinA = 7;
int ENA = 3;
int v = 0;
void setup() { 
 
Serial.begin(9600); // baud rate

//lcd.begin(20, 4);

pinMode(dir1PinA,OUTPUT);
pinMode(dir2PinA,OUTPUT); 
analogWrite(ENA,0);
}


void loop() {
if (Serial.available() > 0)
{
  int inByte = Serial.read();
  switch (inByte)
  {
    case 'C': // Clockwise rotation
    for(int i=analogRead(ENA); i>=0; i--)
    {
      analogWrite(ENA,i);
    }
    for(int i=0; i<256; i++)
    {
      digitalWrite(dir1PinA, LOW);
      digitalWrite(dir2PinA, HIGH);
      analogWrite(ENA,i);
      delay(50);
    }
    Serial.println("Clockwise rotation"); 
    Serial.println("   "); 
    break;
    
    case 'S': // No rotation
    digitalWrite(dir1PinA, LOW);
    digitalWrite(dir2PinA, LOW);
    Serial.println("No rotation");
    Serial.println("   ");
    break;
    
    case 'A': // Clockwise rotation
    for(int i=analogRead(ENA); i>=0; i--)
    {
      analogWrite(ENA,i);
    }
    for(int i=0; i<256; i++)
    {
      digitalWrite(dir1PinA, HIGH);
      digitalWrite(dir2PinA, LOW);
      analogWrite(ENA,i);
      delay(50);
    }
    Serial.println("Anti Clockwise rotation"); 
    Serial.println("   "); 
    break;
  }
}
}
