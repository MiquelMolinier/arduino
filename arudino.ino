#include<LiquidCrystal.h>
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
int SA = 5; // Entrada digital del sensor A
long i = 0;
int M1= LOW, M2 = HIGH;
int valor;
void setup()
{
  attachInterrupt(0, giro, RISING);
  attachInterrupt(1, SB, RISING);

  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(SA, INPUT);

  lcd.begin(16, 2);
  lcd.print("Practica L298N");
  lcd.setCursor(0, 1);
  lcd.print("ENCODER");
  delay(500);
  lcd.clear();
}
void loop()
{
  lcd.setCursor(0, 0);
  lcd.print("ENCODER:");
  lcd.setCursor(0, 1);
  lcd.print(i);
  lcd.print("    ");
  digitalWrite(7, M1);
  digitalWrite(6, M2);
}

void giro()
{
  M1 = !M1;
  M2 = !M2;
}

void SB()
{
  valor = digitalRead(SA);
  if(valor==1)
  {
    i++;
  }
  else
  {
    i--;
  }
}
