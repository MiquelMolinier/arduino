volatile byte pulsos;
unsigned int RPM;
unsigned long timeb;
void functionRPM(){
  pulsos++;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // 57600?
  attachInterrupt(0, functionRPM, RISING);
  pulsos = 0;
  RPM = 0;
  timeb = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  int timea = millis()
  int deltat = timea-timeb;
  if(deltat>=5){
    detachInterrupt(0);
    RPM = ((pulsos/deltat)*1000);//pulsos por segundo
    RPM /= 11; // IPM: numero de pulsos por segundo
    RPM *= 60; // Transformar segundos a minutos
    timeb = timea;
    pulsos = 0;
  }
}
