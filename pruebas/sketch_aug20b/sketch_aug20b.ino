#include <PID_v1.h>
#include <TimerOne.h>
#define tracking A1     // pin por donde entra el set point
#define sensorP A0      // Pin de entrada de la lectura del sensor          
// Control del motor DC en el puerto 1 del L298N 
int IN1; 
int IN2;
int PWM1;
int forWARDS  = 1; 
int backWARDS = 0;
// Almacenamiento temporal de la lectura del sensor
double sensorValue;

//Definir vairables del control PID
double Setpoint, Input, Output;
float ang;
//Espicifíca, los parámetros del PID
//double Kp=0.4, Ki=0.13, Kd=0.18;
//double Kp=1.5, Ki=0.25, Kd=0.4;   ===>>
//double Kp=0.7, Ki=0.22, Kd=0.35; 
double Kp=1.2, Ki=0.19, Kd=0.001; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Accion en la interrupción 
void flash(){  
  myPID.Compute();  // Calcula la señal de control
  RunMotor(Output); // Aplica la señal de control hacia el Motor
}

/*----------------------------------------------------------------------*/
void setup(){ 
  Serial.begin(115200); 
  //Iniciando PWM
  IN1 = 9;
  IN2 = 8;
  PWM1 = 11;
  pinMode(IN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  //Iniciando lectura de sensores
  sensorValue = analogRead(sensorP);
  Input= -0.1899*sensorValue + 102.18; // ángulos sexagesimales
    // SP
  Setpoint = 60.0; // Valor del Setpoint en grados sexagesimales - predeterminado

  //Activa el PID
  myPID.SetMode(AUTOMATIC);
    // valores maximos y minimos de salida del controlador
  myPID.SetOutputLimits(-19.0,19.0); 
  //TIempo de muestreo milisegundos para el PID
  myPID.SetSampleTime(3);      

  // Inicializando Interrupciones
  Timer1.initialize(4000); // cada 4ms
  Timer1.attachInterrupt(flash); // la funcion flash se acciona en cada INT
}

void loop(){  
  // Captura el valor de la lectura del sensor para el SP
  Setpoint=int(analogRead(tracking)/1024.0*180-90);

  // Pondera el valor de 3+1 lecturas del sensor de posición
  for(int i=1;i<=3;i++){
      sensorValue += analogRead(sensorP);
     }
  sensorValue=sensorValue/4;

  // Constantemente está leyendo el sensor de posición
  ang= -0.1899*sensorValue + 102.18; // Obtenido experimentalmente

  // Algoritmo de protección de mecanismos de posición
  if(ang>=100){
     shaftrev(IN1,IN2,PWM1,backWARDS,0);
  }else if(ang<=-100){
     shaftrev(IN1,IN2,PWM1,backWARDS,0);
  }

  // Introduce el angulo para cerrar el lazo de control
  Input=ang;

  //Muestra la lectura del sensor y el SP
    Serial.print(ang); Serial.print("\t");Serial.println(Setpoint);
}
/*----------------------------------------------------------------------*/

// Función para accionar el giro del eje del motor
void RunMotor(double Usignal){  
  double pwmS;

  if(Usignal>=0){
      pwmS=Usignal*10000/719-9089.0/719.0;
    shaftrev(IN1,IN2,PWM1,backWARDS, pwmS);
  }else{
      pwmS=-Usignal*10000/719-9089.0/719.0;
      shaftrev(IN1,IN2,PWM1,forWARDS, pwmS);
  }   
}

// Función que configura el motor que se quiere controlar
void shaftrev(int in1, int in2, int PWM, int sentido,int Wpulse){  
  if(sentido == 0){ //backWARDS
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    analogWrite(PWM,Wpulse);
    }
  if(sentido == 1){ //forWARDS
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    analogWrite(PWM,Wpulse);     
    }
}
