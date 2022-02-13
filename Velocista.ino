#include <QTRSensors.h>

//Mapeo de pines
#define STBY 6  
#define AIN1 5   
#define AIN2 4 
#define PWMB 9 
#define PWMA 3
#define BIN2 8 
#define BIN1 7 
#define NUM_SENSORS             6    
#define NUM_SAMPLES_PER_SENSOR  5    
#define EMITTER_PIN            11  
#define LED                    13     

#define  pulsador  12     // pin usado para el pulsador para configurar sensores

// Constantes para PID
float KP = 1.45;  // variables a cambiar  para mejorar desempeño 
float KD = 60;   // variables a cambiar  para mejorar desempeño 
float Ki = 0;

// Regulación de la velocidad Máxima
int Velmax = 80;   // variables a cambiar  para mejorar desempeño 

// Data para intrgal 
int error1=0;
int error2=0;
int error3=0;
int error4=0;
int error5=0;
int error6=0;

byte estado = 0;   // variable para controlar cuando se activa pulsador calibracion

// Configuración de la librería QTR-8A         
QTRSensorsAnalog qtra((unsigned char[]) {0,1,2,3,4, 5}
, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

// Función accionamiento motor izquierdo
void Motoriz(int value)
{
  if ( value >= 0 )
  {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }
  else
  {
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
    value *= -1;
  }
  analogWrite(PWMB,value);
}

// Función accionamiento motor derecho
void Motorde(int value)
{  
  if ( value >= 0 )
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }
  else
  {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    value *= -1;
  }    
  analogWrite(PWMA,value);
}

//Accionamiento de motores
void Motor(int righ, int left)
{
  digitalWrite(STBY,HIGH);
  Motoriz(left);
  Motorde(righ);
}

//función de freno
void freno(boolean righ, boolean left, int value)
{
  digitalWrite(STBY,HIGH);
  if ( left )
  {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,HIGH);
    analogWrite (PWMB, value);
  }
  if ( righ )
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,HIGH);
    analogWrite (PWMA, value);
  }
}

void setup()
{

  Serial.begin(9600);
  
// Declaramos como salida los pines utilizados
  pinMode(LED   ,OUTPUT);
  pinMode(BIN2  ,OUTPUT);
  pinMode(STBY  ,OUTPUT);
  pinMode(BIN1  ,OUTPUT);
  pinMode(PWMB  ,OUTPUT);
  pinMode(AIN1  ,OUTPUT);
  pinMode(AIN2  ,OUTPUT);
  pinMode(PWMA  ,OUTPUT);


calibracion();

}

unsigned int position = 0; 

//declaraos variables para utilizar PID
int proporcional = 0;         // Proporcional
int integral = 0;           //Intrgral
int derivativo = 0;          // Derivativo
     

int diferencial = 0;   // Diferencia aplicada a los motores
int last_prop;         // Última valor del proporcional (utilizado para calcular la derivada del error)
int Target = 2500; // Setpoint (Como utilizamos 6 sensores, la línea debe estar entre 0 y 5000, por lo que el ideal es que esté en 2500)

void loop()
{   
 
  position = qtra.readLine(sensorValues);
  proporcional = ((int)position) - 2500;

 // Serial.println(position);
  //delay(100);

/*
 if ( proporcional <= -Target )
  {
   Motoriz(0);
    freno(false,true,255);
  }
  else if ( proporcional >= Target )
  {
     Motorde(0);
    freno(true,false,255);
    
  }

  */

  derivativo = proporcional - last_prop; 
  integral = error1+error2+error3+error4+error5+error6;
  last_prop = proporcional;
  
  error6=error5;
  error5=error4;  
  error4=error3;  
  error3=error2;
  error2=error1;
  error1=proporcional;

 int diferencial = ( proporcional * KP ) + ( derivativo * KD )+ (integral*Ki) ;
  
  if ( diferencial > Velmax ) diferencial = Velmax; 
  else if ( diferencial < -Velmax ) diferencial = -Velmax;

  ( diferencial < 0 ) ? 
    Motor(Velmax+diferencial,Velmax) : Motor(Velmax, Velmax-diferencial );
  
}


void calibracion() {
  
 estado = digitalRead(pulsador);   // se lee estado pulsador
  
  while (estado == 1)      
  {
   digitalWrite(LED, HIGH);
   estado = digitalRead(pulsador);
  };
  
  if (estado == 0)    // el pulsador se activa con 0 entonces apaga los led led
 {
  digitalWrite(LED, LOW);
  
};
 
  delay(2000); 
  digitalWrite(LED, HIGH);

  ///////inicio calibracion
  
  for (int i=0; i<70; i++)
  {
    digitalWrite(LED, HIGH); delay(20);
    qtra.calibrate();
    digitalWrite(LED, LOW);  delay(20);
  }
  delay(3000);


/////// fin calibracion////////////


digitalWrite(LED, LOW);     // Apaga el led para indicar que se termino la calibracion.

   
  delay(300);
  digitalWrite(LED, HIGH);     

  delay(300);
  digitalWrite(LED, LOW);     // Se encienden y apagan LEDS .

  delay(300);               
  digitalWrite(LED, HIGH);     
   
  delay(300);
  digitalWrite(LED, LOW);     

  delay(300);

  
  estado = digitalRead(pulsador);
  while (estado == HIGH)         
  {
   digitalWrite(LED, HIGH);  
 
  estado = digitalRead(pulsador);    
  };
  if (estado == LOW)     //   se esepera a que se presione pulsador para poner a funcionar robot
 {
  digitalWrite(LED, LOW);

  delay(1500);   
};

  
  
  }

