#include <Arduino.h>

//Motor 1
const int IN1 = 12;
const int IN2 = 11;
const int ENA = 10;
//Motor 2
const int IN3 = 7;
const int IN4 = 8;
const int ENB = 9;

//Entradas pwm valor inicial
int pwmValueA = 0; //modificable (95)
int pwmValueB = 0;

// Sensor Definicion de todos los sensores
// const int line_pin[5] = {2, 3, 4, 5, 6};

// Si queremos usar por ejemplo solo 2
const int S1=2; // Morado
const int S2=3; // Azul
const int S3=4; // Verde
const int S4=5; // Amarillo 
const int S5=6; // Naranja
// Esto se puede hacer tambien asi:
// const int line_pin[3] = {3, 4, 5};


void setup() {

   // Configuramos los pines como salida
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuramos los pines de entrada
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  

  // for (int i = 0; i < 3; i++) {
  //   pinMode(line_pin[i], INPUT);
  // }

  // Configuramos el puerto serial (via USB)
  Serial.begin(9600);
}

void loop() {
  
  // Leer el valor de los sensores y ponerlo en variables
  int inS1 = digitalRead(S1);
  int inS2 = digitalRead(S2);
  int inS3 = digitalRead(S3);
  int inS4 = digitalRead(S4);
  int inS5 = digitalRead(S5);

  // Escribir en el puerto serial (USB)

  // Escribir lo que se recibio del sensor S2 en el USB
  Serial.print("Pin N");
  Serial.print(S1);
  Serial.print(": ");
  Serial.println(inS1 == HIGH ? "HIGH" : "LOW");
  
  // Escribir lo que se recibio del sensor S2 en el USB
  Serial.print("Pin Y");
  Serial.print(S2);
  Serial.print(": ");
  Serial.println(inS2 == HIGH ? "HIGH" : "LOW");
  
  // Escribir lo que se recibio del sensor S3 en el USB
  Serial.print("Pin Y");
  Serial.print(S3);
  Serial.print(": ");
  Serial.println(inS3 == HIGH ? "HIGH" : "LOW");

  // Escribir lo que se recibio del sensor S4 en el USB
  Serial.print("Pin Y");
  Serial.print(S4);
  Serial.print(": ");
  Serial.println(inS4 == HIGH ? "HIGH" : "LOW");

  // Escribir lo que se recibio del sensor S2 en el USB
  Serial.print("Pin N");
  Serial.print(S5);
  Serial.print(": ");
  Serial.println(inS5 == HIGH ? "HIGH" : "LOW");
  
  // Las lineas de arriba se pueden implementar con el siguiente codigo:
  // for (int i = 0; i < 3; i++) {
  //   Serial.print("Pin ");
  //   Serial.print(line_pin[i]);
  //   Serial.print(": ");
  //   Serial.println(pinState == HIGH ? "HIGH" : "LOW");
  // }

  // Controlar el motor 
  // Sensor de la derecha
  //    x 
  if (inS2 != HIGH){
      pwmValueA = 95;
      pwmValueB = 0;
  } 
  //                        x        
  else if (inS3 != HIGH){
      pwmValueA = 105;
      pwmValueB = 105;
  }
  //                                        x
  else if (inS4 != HIGH){
      pwmValueA = 0;
      pwmValueB = 95;
  }
  else {
      pwmValueA = 0;
      pwmValueB = 0;
  }

  // Escribir los valores en el pwm
  // Motor 1 hacia adelante
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Controlamos la velocidad con PWM
  analogWrite(ENA, pwmValueA);


  // Motor 2 también hacia adelante
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  // Controlamos la velocidad con PWM
  analogWrite(ENB, pwmValueB);

  // Mantener esto por un valor de tiempo
  delay(100);
}

