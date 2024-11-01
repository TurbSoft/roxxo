#include <Arduino.h>

//Motor 1
const int IN1 = 12;
const int IN2 = 11;
const int ENA = 10;
//Motor 2
const int IN3 = 7;
const int IN4 = 8;
const int ENB = 9;

const int pwmValueA = 95;
const int pwmValueB = 95;



void setup() 
{
   // Configuramos los pines como salida
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Control de los motores, ambos giran hacia adelante

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
}