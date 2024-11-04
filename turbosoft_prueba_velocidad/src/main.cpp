#include <Arduino.h>

// Motor 1
const int IN1 = 12;
const int IN2 = 11;
const int ENA = 10;
// Motor 2
const int IN3 = 7;
const int IN4 = 8;
const int ENB = 9;

int pwmValueA = 90;
int pwmValueB = 90;


void setup() 
{
   // Configuramos los pines como salida
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
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


    delay(3000);

    pwmValueA = pwmValueA + 1;
    pwmValueB = pwmValueB + 1;


    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);

}