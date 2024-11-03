#include <Arduino.h>

// Motor 1
const int IN1 = 12;
const int IN2 = 11;
const int ENA = 10;
// Motor 2
const int IN3 = 7;
const int IN4 = 8;
const int ENB = 9;

// Entradas pwm valor inicial
int pwmValueD = 0; // modificable (95)
int pwmValueI = 0;

// Sensor Definicion de todos los sensores
// const int line_pin[5] = {2, 3, 4, 5, 6};

// Si queremos usar por ejemplo solo 2
const int S1 = 2; // Morado
const int S2 = 3; // Azul
const int S3 = 4; // Verde
const int S4 = 5; // Amarillo 
const int S5 = 6; // Naranja

void robotDerecha1Rueda(int pwm);
void robotIzquierda1Rueda(int pwm);

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
  
  // Configuramos el puerto serial (via USB)
  Serial.begin(9600);
}

void loop() {
  bool startup = false;

  if (!startup) {
      delay(5000);
      startup = true;
  }
  
  int lastValue = 0;
  int currentValue = 0; // Declare currentValue

  if (startup) {
    // 1. Leer el valor de los sensores y ponerlo en variables
    int inS1 = digitalRead(S1);
    int inS2 = digitalRead(S2);
    int inS3 = digitalRead(S3);
    int inS4 = digitalRead(S4);
    int inS5 = digitalRead(S5);

    // 2. Escribir en el puerto serial (USB)
    print_to_serial(inS1, inS2, inS3, inS4, inS5);

    // 3. Controlar el motor 
    currentValue = compute_send_pwm_from_sensor(inS1, inS2, inS3, inS4, inS5, lastValue);

    // Mantener esto por un valor de tiempo
    delay(50);
    lastValue = currentValue;
  }
}

void print_to_serial(int inS1, int inS2, int inS3, int inS4, int inS5) {
    Serial.print("Pin N");
    Serial.print(S1);
    Serial.print(": ");
    Serial.println(inS1 == HIGH ? "HIGH" : "LOW");
    
    Serial.print("Pin Y");
    Serial.print(S2);
    Serial.print(": ");
    Serial.println(inS2 == HIGH ? "HIGH" : "LOW");
    
    Serial.print("Pin Y");
    Serial.print(S3);
    Serial.print(": ");
    Serial.println(inS3 == HIGH ? "HIGH" : "LOW");

    Serial.print("Pin Y");
    Serial.print(S4);
    Serial.print(": ");
    Serial.println(inS4 == HIGH ? "HIGH" : "LOW");

    Serial.print("Pin N");
    Serial.print(S5);
    Serial.print(": ");
    Serial.println(inS5 == HIGH ? "HIGH" : "LOW");
}

void send_pwm(int pwmValueA, int pwmValueB) {
  // Escribir los valores en el pwm
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwmValueA);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, pwmValueB);
}

int compute_send_pwm_from_sensor(int inS1, int inS2, int inS3, int inS4, int inS5, int lastValue) {
  int lastActiveMotor;

  if (inS2 != HIGH) {
      pwmValueD = 95;
      pwmValueI = 0;
      robotDerecha1Rueda(pwmValueD);
      lastActiveMotor = 0; // Derecha
      return lastActiveMotor;
  }
  if (inS4 != HIGH) {
      pwmValueD = 0;
      pwmValueI = 95;
      robotIzquierda1Rueda(pwmValueI);
      lastActiveMotor = 1; // Izquierda
      return lastActiveMotor;
  }

  pwmValueD = 95;
  pwmValueI = 0;
  robotDerecha1Rueda(pwmValueD);
  lastActiveMotor = 0;  
  return lastActiveMotor;
}

void motorDerecha_adelante(int pwmValueA) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmValueA);
}

void motorDerecha_atras(int pwmValueA) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwmValueA); 
}
    
void motorIzquierda_adelante(int pwmValueB) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwmValueB);    
}

void motorIzquierda_atras(int pwmValueB) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmValueB);    
}

void robotAdelante(int pwmD, int pwmI) {
    motorDerecha_adelante(pwmD);
    motorIzquierda_adelante(pwmI);
}

void robotIzquierda2Ruedas(int pwmD, int pwmI) {
    motorIzquierda_atras(pwmI);
    motorDerecha_adelante(pwmD);
}

void robotIzquierda1Rueda(int pwm) {
    motorIzquierda_adelante(0);
    motorDerecha_adelante(pwm);
}

void robotDerecha2Ruedas(int pwmD, int pwmI) {
    motorDerecha_atras(pwmD);
    motorIzquierda_adelante(pwmI);
}

void robotDerecha1Rueda(int pwm) {
    motorDerecha_adelante(0);
    motorIzquierda_adelante(pwm);
}
