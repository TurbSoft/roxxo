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

// Valores de velocidad de los motores
int lowSpeed  = 95; 
int highSpeed = 100;

// Sensor Definicion de todos los sensores
// const int line_pin[5] = {2, 3, 4, 5, 6};

// Si queremos usar por ejemplo solo 2
const int S1 = 2; // Morado
const int S2 = 3; // Azul
const int S3 = 4; // Verde
const int S4 = 5; // Amarillo 
const int S5 = 6; // Naranja

// Inicio startup
bool startup = false;

// Memoria
int lastRead=HIGH;
int currentRead=HIGH;
int counter=0;

// Declaracion de funciones
void motorDerecha_adelante(int pwmValueA);
void motorDerecha_atras(int pwmValueA);
void motorIzquierda_adelante(int pwmValueB);
void motorIzquierda_atras(int pwmValueB);
void robotAdelante(int pwmD, int pwmI);
void robotIzquierda2Ruedas(int pwmD, int pwmI);
void robotIzquierda1Rueda(int pwm);
void robotDerecha2Ruedas(int pwmD, int pwmI);
void robotDerecha1Rueda(int pwm);

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
 
  // Startup (Reset 5 segundos antes de comenzar)
  if (!startup) {
      delay(5000);
      startup = true;
  }
  
  if (startup) {
    // 1. Leer el valor de los sensores y ponerlo en variables
    int inS1 = digitalRead(S1);
    int inS2 = digitalRead(S2);
    int inS3 = digitalRead(S3);
    int inS4 = digitalRead(S4);
    int inS5 = digitalRead(S5);

    // 3. Controlar el motor 
    currentRead = compute_send_pwm_from_sensor(inS1, inS2, inS3, inS4, inS5, lastRead);
    lastRead = currentRead;

    // Mantener esto por un valor de tiempo
    delay(50);
  }
}

void print_to_serial(int inS1, int inS2, int inS3, int inS4, int inS5, int pwmI, int pwmD) {
    Serial.print("I D:\t");
    Serial.print(pwmI);
    Serial.print("\t");
    Serial.print(pwmD);
    Serial.print("\t");
    Serial.print("| S1 S2 S3 S4 S5: ");
    Serial.print(inS1);
    Serial.print(" ");
    Serial.print(inS2);
    Serial.print(" ");
    Serial.print(inS3);
    Serial.print(" ");
    Serial.print(inS4);
    Serial.print(" ");
    Serial.println(inS5);  // Ends the line after the last value
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

int compute_send_pwm_from_sensor(int inS1, int inS2, int inS3, int inS4, int inS5, int lastRead) {
  // Logica de control 

  //         S3              
  // S1  S2  x   S4  S5 
    
  // S2 //
  if (inS1 == HIGH && inS2 == LOW && inS3 == HIGH && inS4==HIGH && inS5==HIGH) {
      counter = 0;
      pwmValueD = highSpeed;
      pwmValueI = lowSpeed;
      print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
      robotAdelante(pwmValueD, pwmValueI);
      return S2;     
  }
  // S2 y S3
  if (inS1 == HIGH && inS2 == LOW && inS3 == LOW && inS4==HIGH && inS5==HIGH) {
      counter = 0;
      pwmValueD = lowSpeed;
      pwmValueI = 0;
      print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
      robotAdelante(pwmValueD, pwmValueI);
      return S2;     
  }
  // S3
  if (inS1 == HIGH && inS2 == HIGH && inS3 == LOW && inS4==HIGH && inS5==HIGH) {
      counter = 0;
      pwmValueD = lowSpeed;
      pwmValueI = lowSpeed;
      print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
      robotAdelante(pwmValueD, pwmValueI);
      return S3; 
  }
  // S4
  if (inS1 == HIGH && inS2 == HIGH && inS3 == HIGH && inS4==LOW && inS5==HIGH) {
      counter = 0;
      pwmValueD = lowSpeed;
      pwmValueI = highSpeed;
      print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
      robotAdelante(pwmValueD, pwmValueI);
      return S4;
  } 
  // S3 y S4
  if (inS1 == HIGH && inS2 == HIGH && inS3 == LOW && inS4==LOW && inS5==HIGH) {
      counter = 0;
      pwmValueD = 0;
      pwmValueI = lowSpeed;
      print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
      robotAdelante(pwmValueD, pwmValueI);
      return S4;     
  }
  // S1
  if (inS1 == LOW && inS2 == HIGH && inS3 == HIGH && inS4==HIGH && inS5==HIGH) {
      counter = 0;
      pwmValueD = lowSpeed;
      pwmValueI = 0;
      print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
      robotAdelante(pwmValueD, pwmValueI);
      return S1;     
  }
  // S5
  if (inS1 == HIGH && inS2 == HIGH && inS3 == HIGH && inS4==HIGH && inS5==LOW) {
      counter = 0;
      pwmValueD = 0;
      pwmValueI = lowSpeed;
      print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
      robotAdelante(pwmValueD, pwmValueI);
      return S5;     
  }
 

  if (inS1 == HIGH && inS2 == HIGH && inS3 == HIGH && inS4==HIGH && inS5==HIGH)
  {
    // buscar
    if ((lastRead == S3 || lastRead == S1 || lastRead == S5 ) && counter < 20){
        while (counter<20){
            pwmValueD = lowSpeed;
            pwmValueI = lowSpeed;
            print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
            robotAdelante(pwmValueI, pwmValueI);
            counter++;
            return lastRead;
        }
    }
    else{
        int search_dir = random(1); // 0 o 1
        pwmValueD = lowSpeed;
        pwmValueI = lowSpeed;
        if (search_dir == 0){
            while(inS1 == HIGH && inS2 == HIGH && inS3 == HIGH && inS4==HIGH && inS5==HIGH)
            {
                robotDerecha2Ruedas(pwmValueD, pwmValueI);
                return S3;
            }
        }
        if (search_dir == 1){
            while(inS1 == HIGH && inS2 == HIGH && inS3 == HIGH && inS4==HIGH && inS5==HIGH)
            {
                robotIzquierda2Ruedas(pwmValueD, pwmValueI);
                return S3;
            }
        }
        

    }
  }
  else
   {
      delay(200);
      pwmValueD = 0;
      pwmValueI = 0;
      print_to_serial(inS1, inS2, inS3, inS4, inS5, pwmValueI, pwmValueD);
      robotAdelante(pwmValueD, pwmValueI);
      return S3;
  }
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
    motorIzquierda_adelante(pwmI);
    motorDerecha_atras(pwmD);
}

void robotDerecha2Ruedas(int pwmD, int pwmI) {
    motorDerecha_adelante(pwmD);
    motorIzquierda_atras(pwmI);
}