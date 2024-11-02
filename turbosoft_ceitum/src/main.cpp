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
int pwmValueD = 0; //modificable (95)
int pwmValueI = 0;

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
  
  /*
  1. Leer la informacion de los sensores
  2. Enviar la informacion de los sensores al USB;
  3. Logica de control: Como fijar los motores segun los sensores
  4. Enviar los valores de pwm a los motores ;) 
  */


  // 1. Leer el valor de los sensores y ponerlo en variables
  int inS1 = digitalRead(S1);
  int inS2 = digitalRead(S2);
  int inS3 = digitalRead(S3);
  int inS4 = digitalRead(S4);
  int inS5 = digitalRead(S5);

  // 2. Escribir en el puerto serial (USB)
  print_to_serial(inS1, inS2,inS3, inS4, inS5);

  // 3. Controlar el motor 
  compute_pwm_from_sensor(inS1, inS2, inS3, inS4, inS5);

  // 4. Enviar los valores de pwm a los motores 
 // send_pwm(pwmValueA, pwmValueB);

  // Mantener esto por un valor de tiempo
  delay(100);
}

void print_to_serial(int inS1, int inS2, int inS3, int inS4, int inS5){
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
    
}

void send_pwm(int pwmValueA, int pwmValueB){

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

}

void compute_pwm_from_sensor(int inS1, int inS2, int inS3, int inS4, int inS5){
  // Definimos la logica del sensor 
  /* El robot dispone de 5 entradas (sensores distribuidos asi)
                    S3
    S1      S2             S4       S5



    MotorB                  MotorA
    (Izquierda)              (Derecha)

    Proposicion:
    
    Idea 1:

    1. Si S3 detecta entonces pwmA = pwmB = 105 (Andar hacia adelante)
    2. Si S2 detecta entonces pwmA = 105, pwmB=0 (Girar izquierda)
    3. Si S4 detecta entonces pwmA = 0, pwmB=105 (Girar derecha)

    Idea 2: 
    Programar S1 y S5 

    1. Si S3=LOW y S2=HIGH y S4=HIGH entonces robotAdelante
    2. Si S2=LOW entonces robotDerecha1Rueda
    3. Si S4=LOW entonces robotIzquierda1Rueda
    4. Si S5=LOW entonces robotIzquierda2Ruedas
    5. Si S1=LOW entonces robotDerecha2Ruedas

  */


  // Sensor de la izquierda
  if (inS2 != HIGH ){
      pwmValueD = 105;
      pwmValueI = 0;
      robotDerecha1Rueda(pwmValueD)
  } 
  // Andar hacia adelante
  else if (inS3 != HIGH && inS2 == HIGH && inS4==HIGH){
      pwmValueI = 105;
      pwmValueD = 105;
      robotAdelante(pwmValueD, pwmValueI);
  }
  // Sensor de la derecha
  else if (inS4 != HIGH){
      pwmValueD = 0;
      pwmValueI = 105;
      robotIzquierda1Rueda(pwmValueI);
  }
  else if(inS1 != HIGH){
      pwmValueD = 95;
      pwmValueI = 95;
      while (inS3 == HIGH){
        robotDerecha2Ruedas(pwmValueD, pwmValueD);
      }
  } 
  else if(inS5 !=HIGH){
      pwmValueD = 95;
      pwmValueI = 95;
      while (inS3 == HIGH){
        robotIzquierda2Ruedas(pwmValueD, pwmValueD);
      }
  }
  else
  {
      pwmValueI = 95;
      pwmValueD = 95;
      robotAdelante(pwmValueD, pwmValueD);
  }
}

void motorDerecha_adelante(int pwmValueA){

    // Motor 1 hacia adelante
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    
    // Controlamos la velocidad con PWM
    analogWrite(ENA, pwmValueA);
}

void motorDerecha_atras(int pwmValueA){

    // Motor 1 hacia adelante
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    
    // Controlamos la velocidad con PWM
    analogWrite(ENA, pwmValueA); 
}
    
void motorIzquierda_adelante(int pwmValueB){

  // Motor 2 también hacia adelante
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Controlamos la velocidad con PWM
  analogWrite(ENB, pwmValueB);    
}

void motorIzquierda_atras(int pwmValueB){

  // Motor 2 también hacia adelante
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Controlamos la velocidad con PWM
  analogWrite(ENB, pwmValueB);    
}

void robotAdelante(int pwmD, int pwmI){
    motorDerecha_adelante(pwmD);
    motorIzquierda_adelante(pwmI);
}

void robotIzquierda2Ruedas(int pwmD, int pwmI){
    motorIzquierda_atras(pwmI);
    motorDerecha_adelante(pwmD);
}

void robotIzquierda1Rueda(int pwm){
    motorIzquierda_adelante(0);
    motorDerecha_adelante(pwm);
}

void robotDerecha2Ruedas(int pwmD, int pwmI){
    motorDerecha_atras(pwmD);
    motorIzquierda_adelante(pwmI)
}

void robotDerecha1Rueda(int pwm){
    motorDerecha_adelante(0);
    motorIzquierda_adelante(pwm);
}