#include <Arduino.h>

// int ledPin = 13;

void setup()
{
  // Configuramos el pin del LED como salida
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // Encendemos el LED
  digitalWrite(LED_BUILTIN,HIGH);
  // Esperamos un segundo (1000 milisegundos)
  delay(1000);

  // Apagamos el LED
  digitalWrite(LED_BUILTIN, LOW);
  // Esperamos otro segundo
  delay(1000);
}
