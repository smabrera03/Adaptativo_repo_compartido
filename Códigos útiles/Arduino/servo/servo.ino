// Ejercicio 3 -- Grupo 7

#include <Servo.h>
#define PERIODO 20000
Servo miServo;

void setup() {
  miServo.attach(9); // pin PWM
  Serial.begin(9600); // Inicia la comunicación serial
}

void loop() {
  unsigned long t_ini = micros();

  int valor = analogRead(A0); //lectura del potenciómetro
  Serial.println(valor);

  int angulo = mapFloat(valor, 0, 1023, 600, 2400);
  int duty_cycle_servo = (int)mapFloat(angulo, -90, 90, 600, 2400);
  miServo.writeMicroseconds(duty_cycle_servo);

  while (micros() - t_ini < PERIODO) {}
}


float mapFloat(float valor, float x_inicial, float x_final, float y_inicial, float y_final){
  return (y_final - y_inicial)/(x_final - x_inicial) * (valor - x_inicial) + y_inicial;
}
