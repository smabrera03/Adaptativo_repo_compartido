// Ejercicio 2 -- Grupo 7

#include <NewPing.h>

#define TRIGGER_PIN  6   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     7   // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 40 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define C 0.0343 //Velocidad del sonido en cm/us
#define PERIODO 20000 //Período en us
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(115200);
}


void loop() {
  unsigned long t_ini = micros();

  unsigned long med = sonar.ping(MAX_DISTANCE); //medición en cm. sonar.ping devuelve us.
  Serial.print("Medición del sensor: ");
  Serial.print(med); //OBS: Tiene algo de ruido
  // OBSERVACIóN IMPORTANTE: Tiene que haber un delay entre 2 mediciones. Si no, hay errores
  Serial.print("||| Posición del carro: ");
  Serial.println(posicion_carrito(med));

  while (micros() - t_ini < PERIODO) {}
}

float mapFloat(float valor, float x_inicial, float x_final, float y_inicial, float y_final){
  return (y_final - y_inicial)/(x_final - x_inicial) * (valor - x_inicial) + y_inicial;
}

float posicion_carrito(unsigned long tiempo){ //Esta función devuelve la posición del carro en nuestro sistema de referencia
  //unsigned long tiempo = sonar.ping(MAX_DISTANCE); //NO se pueden hacer 2 mediciones seguidas
  float medicion = (C/2) * tiempo; 
  if(tiempo == 0){ //Se excedió el tiempo máximo, el carrito se cayó de la barra o está pegado al sensor
    medicion = 15.5; //Quiero que cuando el carrito se caiga, el sensor lo detecte en 0.
  } else if(medicion < 2){
    medicion = 2; //Para las mediciones menores a 2cm
  }
  return mapFloat(medicion, 15.5, 32, 0, 17.25);
}
