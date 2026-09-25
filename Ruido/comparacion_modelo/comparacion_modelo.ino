/*
Explicación: 
Este código da una serie de escalones de distintas alturas como entrada.

Las alturas están en el vector escalones[]

Cada escalón tiene una duración guardada en el vector duracion[]

la variable indice_escalon va aumentando a medida que pasan los escalones

Cuando se recorren todos los escalones, el código se queda en el último. 
*/

#include <NewPing.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define TRIGGER_PIN  6   // Pin de arduino conectado al pin del trigger
#define ECHO_PIN     7   // Pin de arduino conectado al pin del echo
#define MAX_DISTANCE 50 // Distancia máxima en centímetros
#define PERIODO 20000.0 //Período en us
#define N_MUESTRAS 50 //Cantidad de vececs que se mide el ángulo de la IMU para estimar el sesgo
#define C 0.0343 //Velocidad del sonido en cm/us

#define ANGULO_SERVO_MAX 58.55
#define ANGULO_SERVO_MIN -46.84

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Setup
Servo miServo;
Adafruit_MPU6050 mpu;
//float theta_bias = 0;


// ========================================00
// Defino la secuencia de entrada
// ==========================================

/*
//Este par de vectores sirve para balancear el carrito, pero no para identificar b
float escalones [] = {
  0, 15, 0, -15, 
  0, 20, 0, -20, 
  0
};

unsigned long duracion [] = { //Duración del escalón en ms
  1000, 1500, 1000, 500, 
  1000, 1000, 1000, 500,
  1000
};
*/


/*
//Estos valores corresponden a la medición b_izuquierda.mat
float escalones [] = {
  0, 20, 0
};

unsigned long duracion [] = {
  50, 1250, 1000
};
*/


/*
//Estos valores corresponden a la medición b_derecha
float escalones [] = {
  0, -17, 0
};

unsigned long duracion [] = {
  50, 1000, 1000
};
*/

float escalones [] = {
  0, -10, 15, -10,
  0
};

unsigned long duracion [] = {
  20, 600, 1000, 600, 
  1000
};

//De esta forma, el escalon escalones[i] dura duracion[i]
//Estos valores fueron encontrados por prueba y error, procurando mantener el carrito sobre la barra


const int N_ESCALONES = sizeof(escalones) / sizeof(escalones[0]); //Cantidad de escalones
int indice_escalon = 0; //Este indice va subiendo a medida que avancen los escalones
unsigned long tiempo_inicio_escalon;


void setup() {
  miServo.attach(9); // pin PWM
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("IMU no encontrada");
    while (1) {
      delay(10);
    }
  }
  Serial.println("IMU inicializada correctamente");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  /*
  //Comento esta parte porque este sesgo ya está contemplado en la función theta_b = phi(theta_s)
  sensors_event_t a, g, t;
  for(int i = 0; i < N_MUESTRAS; i++){ //Se calcula es sesgo como el promedio de (N_MUESTRAS) mediciones
    mpu.getEvent(&a, &g, &t);
    theta_bias += (180/PI) * atan2(a.acceleration.y, a.acceleration.z);
  }
  theta_bias/=N_MUESTRAS;
  */

  //Se comanda el primer escalón
  float angulo = escalones[0];
  int duty_cycle_servo = (int)mapFloat(angulo, -90, 90, 600, 2400);
  miServo.writeMicroseconds(duty_cycle_servo);
  delay(500);
  tiempo_inicio_escalon = millis();
}


float theta_x_acc = 0; //Posición angular estimada por el acelerómetro, asumimos que inicialmente es 0
float theta_x_fc = 0; 
float theta_x_gyro_fc = 0;

float alfa = 0.1;

void loop() {
  unsigned long t_ini = micros();

  //Sensor ultrasónico
  float posicion = posicion_carrito(); //medición en cm  

  //Servomotor

  //Lógica para actualizar el escalón.
  if(millis() - tiempo_inicio_escalon >= duracion[indice_escalon]){
    indice_escalon++;

    if(indice_escalon >= N_ESCALONES){
      indice_escalon = N_ESCALONES - 1; //Si termina la secuencia, se queda en el último escalón.
      //Otra opcion sería indice_escalones = 0, y volver a empezar
    }

    tiempo_inicio_escalon = millis();
  }

  float angulo = escalones[indice_escalon];
  
  if(angulo < ANGULO_SERVO_MIN){
    angulo = ANGULO_SERVO_MIN;
  } else if(angulo > ANGULO_SERVO_MAX){
    angulo = ANGULO_SERVO_MAX;
  }
  int duty_cycle_servo = (int)mapFloat(angulo, -90, 90, 600, 2400);
  miServo.writeMicroseconds(duty_cycle_servo);

  //IMU
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  

  theta_x_acc = (180/PI) * atan2(a.acceleration.y, a.acceleration.z);
  theta_x_gyro_fc = theta_x_fc + (180/PI) * g.gyro.x * (PERIODO/1000000.0);
  
  theta_x_fc = alfa * theta_x_acc + (1 - alfa) * theta_x_gyro_fc;

  float datos[3] = {posicion, angulo, theta_x_fc};
  matlab_send(datos, 3);

  while (micros() - t_ini < PERIODO) {}
}

float mapFloat(float valor, float x_inicial, float x_final, float y_inicial, float y_final){
  return (y_final - y_inicial)/(x_final - x_inicial) * (valor - x_inicial) + y_inicial;
}

void matlab_send(float *datos, size_t largo){
  Serial.write("abcd");

  for(size_t i = 0; i < largo; i++){
    byte * b = (byte *) &datos[i];
    Serial.write(b,4);
  }
}

float posicion_carrito(){ //Esta función devuelve la posición del carro en nuestro sistema de referencia
  unsigned long tiempo = sonar.ping(MAX_DISTANCE); //NO se pueden hacer 2 mediciones seguidas
  float medicion = (C/2) * tiempo; 
  if(tiempo == 0){ //Se excedió el tiempo máximo, el carrito se cayó de la barra o está pegado al sensor
    medicion = 15.5; //Quiero que cuando el carrito se caiga, el sensor lo detecte en 0.
  } else if(medicion < 2){
    medicion = 2; //Para las mediciones menores a 2cm
  }
  return mapFloat(medicion, 15.5, 32, 0, 17.25);
}

