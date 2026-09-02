/*
Código basado en la plantilla


1) Poner el servo en el ángulo mínimo
2) Por cada iteración:
2.a) leer el ángulo con la IMU (SOLO CON EL ACELERÓMETRO, SIN EL FILTRO COMPLEMENTARIO)
2.b) Enviar el ángulo leído por la IMU y el ángulo comandado
2.c) Incrementar el ángulo. Esperar un timepo suficiente para que se establezca
3) Llegar hasta el ángulo máximo, momento en el cual termina el programa
*/

#include <NewPing.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define TRIGGER_PIN  6   // Pin de arduino conectado al pin del trigger
#define ECHO_PIN     7   // Pin de arduino conectado al pin del echo
#define MAX_DISTANCE 50 // Distancia máxima en centímetros
#define PERIODO 1000000 //Período en us
#define C 0.0343 //Velocidad del sonido en cm/us

#define ANGULO_SERVO_MAX 58.55
#define ANGULO_SERVO_MIN -46.84
#define INCREMENTO 2.0 //pasos de 2° de incremento

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Setup
Servo miServo;
Adafruit_MPU6050 mpu;

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

  int duty_cycle_servo = (int)mapFloat(ANGULO_SERVO_MIN + 0.84, -90, 90, 600, 2400);
  miServo.writeMicroseconds(duty_cycle_servo); //Muevo el servo al ángulo mínimo (y un poquito más para que no toque el piso)
  delay(1000);
}


float theta_x_acc = 0; 
float angulo = ANGULO_SERVO_MIN + 0.84; //Ángulo comandado al servo (acción de control)

void loop() {
  unsigned long t_ini = micros();

  //IMU 2.a)
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  theta_x_acc = (180/PI) * atan2(a.acceleration.y, a.acceleration.z);

  //Envío de datos 2.b)
  float datos[2] = {angulo, theta_x_acc};
  matlab_send(datos, 2);

  //Servomotor 2.
  angulo += INCREMENTO;
  if(angulo > ANGULO_SERVO_MAX){
    while(1){} //se congela el programa
  }
  int duty_cycle_servo = (int)mapFloat(angulo, -90, 90, 600, 2400);
  miServo.writeMicroseconds(duty_cycle_servo);
  delay(500);

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

