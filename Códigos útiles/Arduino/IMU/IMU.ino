#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

#define PERIODO 20000
#define N_MUESTRAS 50 //Cantidad de vececs que se mide el ángulo de la IMU para estimar el sesgo

Servo miServo; //Es importante que la barra arranque horizontal, para lo que se necesita el servo
Adafruit_MPU6050 mpu;
float theta_bias = 0;

void setup() {
  Serial.begin(115200);
  miServo.attach(9);
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

  miServo.writeMicroseconds(1500);
  delay(1000);
  sensors_event_t a, g, t;
  for(int i = 0; i < N_MUESTRAS; i++){ //Se calcula es sesgo como el promedio de (N_MUESTRAS) mediciones
    mpu.getEvent(&a, &g, &t);
    theta_bias += (180/PI) * atan2(a.acceleration.y, a.acceleration.z);
  }
  theta_bias/=N_MUESTRAS;
}

float theta_x_gyro = 0; //Posición angular estimada por el giroscopio, asumimos que inicialmente es 0. Puede no usarse
float theta_x_acc = 0; //Posición angular estimada por el acelerómetro, asumimos que inicialmente es 0
float theta_x_fc = 0; 
float theta_x_gyro_fc = 0;

float alfa = 0.1;

void loop() {
  unsigned long t_ini = micros();

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  
  theta_x_gyro = theta_x_gyro + (180/PI) * g.gyro.x * (PERIODO/1000000); 
  theta_x_acc = (180/PI) * atan2(a.acceleration.y, a.acceleration.z) - theta_bias;
  theta_x_gyro_fc = theta_x_fc + (180/PI) * g.gyro.x * (PERIODO/1000000);
  
  theta_x_fc = alfa * theta_x_acc + (1 - alfa) * theta_x_gyro_fc;

  float datos[4] = {theta_x_gyro, theta_x_acc, theta_x_gyro_fc, theta_x_fc};
  matlab_send(datos, 4);

  while (micros() - t_ini < PERIODO) {}
}


void matlab_send(float *datos, size_t largo){
  Serial.write("abcd");

  for(size_t i = 0; i < largo; i++){
    byte * b = (byte *) &datos[i];
    Serial.write(b,4);
  }
}


/*
NOTAS:

1) La estimación del giróscopo tiene el problema del bias
2) La estimación del acelerómetro tiene el problema del ruido

Al día de la fecha (10/5) el valor para la constante del filtro complementario es alfa = 0.1

3) ¿Qué pasa si 20000 us es un período muy corto? No se contempla esta posibilidad en el código

4) La variable theta_x_gyro puede no usarse en los códigos en donde no importe ver la deriva de la estimación del gyróscopo
*/