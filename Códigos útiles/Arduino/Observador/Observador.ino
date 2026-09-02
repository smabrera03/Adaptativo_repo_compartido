#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>

#define PERIODO 20000 //período del ciclo en us. En este caso se elige un preíodo grande para que el servo tenga tiempo para estabilizarse
#define N_MUESTRAS 50 //Cantidad de vececs que se mide el ángulo de la IMU para estimar el sesgo
#define CICLOS 50 //Cantidad de ciclos que tienen que pasar para cambiar el valor de uk
#define UK_REF -20 //ángulo que se moverá el servo

Adafruit_MPU6050 mpu;
Servo miServo;

float theta_bias = 0;

void setup() {
  miServo.attach(9); // pin PWM
  Serial.begin(115200); // Inicia la comunicación serial

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  miServo.writeMicroseconds(1500);
  delay(500);

  sensors_event_t a, g, t;
  for(int i = 0; i < N_MUESTRAS; i++){ //Se calcula es sesgo como el promedio de (N_MUESTRAS) mediciones
    mpu.getEvent(&a, &g, &t);
    theta_bias += (180/PI) * atan2(a.acceleration.y, a.acceleration.z);
  }
  theta_bias/=N_MUESTRAS;
}


float theta_x_gyro = 0; //Posición angular estimada por el giroscopio, asumimos que inicialmente es 0
float theta_x_acc = 0; //Posición angular estimada por el giroscopio, asumimos que inicialmente es 0

float alpha = 0.1; //NO TOCAR

float uk = 0;
float uk_1 = 0;
int n_ciclo = 0; //número del ciclo

float yk = 0;
float yk_1 = 0;

using namespace BLA;

Matrix<2, 2> Ad= {1.0000, 0.0200, -8.6415, 0.0829};
Matrix<2, 1> Bd = {0, 2.7781};
Matrix<1, 2> Cd = {1, 0};

Matrix<2, 1> xk = {0, 0};
Matrix<2, 1> xk_1 = {0, 0};

Matrix<2, 1> L = {1.0466, -8.4321};

void loop() {
  unsigned long t_ini = micros();

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float velocidad_gyro = (180/PI) * g.gyro.x;
  theta_x_gyro = yk + velocidad_gyro * PERIODO/1000000; //OBS: ¿Qué pasa si el ciclo tarda más de lo esperado?
  theta_x_acc = (180/PI) * atan2(a.acceleration.y, a.acceleration.z) - theta_bias;

  yk_1 = yk;
  yk = alpha * theta_x_acc + (1 - alpha) * theta_x_gyro;

  uk_1 = uk;

  xk_1 = xk;
  xk = Ad * xk_1 + Bd * uk_1 + L * (yk_1 - Cd * xk_1);

  if(n_ciclo%CICLOS == 0){
    uk = (-1) * UK_REF;
  };

  int duty_cycle_servo = (int)mapFloat(uk, -90, 90, 600, 2400);
  miServo.writeMicroseconds(duty_cycle_servo);

  if((n_ciclo / CICLOS) % 2 == 0){
    uk = UK_REF;
  } else {
    uk = -UK_REF;
  };

  n_ciclo++;

  float datos[4] = {xk(0), yk, xk(1), velocidad_gyro};
  matlab_send(datos, 4);

  while(micros() - t_ini < PERIODO){}
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
