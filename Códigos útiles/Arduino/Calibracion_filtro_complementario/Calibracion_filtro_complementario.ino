#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define PERIODO 20000 //período del ciclo en us.
#define N_MUESTRAS 50 //Cantidad de vececs que se mide el ángulo de la IMU para estimar el sesgo
#define N 50 //Cantidad de ciclos para cambiar el comando

Adafruit_MPU6050 mpu;
float theta_bias = 0;
Servo miServo;

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
  delay(1000);
  sensors_event_t a, g, t;
  for(int i = 0; i < N_MUESTRAS; i++){ //Se calcula es sesgo como el promedio de (N_MUESTRAS) mediciones
    mpu.getEvent(&a, &g, &t);
    theta_bias += (180/PI) * atan2(a.acceleration.y, a.acceleration.z);
  }
  theta_bias/=N_MUESTRAS;
}


float theta_x_gyro = 0; //Posición angular estimada por el giroscopio, asumimos que inicialmente es 0
float theta_x_gyro_fc = 0; //predicción del filtro complementario
float theta_x_acc = 0; //Posición angular estimada por el giroscopio, asumimos que inicialmente es 0
float theta_x_fc = 0; //Posición angular estimada por el filtro complementario

float alpha = 0.1;

float angulo_comando = 0;

int contador = 0;
void loop() {
  unsigned long t_ini = micros();

  int duty_cycle_servo = (int)mapFloat(angulo_comando, -90, 90, 600, 2400); //544 y 2400 son los valores que acepta writeMicroseconds según la documentación
  miServo.writeMicroseconds(duty_cycle_servo);

  //Estimo el ángulo
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  theta_x_gyro = theta_x_gyro + (180/PI) * g.gyro.x * PERIODO/1000000; //OBS: ¿Qué pasa si el ciclo tarda más de lo esperado?
  theta_x_acc = (180/PI) * atan2(a.acceleration.y, a.acceleration.z) - theta_bias;
  theta_x_gyro_fc = theta_x_fc + (180/PI) * g.gyro.x * PERIODO/1000000;

  theta_x_fc = alpha * theta_x_acc + (1 - alpha) * theta_x_gyro_fc;

  float datos[4] = {theta_x_gyro, theta_x_acc, theta_x_gyro_fc, theta_x_fc};
  matlab_send(datos, 4);

  contador++;
  if(contador%N == 0){
    if(angulo_comando == 0) angulo_comando = 30;
    else angulo_comando = 0;
  }
  while (micros() - t_ini < 20000) {}
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