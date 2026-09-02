#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define PERIODO 20000 //período del ciclo en us. En este caso se elige un preíodo grande para que el servo tenga tiempo para estabilizarse
#define N 100 //Cantidad de ciclos para cambiar el comando
#define N_MUESTRAS 50 //Cantidad de vececs que se mide el ángulo de la IMU para estimar el sesgo

Adafruit_MPU6050 mpu;
Servo miServo;

float theta_bias = 0;

typedef enum {HORIZONTAL_SUBIENDO, ANTIHORARIO, HORIZONTAL_BAJANDO, HORARIO} estado_t;
#define ANGULO_1 0
#define ANGULO_2 45

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


float theta_x_acc = 0; //Posición angular estimada por el acelerómetro, asumimos que inicialmente es 0
float theta_x_fc = 0; //Posición angular estimada por el filtro complementario
float theta_x_gyro_fc = 0; //predicción del filtro complementario

float alpha = 0.1;

estado_t estado = HORIZONTAL_SUBIENDO;
float angulo_comando = ANGULO_1;
int contador = 0;

void loop() {
  unsigned long t_ini = micros();

  int duty_cycle_servo = (int)mapFloat(angulo_comando, -90, 90, 600, 2400); //544 y 2400 son los valores que acepta writeMicroseconds según la documentación
  miServo.writeMicroseconds(duty_cycle_servo);

  //Estimo el ángulo
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  theta_x_acc = (180/PI) * atan2(a.acceleration.y, a.acceleration.z) - theta_bias;
  theta_x_gyro_fc = theta_x_fc + (180/PI) * g.gyro.x * (PERIODO/1000000);

  theta_x_fc = alpha * theta_x_acc + (1 - alpha) * theta_x_gyro_fc;

  float datos[2] = {(float)angulo_comando, theta_x_fc};
  //Envío los datos a matlab
  matlab_send(datos, 2);

  contador++;

  if(contador%N == 0){
    switch(estado){
      case HORIZONTAL_SUBIENDO:
        estado = ANTIHORARIO;
        angulo_comando = ANGULO_2;
        break;
      
      case ANTIHORARIO:
        estado = HORIZONTAL_BAJANDO;
        angulo_comando = ANGULO_1;
        break;
      
      case HORIZONTAL_BAJANDO:
        estado = HORARIO;
        angulo_comando = (-1) * ANGULO_2;
        break;

      case HORARIO:
        estado = HORIZONTAL_SUBIENDO;
        angulo_comando = ANGULO_1;
        break;
    }
  }

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