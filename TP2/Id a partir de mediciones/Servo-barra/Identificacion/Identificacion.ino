#include <NewPing.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define TRIGGER_PIN  6   // Pin de arduino conectado al pin del trigger
#define ECHO_PIN     7   // Pin de arduino conectado al pin del echo
#define MAX_DISTANCE 40 // Distancia máxima en centímetros
#define PERIODO 20000 //Período en us
#define N_MUESTRAS 50 //Cantidad de vececs que se mide el ángulo de la IMU para estimar el sesgo
#define C 0.0343 //Velocidad del sonido en cm/us
#define PRBS_AMPLITUD 20.0f //Amplitud de la PRBS en grados
#define PRBS_HOLD 10 //Cantidad de muestras que se mantendrá fija la entrada
#define PRBS_SEED 1UL //semilla

int prbs_counter = 0;
float u_prbs = PRBS_AMPLITUD;

#define ANGULO_SERVO_MAX 58.55
#define ANGULO_SERVO_MIN -46.84

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Setup
Servo miServo;
Adafruit_MPU6050 mpu;
float theta_bias = 0;

void setup() {
  miServo.attach(9); // pin PWM
  Serial.begin(115200);

  randomSeed(PRBS_SEED);

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

  sensors_event_t a, g, t;
  for(int i = 0; i < N_MUESTRAS; i++){ //Se calcula es sesgo como el promedio de (N_MUESTRAS) mediciones
    mpu.getEvent(&a, &g, &t);
    theta_bias += (180/PI) * atan2(a.acceleration.y, a.acceleration.z);
    delay(10);
  }
  theta_bias/=N_MUESTRAS;
}


float theta_x_acc = 0; //Posición angular estimada por el acelerómetro, asumimos que inicialmente es 0
float theta_x_fc = 0; 
float theta_x_gyro_fc = 0;

float alfa = 0.1;

void loop() {
  unsigned long t_ini = micros();

  //Sensor ultrasónico
  float posicion = posicion_carrito();

  //IMU
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  
  theta_x_acc = (180/PI) * atan2(a.acceleration.y, a.acceleration.z) - theta_bias;
  theta_x_gyro_fc = theta_x_fc + (180/PI) * g.gyro.x * (PERIODO/1000000.0f);
  
  theta_x_fc = alfa * theta_x_acc + (1 - alfa) * theta_x_gyro_fc;

  
  float angulo = actualizarPRBS();

  if(angulo < ANGULO_SERVO_MIN){
    angulo = ANGULO_SERVO_MIN;
  } else if(angulo > ANGULO_SERVO_MAX){
    angulo = ANGULO_SERVO_MAX;
  }
  int duty_cycle_servo = (int)mapFloat(angulo, -90, 90, 600, 2400);
  miServo.writeMicroseconds(duty_cycle_servo);

  float datos[3] = {angulo, theta_x_fc, posicion};
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
  unsigned long tiempo = sonar.ping(MAX_DISTANCE);
  float medicion = (C/2) * tiempo; 
  if(tiempo == 0){ //Se excedió el tiempo máximo, el carrito se cayó de la barra o está pegado al sensor
    medicion = 15.5; //Quiero que cuando el carrito se caiga, el sensor lo detecte en 0.
  } else if(medicion < 2){
    medicion = 2; //Para las mediciones menores a 2cm
  }
  return mapFloat(medicion, 15.5, 32, 0, 17.25);
}

float actualizarPRBS() { //Genera +PRBS_AMPLITUD ó -PRBS_AMPLITUD cada PRBS_HOLD muestras
  if (prbs_counter == 0) {
    if (random(0, 2) == 0) {
      u_prbs = -PRBS_AMPLITUD;
    } else {
      u_prbs = PRBS_AMPLITUD;
    }
  }

  prbs_counter++;

  if (prbs_counter >= PRBS_HOLD) {
    prbs_counter = 0;
  }

  return u_prbs;
}
