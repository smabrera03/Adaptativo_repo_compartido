#Código para registrar las mediciones - Este programa no mueve el servomotor: únicamente mide la distancia y envía un CSV por el monitor serie.
# Los pines 9 y 10 son de ejemplo. No conviene dejar simultáneamente este programa y el firmware anterior intentando controlar el mismo sensor.


/*
  CARACTERIZACION DEL RUIDO - HC-SR04
  Planta servo-barra-carrito

  Salida CSV:
  t_us,distancia_cm,valid,echo_us

  Modificar TRIG_PIN y ECHO_PIN
  segun el conexionado real.
*/

const byte TRIG_PIN = 9;
const byte ECHO_PIN = 10;

// Muestreo nominal: 50 Hz
const unsigned long TS_US = 20000UL;

// Ajustar segun la distancia maxima esperada.
// 5000 us equivale aproximadamente a 86 cm
// de recorrido unidireccional.
const unsigned long ECHO_TIMEOUT_US = 5000UL;

unsigned long nextSampleUs = 0;

void setup() {

  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  delay(1000);

  Serial.println(
    "t_us,distancia_cm,valid,echo_us"
  );

  nextSampleUs = micros();

}

void loop() {

  unsigned long now = micros();

  // Comparacion robusta ante desborde de micros()
  if ((long)(now - nextSampleUs) < 0) {
    return;
  }

  // Programar la siguiente muestra
  nextSampleUs += TS_US;

  // Pulso de disparo
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  // Tiempo de inicio de la medicion
  unsigned long tSample = micros();

  // Duracion del eco
  unsigned long echoUs = pulseIn(
    ECHO_PIN,
    HIGH,
    ECHO_TIMEOUT_US
  );

  bool valid = echoUs > 0;

  // Velocidad aproximada del sonido:
  // 0.0343 cm/us
  float distanceCm = NAN;

  if (valid) {
    distanceCm = echoUs * 0.0343f / 2.0f;
  }

  // Salida CSV
  Serial.print(tSample);
  Serial.print(",");

  if (valid) {
    Serial.print(distanceCm, 4);
  } else {
    Serial.print("nan");
  }

  Serial.print(",");
  Serial.print(valid ? 1 : 0);
  Serial.print(",");
  Serial.println(echoUs);

  // Si el procesamiento se atraso, evitar
  // una rafaga de disparos para "recuperar" tiempo.
  if ((long)(micros() - nextSampleUs) >= 0) {
    nextSampleUs = micros() + TS_US;
  }

}
