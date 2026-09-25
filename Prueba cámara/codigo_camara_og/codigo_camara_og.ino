// ============================================================
// FIRMWARE DE VISION para detectar la posición de la bola
// HW: AI Thinker ESP32-CAM
// ============================================================


#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>

// ============================================================
// CONFIGURACIÓN DE VISIÓN
// ============================================================
const int IMAGE_WIDTH = 800;
const int IMAGE_HEIGHT = 600;

int roiWidth = 800;
//int roiHeight = 64;
int roiHeight = 128;
// Vamos a detectar una BIC azul
const uint32_t Nmin = 100;
const uint32_t Nmax = 1000;
const float circularityMax = 1.5;
const uint8_t rMin = 26;
const uint8_t gMin = 28;
const uint8_t bMax = 10;

uint16_t *rgb565 = nullptr;

typedef struct BallDetection {
  bool valid;
  uint32_t pixels;

  float x;
  float y;

  float varX;
  float varY;
  float covXY;

  float lambda1;
  float lambda2;

  float sigma;
  float circularity;
} BallDetection;


// ============================================================
// WIFI
// ============================================================
#define AP_MODE

#ifdef AP_MODE
const char *AP_SSID = "ESP32CAM";
const char *AP_PASS = "12345678";
#else
const char *WIFI_SSID = "******";
const char *WIFI_PASS = "******";
#endif

// ============================================================
// UDP
// ============================================================
IPAddress controllerIP(192, 168, 1, 100);
const uint16_t UDP_PORT = 5000;

WiFiUDP udp;

// ============================================================
// HTTP
// ============================================================
WebServer server(80);

// ============================================================
// CÁMARA
// ============================================================
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define LED_PIN 4
// ============================================================
// MODOS
// ============================================================
enum Mode {
  MODE_PRODUCTION,
  MODE_DEBUG,
  MODE_CALIB
};

Mode mode = MODE_PRODUCTION;

// ============================================================
// PROTOCOLO UDP
// ============================================================
struct Measurement {
  uint32_t sequence;
  int16_t x;
  int16_t y;
};

uint32_t sequence = 0;

// ============================================================
// HTTP /capture
// ============================================================
void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    server.send(
      500,
      "text/plain",
      "Error capturando imagen");
    return;
  }

  WiFiClient client = server.client();

  server.setContentLength(fb->len);

  server.send(
    200,
    "image/jpeg");

  client.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}


// ============================================================
// HTTP /
// ============================================================



void handleRoot() {
  String html;

  html += "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta name='viewport' content='width=device-width'>";
  html += "<title>ESP32-CAM Rampa</title>";
  html += "</head>";
  html += "<body>";

  html += "<h2>ESP32-CAM</h2>";

  html += "<p>Modo: ";

  if (mode == MODE_DEBUG)
    html += "DEBUG";
  else if (mode == MODE_PRODUCTION)
    html += "PRODUCCION";
  else
    html += "CALIBRACION";

  html += "</p>";

  html += "<p>";
  html += "ROI: ";
  html += String(roiWidth);
  html += " x ";
  html += String(roiHeight);
  html += " px";
  html += "</p>";

  html += "<p>";
  html += "<a href='/capture'>";
  html += "Capturar imagen";
  html += "</a>";
  html += "</p>";

  html += "</body>";
  html += "</html>";

  server.send(200, "text/html", html);
}

// ============================================================
// HTTP
// ============================================================
void startHTTP() {
  server.on("/", handleRoot);
  server.on("/capture", handleCapture);

  server.begin();

  Serial.println("HTTP iniciado");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/capture");
}


void stopHTTP() {
  server.stop();
  Serial.println("HTTP detenido");
}


// ============================================================
// CAMBIO DE MODO
// ============================================================
void setMode(Mode newMode) {
  if (mode == newMode)
    return;

  mode = newMode;

  if (mode == MODE_DEBUG) {
    Serial.println("Modo DEBUG");
    startHTTP();

  } else {
    stopHTTP();
    if (mode == MODE_PRODUCTION)
      Serial.println("Modo PRODUCCION");
    else
      Serial.println("Modo CALIBRACION");
  }
}


// ============================================================
// UDP
// ============================================================
void sendMeasurement() {
  Measurement m;

  m.sequence = sequence++;

  m.x = 0;
  m.y = 0;

  /*udp.beginPacket(controllerIP, udpPort);
  udp.write(
    (uint8_t*)&m,
    sizeof(m)
  );
  udp.endPacket();
  */
}


// ============================================================
// STATUS
// ============================================================
void printStatus() {
  Serial.println();
  Serial.println("========== STATUS ==========");

  Serial.print("Modo: ");

  if (mode == MODE_DEBUG)
    Serial.println("DEBUG");
  else if (mode == MODE_PRODUCTION)
    Serial.println("PRODUCCION");
  else
    Serial.println("CALIBRACION");

  Serial.print("WiFi: ");
  Serial.println(
    WiFi.status() == WL_CONNECTED
      ? "conectado"
      : "desconectado");

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("ROI: ");
  Serial.print(roiWidth);
  Serial.print(" x ");
  Serial.println(roiHeight);

  /*
  Serial.print("UDP destino: ");
  Serial.print(controllerIP);
  Serial.print(":");
  Serial.println(udpPort);
  */

  Serial.println("============================");
  Serial.println();
}


// ============================================================
// HELP
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("Comandos:");
  Serial.println();
  Serial.println("  help");
  Serial.println("      muestra esta ayuda");
  Serial.println();
  Serial.println("  status");
  Serial.println("      muestra configuracion");
  Serial.println();
  Serial.println("  debug");
  Serial.println("      cambia a modo DEBUG");
  Serial.println();
  Serial.println("  prod");
  Serial.println("      cambia a modo PRODUCCION");
  Serial.println();
  Serial.println("  toggle");
  Serial.println("      alterna el modo");
  Serial.println();
  Serial.println("  calib");
  Serial.println("      muestra los valores de color en el pixel central");
  Serial.println();
  Serial.println("  roi");
  Serial.println("      muestra la ROI");
  Serial.println();
  //Serial.println("  roi <ancho> <alto>");
  //Serial.println("      modifica la ROI");
  //Serial.println();
  Serial.println("  exp <valor>");
  Serial.println("      modifica la exposición (0 a 1200)");
  Serial.println();
  Serial.println("  gain <valor>");
  Serial.println("      modifica la ganancia del sensor (0 a 30)");
  Serial.println();
  Serial.println("  wb <valor>");
  Serial.println("      modifica el perfil de balance de blancos  (1: Sunny 2:Cloudy 3:Office 4:Home)");
  Serial.println();
  Serial.println("  led <valor>");
  Serial.println("      modifica la iluminación (0 a 255)");
  Serial.println();
  Serial.println("  capture");
  Serial.println("      muestra la URL de captura");
  Serial.println();
}


// ============================================================
// ROI
// ============================================================
// Lo desactivé porque al hacerla por HW se pone muy fragil modificar al vuelo
void setROI(int width, int height) {
  if (width < 1 || width > IMAGE_WIDTH) {
    Serial.println("Error: ancho fuera de rango");
    return;
  }

  if (height < 1 || height > IMAGE_HEIGHT) {
    Serial.println("Error: alto fuera de rango");
    return;
  }

  roiWidth = width;
  roiHeight = height;

  Serial.print("ROI configurada: ");
  Serial.print(roiWidth);
  Serial.print(" x ");
  Serial.println(roiHeight);
}


// ============================================================
// PARSER SERIAL
// ============================================================

void processCommand(String command) {
  command.trim();

  if (command.length() == 0)
    return;

  // ----------------------------------------------------------
  // HELP
  // ----------------------------------------------------------
  if (command == "help") {
    printHelp();
    return;
  }

  // ----------------------------------------------------------
  // STATUS
  // ----------------------------------------------------------
  if (command == "status") {
    printStatus();
    return;
  }

  // ----------------------------------------------------------
  // DEBUG
  // ----------------------------------------------------------
  if (command == "debug") {
    setMode(MODE_DEBUG);
    return;
  }

  // ----------------------------------------------------------
  // CALIBRACION
  // ----------------------------------------------------------
  if (command == "calib") {
    setMode(MODE_CALIB);
    return;
  }

  // ----------------------------------------------------------
  // PRODUCCION
  // ----------------------------------------------------------
  if (command == "prod") {
    setMode(MODE_PRODUCTION);
    return;
  }

  // ----------------------------------------------------------
  // TOGGLE
  // ----------------------------------------------------------
  if (command == "toggle") {
    if (mode == MODE_DEBUG)
      setMode(MODE_PRODUCTION);
    else if (mode == MODE_PRODUCTION)
      setMode(MODE_CALIB);
    else if (mode == MODE_CALIB)
      setMode(MODE_DEBUG);
    return;
  }

  // ----------------------------------------------------------
  // CAPTURE
  // ----------------------------------------------------------
  if (command == "capture") {
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println("/capture");
    return;
  }

  // ----------------------------------------------------------
  // ROI
  // ----------------------------------------------------------

  if (command == "roi") {
    Serial.print("ROI: ");
    Serial.print(roiWidth);
    Serial.print(" x ");
    Serial.println(roiHeight);
    return;
  }

  /*
  if (command.startsWith("roi ")) {

    int width;
    int height;

    int n = sscanf(
      command.c_str(),
      "roi %d %d",
      &width,
      &height);

    if (n == 2) {

      setROI(width, height);

    } else {

      Serial.println(
        "Uso: roi <ancho> <alto>");
    }    

    return;
  }
  */

  // ----------------------------------------------------------
  // EXPOSICION
  // ----------------------------------------------------------
  if (command.startsWith("exp ")) {

    int valor;
    int n = sscanf(
      command.c_str(),
      "exp %d",
      &valor);

    if (n == 1) {
      set_AEC_AGC_params(valor, -1);
    } else {
      Serial.println(
        "Uso: exp <valor>");
    }
    return;
  }

  // ----------------------------------------------------------
  // GANANCIA
  // ----------------------------------------------------------
  if (command.startsWith("gain ")) {

    int valor;
    int n = sscanf(
      command.c_str(),
      "gain %d",
      &valor);

    if (n == 1) {
      set_AEC_AGC_params(-1, valor);
    } else {
      Serial.println(
        "Uso: gain <valor>");
    }
    return;
  }

  // ----------------------------------------------------------
  // GANANCIA
  // ----------------------------------------------------------
  if (command.startsWith("wb ")) {

    int valor;
    int n = sscanf(
      command.c_str(),
      "wb %d",
      &valor);

    if (n == 1) {
      if (valor >= 1 && valor <= 4) {
        sensor_t *s = esp_camera_sensor_get();
        s->set_wb_mode(s, valor);
      }
    } else {
      Serial.println(
        "Uso: wb <valor>");
    }
    return;
  }

  // ----------------------------------------------------------
  // EXPOSICION
  // ----------------------------------------------------------
  if (command.startsWith("led ")) {

    int valor;
    int n = sscanf(
      command.c_str(),
      "led %d",
      &valor);

    if (n == 1) {
      if (valor >= 0 && valor < 256)
        ledcWrite(LED_PIN, valor);
    } else {
      Serial.println(
        "Uso: led <valor>");
    }
    return;
  }

  // ----------------------------------------------------------
  // DESCONOCIDO
  // ----------------------------------------------------------

  Serial.print("Comando desconocido: ");
  Serial.println(command);
}


// ============================================================
// SERVICIO SERIAL
// ============================================================

void serialService() {
  static String command;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (command.length() > 0) {
        processCommand(command);
        command = "";
      }
    } else {
      command += c;
    }
  }
}


// ============================================================
// VISION
// ============================================================
void getColors(uint16_t *pixels) {
  uint16_t p = pixels[roiWidth / 2];
  uint8_t r = (p >> 11) & 0x1F;
  uint8_t g = (p >> 5) & 0x3F;
  uint8_t b = p & 0x1F;

  Serial.printf("b:%d\tr:%d\tg:%d\n", b, r, g);
}

BallDetection detectBallRGB565(
  uint16_t *pixels,
  int roiWidth,
  int roiHeight,
  uint8_t rMin,
  uint8_t gMin,
  uint8_t bMax,
  uint32_t nmin,
  uint32_t nmax,
  float circularityMax) {
  BallDetection result = {};

  uint32_t N = 0;

  uint64_t sumX = 0;
  uint64_t sumY = 0;
  uint64_t sumXX = 0;
  uint64_t sumYY = 0;

  for (int y = 0; y < roiHeight; y += 1) {

    int offset = y * roiWidth;

    for (int x = 0; x < roiWidth; x++) {

      uint16_t p = pixels[offset + x];

      uint8_t r = (p >> 11) & 0x1F;
      uint8_t g = (p >> 5) & 0x3F;
      uint8_t b = p & 0x1F;

      if (r < rMin)
        continue;
      if (g < gMin)
        continue;
      if (b > bMax)
        continue;

      // Estadísticos
      N++;

      sumX += x;
      sumY += y;

      sumXX += (uint32_t)x * x;
      sumYY += (uint32_t)y * y;
    }
  }

  result.pixels = N;

  if (N == 0)
    return result;

  float mx = (float)sumX / N;
  float my = (float)sumY / N;

  result.x = mx;
  result.y = my;

  result.varX =
    (float)sumXX / N - mx * mx;

  result.varY =
    (float)sumYY / N - my * my;

  result.sigma =
    sqrtf((result.varX + result.varY) / 2.0f);

  float vmin = min(result.varX, result.varY);
  float vmax = max(result.varX, result.varY);

  if (vmin > 0)
    result.circularity = vmax / vmin;
  else
    result.circularity = 9999.0f;

  result.valid =
    N >= nmin && N <= nmax;  // && result.circularity <= circularityMax;

  return result;
}


// ============================================================
// CAMARA
// ============================================================
void set_AEC_AGC_params(int sensor_exposure, int sensor_gain) {
  sensor_t *s = esp_camera_sensor_get();
  if (sensor_exposure >= 0 && sensor_exposure <= 1200) {
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, sensor_exposure);
  }
  if (sensor_gain >= 0 && sensor_gain <= 30) {
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, sensor_gain);
  }
}


// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.println("============================");
  Serial.println(" ESP32-CAM  <->  BOLA-RAMPA");
  Serial.println("============================");

  // ----------------------------------------------------------
  // CÁMARA
  // ----------------------------------------------------------
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;  // A los fines de reservar espacio: después hacemos escaneo parcial
  config.jpeg_quality = 5;            // Buena calidad de compresión
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf(
      "Error inicializando cámara: 0x%x\n",
      err);
    while (true)
      delay(1000);
  }
  Serial.println("Camara OK");

  // ---- CONFIGURAR EL ESCANEO PARCIAL (ROI) POR HARDWARE ----
  sensor_t *s = esp_camera_sensor_get();
  //s->set_res_raw(s, 0, 0, 0, 0, offsetX, offsetY, totalX, totalY, roiWidth, roiHeight, true, true);
  s->set_res_raw(s, 0, 0, 0, 0, 0, 600, 1600, roiHeight * 2, roiWidth, roiHeight, true, true);
  Serial.println("ROI por Hardware ... OK");

  // Ajustamos el balance de blancos
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 1);

  // Preparo el buffer de la imagen
  rgb565 = (uint16_t *)malloc(roiWidth * roiHeight * sizeof(uint16_t));
  if (!rgb565) {
    Serial.println("ERROR: no hay memoria para RGB565");
    return;
  }

  ledcAttach(LED_PIN, 5000, 8);  // 5 kHz, 8 bits
  ledcWrite(LED_PIN, 0);         // 50 % aproximadamente

  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

#ifdef AP_MODE
  WiFi.mode(WIFI_AP);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);

  if (!ok) {
    Serial.println("ERROR al iniciar AP");
    while (true) delay(1000);
  }

  Serial.println("AP iniciado");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
#else
  WiFi.begin(
    WIFI_SSID,
    WIFI_PASS);

  Serial.print("Conectando WiFi");

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);
    Serial.print(".");
  }

  Serial.println();

  Serial.println("WiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
#endif

  // ----------------------------------------------------------
  // UDP
  // ----------------------------------------------------------

  udp.begin(UDP_PORT);

  // ----------------------------------------------------------
  // MODO INICIAL
  // ----------------------------------------------------------

  mode = MODE_PRODUCTION;

  Serial.println();
  Serial.println("Modo inicial: PRODUCCION");
  Serial.println();
  Serial.println("Escriba 'help' para ver comandos.");
  Serial.println();
}



// ============================================================
// LOOP
// ============================================================
uint32_t fpsFrames = 0;
uint32_t fpsLastReport = 0;
float fps = 0;

void loop() {
  // Siempre disponible
  serialService();

  // Adquiero un frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    // Convierto el buffer del frame JPG->RGB555
    bool ok = jpg2rgb565(
      fb->buf,
      fb->len,
      (uint8_t *)rgb565,
      JPG_SCALE_NONE);

    esp_camera_fb_return(fb);
    fb = nullptr;
    fpsFrames++;
  }

  if (mode == MODE_CALIB) {
    getColors(rgb565);
  } else if (mode == MODE_DEBUG) {
    server.handleClient();
  } else {
    // --------------------------------------------------------
    // PRODUCCION
    // --------------------------------------------------------
    // Busco la bola dentro de la ventana
    BallDetection ball = detectBallRGB565(
      rgb565,
      roiWidth, roiHeight,
      rMin, gMin, bMax,
      Nmin, Nmax,
      circularityMax);

    Serial.printf(
      "x:%.1f\ty:%.1f\tN:%lu\tsigma:%.2f\tC:%.2f\tfps:%.1f\n",
      ball.x,
      ball.y,
      ball.pixels,
      ball.sigma,
      ball.circularity,
      fps);
  }

  uint32_t now = millis();
  if (now - fpsLastReport >= 1000) {
    fps = fpsFrames * 1000.0f / (now - fpsLastReport);

    fpsFrames = 0;
    fpsLastReport = now;
  }
}