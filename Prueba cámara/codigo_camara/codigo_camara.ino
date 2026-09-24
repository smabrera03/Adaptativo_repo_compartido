#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ============================================================
// CONFIGURACION GENERAL
// ============================================================

#define AP_SSID "ESP32CAM_CARRITO"
#define AP_PASS "12345678"

#define IMAGE_WIDTH  320
#define IMAGE_HEIGHT 240

#define LED_PIN 4

WebServer server(80);

// ============================================================
// PINES ESP32-CAM AI THINKER
// ============================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0

#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5

#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// ============================================================
// DETECCION DE MARCA ROJA
// ============================================================

// Valores iniciales: se ajustaran con imagenes reales.

const uint8_t R_MIN = 150;
const uint8_t G_MAX = 110;
const uint8_t B_MAX = 110;

const uint32_t N_MIN = 20;
const uint32_t N_MAX = 15000;

// Buffer para imagen RGB565

uint16_t *rgb565 = nullptr;

// ============================================================
// ESTRUCTURA DE MEDICION
// ============================================================

struct Detection {

  bool valid;

  uint32_t pixels;

  float x;
  float y;

};

Detection lastDetection = {};

uint32_t lastTimestamp = 0;

uint32_t frames = 0;
uint32_t lastFpsTime = 0;

float fps = 0;

// ============================================================
// DETECTOR DE COLOR ROJO
// ============================================================

Detection detectRedMarker(
  const uint16_t *pixels,
  int width,
  int height
) {

  Detection result = {};

  uint32_t N = 0;

  uint64_t sumX = 0;
  uint64_t sumY = 0;

  for (int y = 0; y < height; y++) {

    for (int x = 0; x < width; x++) {

      uint16_t p = pixels[y * width + x];

      // RGB565 -> RGB888 aproximado

      uint8_t r = ((p >> 11) & 0x1F) * 255 / 31;
      uint8_t g = ((p >> 5)  & 0x3F) * 255 / 63;
      uint8_t b = (p & 0x1F) * 255 / 31;

      // Filtro de color

      bool isRed =
        r >= R_MIN &&
        g <= G_MAX &&
        b <= B_MAX &&
        r > g + 40 &&
        r > b + 40;

      if (!isRed) {
        continue;
      }

      N++;

      sumX += x;
      sumY += y;

    }

  }

  result.pixels = N;

  if (N < N_MIN || N > N_MAX) {

    result.valid = false;
    return result;

  }

  result.x = (float)sumX / N;
  result.y = (float)sumY / N;

  result.valid = true;

  return result;

}

// ============================================================
// HTTP: PAGINA PRINCIPAL
// ============================================================

void handleRoot() {

  String html = R"rawliteral(
<!DOCTYPE html>
<html>

<head>

<meta name="viewport"
      content="width=device-width, initial-scale=1">

<title>ESP32-CAM Carrito</title>

<style>

body {
  font-family: Arial;
  text-align: center;
  background: #f3f5f7;
}

img {
  max-width: 95%;
  border: 2px solid #333;
}

pre {
  font-size: 16px;
  background: white;
  padding: 15px;
}

</style>

</head>

<body>

<h2>ESP32-CAM - Carrito</h2>

<p>Imagen de la camara</p>

<img id="camera" src="/capture">

<h3>Medicion</h3>

<pre id="data">Esperando...</pre>

<script>

setInterval(function() {

  document.getElementById("camera").src =
    "/capture?t=" + Date.now();

}, 1000);

setInterval(function() {

  fetch("/data")
    .then(r => r.json())
    .then(d => {

      document.getElementById("data").textContent =
        JSON.stringify(d, null, 2);

    });

}, 500);

</script>

</body>

</html>
)rawliteral";

  server.send(200, "text/html", html);

}

// ============================================================
// HTTP: CAPTURA JPEG
// ============================================================

void handleCapture() {

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {

    server.send(
      500,
      "text/plain",
      "Error capturando imagen"
    );

    return;

  }

  WiFiClient client = server.client();

  server.setContentLength(fb->len);

  server.send(
    200,
    "image/jpeg",
    ""
  );

  client.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);

}

// ============================================================
// HTTP: ULTIMA MEDICION
// ============================================================

void handleData() {

  String json = "{";

  json += "\"t_ms\":";
  json += String(lastTimestamp);

  json += ",\"valid\":";
  json += lastDetection.valid ? "true" : "false";

  json += ",\"x_px\":";

  if (lastDetection.valid) {
    json += String(lastDetection.x, 2);
  } else {
    json += "null";
  }

  json += ",\"y_px\":";

  if (lastDetection.valid) {
    json += String(lastDetection.y, 2);
  } else {
    json += "null";
  }

  json += ",\"pixels\":";
  json += String(lastDetection.pixels);

  json += ",\"fps\":";
  json += String(fps, 2);

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );

}

// ============================================================
// INICIALIZACION CAMARA
// ============================================================

void initCamera() {

  camera_config_t config = {};

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

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA;

  config.jpeg_quality = 12;

  config.fb_count = 1;

  config.fb_location =
    psramFound() ? CAMERA_FB_IN_PSRAM
                 : CAMERA_FB_IN_DRAM;

  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {

    Serial.printf(
      "ERROR CAMARA: 0x%x\n",
      err
    );

    while (true) {
      delay(1000);
    }

  }

  Serial.println("Camara inicializada");

}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==========================");
  Serial.println(" ESP32-CAM CARRITO");
  Serial.println("==========================");

  Serial.printf(
    "PSRAM: %s\n",
    psramFound() ? "SI" : "NO"
  );

  initCamera();

  // Buffer RGB565

  size_t bufferSize =
    IMAGE_WIDTH *
    IMAGE_HEIGHT *
    sizeof(uint16_t);

  if (psramFound()) {

    rgb565 = (uint16_t *)ps_malloc(bufferSize);

  } else {

    rgb565 = (uint16_t *)malloc(bufferSize);

  }

  if (!rgb565) {

    Serial.println(
      "ERROR reservando RGB565"
    );

    while (true) {
      delay(1000);
    }

  }

  // WiFi AP

  WiFi.mode(WIFI_AP);

  if (!WiFi.softAP(AP_SSID, AP_PASS)) {

    Serial.println("ERROR iniciando AP");

    while (true) {
      delay(1000);
    }

  }

  Serial.println();
  Serial.println("WiFi iniciado");

  Serial.print("SSID: ");
  Serial.println(AP_SSID);

  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // HTTP

  server.on("/", handleRoot);

  server.on("/capture", handleCapture);

  server.on("/data", handleData);

  server.begin();

  Serial.println("HTTP iniciado");

  Serial.println();
  Serial.println(
    "t_ms,x_px,y_px,valid,pixels,fps"
  );

  lastFpsTime = millis();

}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // Atender solicitudes HTTP

  server.handleClient();

  // Adquirir imagen

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {

    delay(10);
    return;

  }

  // JPEG -> RGB565

  bool ok = jpg2rgb565(
    fb->buf,
    fb->len,
    (uint8_t *)rgb565,
    JPG_SCALE_NONE
  );

  esp_camera_fb_return(fb);

  if (!ok) {

    Serial.println(
      "ERROR convirtiendo JPEG"
    );

    delay(10);
    return;

  }

  // Detectar marca

  lastDetection = detectRedMarker(
    rgb565,
    IMAGE_WIDTH,
    IMAGE_HEIGHT
  );

  lastTimestamp = millis();

  frames++;

  // Calcular FPS

  uint32_t now = millis();

  if (now - lastFpsTime >= 1000) {

    fps =
      frames * 1000.0f /
      (now - lastFpsTime);

    frames = 0;

    lastFpsTime = now;

  }

  // Salida CSV

  Serial.print(lastTimestamp);
  Serial.print(",");

  if (lastDetection.valid) {

    Serial.print(lastDetection.x, 2);

  } else {

    Serial.print("nan");

  }

  Serial.print(",");

  if (lastDetection.valid) {

    Serial.print(lastDetection.y, 2);

  } else {

    Serial.print("nan");

  }

  Serial.print(",");

  Serial.print(
    lastDetection.valid ? 1 : 0
  );

  Serial.print(",");

  Serial.print(lastDetection.pixels);

  Serial.print(",");

  Serial.println(fps, 2);

  delay(20);

}