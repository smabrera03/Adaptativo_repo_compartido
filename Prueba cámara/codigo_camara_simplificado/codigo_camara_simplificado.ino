/*

Este es el mismo código que el que está en el archivo codigo_camara_og.ino, solo que simplificado. Solamente obtiene la posición x de la bola. No incluye las instancias de calibración ni debug. 

*/


#include "esp_camera.h"
#include <math.h>
#include <stdlib.h>

// ============================================================
// CONFIGURACIÓN DE LA DETECCIÓN
// ============================================================

const int roiWidth = 800;
const int roiHeight = 300;

const uint32_t Nmin = 100;
const uint32_t Nmax = 12000;

const float circularityMax = 10.0f;

const uint8_t rMin = 19;
const uint8_t gMin = 10;
const uint8_t bMax = 5;

uint16_t *rgb565 = nullptr;

// ============================================================
// PINES DE LA ESP32-CAM AI THINKER
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
// RESULTADO DE LA DETECCIÓN
// ============================================================

struct BallDetection {
  bool valid;
  uint32_t pixels;
  float x;
  float circularity;
};

// ============================================================
// DETECCIÓN DE LA BOLA
// ============================================================

BallDetection detectBallRGB565(uint16_t *pixels) {

  BallDetection result = {};
  uint32_t N = 0;

  uint64_t sumX = 0;
  uint64_t sumY = 0;
  uint64_t sumXX = 0;
  uint64_t sumYY = 0;

  for (int y = 0; y < roiHeight; y++) {

    int offset = y * roiWidth;

    for (int x = 0; x < roiWidth; x++) {

      uint16_t p = pixels[offset + x];

      uint8_t r = (p >> 11) & 0x1F;
      uint8_t g = (p >> 5)  & 0x3F;
      uint8_t b = p & 0x1F;

      // Selección de píxeles por color
      if (r < rMin) continue;
      if (g < gMin) continue;
      if (b > bMax) continue;

      N++;

      sumX  += x;
      sumY  += y;
      sumXX += (uint64_t)x * x;
      sumYY += (uint64_t)y * y;
    }
  }

  result.pixels = N;

  if (N == 0) {
    return result;
  }

  // Centroide
  float mx = (float)sumX / N;
  float my = (float)sumY / N;

  result.x = mx;

  // Varianzas de los píxeles detectados
  float varX = (float)sumXX / N - mx * mx;
  float varY = (float)sumYY / N - my * my;

  // Evitar varianzas negativas por redondeo numérico
  if (varX < 0) varX = 0;
  if (varY < 0) varY = 0;

  float vmin = fminf(varX, varY);
  float vmax = fmaxf(varX, varY);

  if (vmin > 0) {
    result.circularity = vmax / vmin;
  } else {
    result.circularity = 9999.0f;
  }

  // Validación por cantidad de píxeles y forma
  result.valid =
      (N >= Nmin && N <= Nmax) &&
      (result.circularity <= circularityMax);

  return result;
}

// ============================================================
// INICIALIZACIÓN
// ============================================================

void setup() {

  Serial.begin(115200);

  // Configuración de la cámara
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

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 5;
  config.fb_count = 2;

  // Inicializar cámara
  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf(
      "Error inicializando cámara: 0x%x\n",
      err);
    while (true) {
      delay(1000);
    }
  }
  Serial.println("Camara OK");
  // Configurar ROI por hardware
  sensor_t *s = esp_camera_sensor_get();

  s->set_res_raw(
    s,
    0, 0, 0, 0,
    0, 600,
    1600, roiHeight * 2,
    roiWidth, roiHeight,
    true, true
  );
  Serial.println("ROI por Hardware ... OK");
  // Balance de blancos
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 1);

  // Reservar memoria para la imagen RGB565
  rgb565 = (uint16_t *)malloc(
    (size_t)roiWidth * roiHeight * sizeof(uint16_t)
  );
  
  if (rgb565 == nullptr) {
    Serial.println("ERROR: no hay memoria para RGB565");
    while (true) {
      delay(1000);
    }
  }
}

// ============================================================
// BUCLE PRINCIPAL
// ============================================================
uint32_t fpsFrames = 0;
uint32_t fpsLastReport = 0;
float fps = 0;

void loop() {

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

  // Detectar bola
  BallDetection ball = detectBallRGB565(rgb565);

  Serial.printf(
      "x:%.1f\tfps:%.1f\tvalid:%d\n",
      ball.x,
      fps,
      ball.valid);
  
  uint32_t now = millis();
  if (now - fpsLastReport >= 1000) {
    fps = fpsFrames * 1000.0f / (now - fpsLastReport);

    fpsFrames = 0;
    fpsLastReport = now;
  }
}

