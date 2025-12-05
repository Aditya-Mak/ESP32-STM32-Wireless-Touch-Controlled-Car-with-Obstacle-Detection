#include <WiFi.h>
#include <esp_now.h>
#include "esp_idf_version.h"   
#include <Wire.h>

// Must match the sender
typedef struct __attribute__((packed)) {
  uint16_t x;
  uint16_t y;
  uint8_t  touch;
} TouchPkt;


static const uint8_t I2C_ADDR = 0x28;
static const int SDA_PIN =21;
static const int SCL_PIN =22;

// Latest values to serve over I2C
static volatile uint16_t gX = 0xFFFF;
static volatile uint16_t gY = 0xFFFF;
portMUX_TYPE gTouchMux = portMUX_INITIALIZER_UNLOCKED;

static inline void setTouchXY(uint16_t x, uint16_t y, uint8_t touch) {
  portENTER_CRITICAL(&gTouchMux);
  if (!touch) { gX = 0xFFFF; gY = 0xFFFF; }   // sentinel for "no touch"
  else { gX = x; gY = y; }
  portEXIT_CRITICAL(&gTouchMux);
}

// 6-byte frame: 0xAA, X_L, X_H, Y_L, Y_H, CRC
void onI2CRequest() {
  uint16_t x, y;
  // take a consistent snapshot
  portENTER_CRITICAL(&gTouchMux);
  x = gX; y = gY;
  portEXIT_CRITICAL(&gTouchMux);

  uint8_t f[6];
  f[0] = 0xAA;                    //header
  f[1] = (uint8_t)(x & 0xFF);     //X low byte
  f[2] = (uint8_t)(x >> 8);       //x high byte
  f[3] = (uint8_t)(y & 0xFF);
  f[4] = (uint8_t)(y >> 8);
  f[5] = (uint8_t)((f[0] + f[1] + f[2] + f[3] + f[4]) & 0xFF); // simple CRC
  Wire.write(f, sizeof(f));
}
// ------------------------------------------


#if ESP_IDF_VERSION_MAJOR >= 5
void onRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(TouchPkt)) return;
  const TouchPkt* p = (const TouchPkt*)data;

  // update the values served over I2C
  setTouchXY(p->x, p->y, p->touch);

  // debug
  Serial.printf("X:%u  Y:%u  Touch:%u\n", p->x, p->y, p->touch);
}
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < (int)sizeof(TouchPkt)) return;
  const TouchPkt* p = (const TouchPkt*)data;

  // update the values served over I2C
  setTouchXY(p->x, p->y, p->touch);

  // debug
  Serial.printf("X:%u  Y:%u  Touch:%u\n", p->x, p->y, p->touch);
}
#endif


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) {}
  }
  esp_now_register_recv_cb(onRecv);

  Serial.println("\nESP32 Receiver Ready — waiting for touch data...");
  Wire.begin((int)I2C_ADDR, SDA_PIN, SCL_PIN, 100000); // addr, SDA, SCL, 100kHz
  Wire.onRequest(onI2CRequest);
}

void loop() {
  // Nothing here — ESP-NOW callback handles everything
}
