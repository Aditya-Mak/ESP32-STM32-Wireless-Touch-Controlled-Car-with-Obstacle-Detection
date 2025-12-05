#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>

// ---------- TSC2046 pins (ESP32 DevKit "Dx" style) ----------
#define CS   5    // D5
#define IRQ  27   // D27 (active LOW)
#define MISO 19   // D19
#define MOSI 23   // D23
#define SCK  18   // D18

// ---------- Receiver MAC (replace with your receiver's) ----------
uint8_t peerMAC[] = { 0x88,0x13,0xbf,0x00,0x24,0x34 };  // <--- put receiver MAC here

// Payload
typedef struct __attribute__((packed)) {
  uint16_t x;
  uint16_t y;
  uint8_t  touch;
} TouchPkt;
TouchPkt pkt;

SPISettings tsc(2000000, MSBFIRST, SPI_MODE0);
const uint8_t CMD_Y = 0x90, CMD_X = 0xD0;

inline bool touched() { return digitalRead(IRQ) == LOW; }

uint16_t read12(uint8_t cmd){
  SPI.beginTransaction(tsc);
  digitalWrite(CS, LOW);
  SPI.transfer(cmd);
  uint16_t v = (SPI.transfer(0) << 8) | SPI.transfer(0);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
  return v >> 4; // 0..4095
}

void setup() {
  Serial.begin(115200);
  // Touch SPI
  SPI.begin(SCK, MISO, MOSI, CS);
  pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);
  pinMode(IRQ, INPUT_PULLUP);

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("ESPNOW init fail"); while(1){} }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;            // follow current channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) { Serial.println("Add peer fail"); while(1){} }
}
void loop() {
  if (!touched()) { delay(10); return; }
  // read Y first (settling), then X
  uint16_t y = read12(CMD_Y);
  uint16_t x = read12(CMD_X);
  pkt.x = x; pkt.y = y; pkt.touch = 1;
  esp_now_send(peerMAC, (uint8_t*)&pkt, sizeof(pkt));   

   Serial.printf("TX X:%u Y:%u\n", x, y);
  delay(40); 
}
