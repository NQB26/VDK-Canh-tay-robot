#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

#define UART_BAUD 9600
#define UART_RX   4
#define UART_TX   5
#define PASSKEY   191106

#define LED_PIN        7
#define LED_ACTIVE_LOW 0      // 0: LED sáng khi chân ở mức cao; đổi thành 1 nếu LED sáng khi chân ở mức thấp
#define BLINK_MS       100

BLECharacteristic *ch;
bool connected = false;

void ledWrite(bool on) { digitalWrite(LED_PIN, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW); }

void ledUpdate() {
  static uint32_t t = 0;
  static bool state = false;
  if (connected) { ledWrite(true); return; }          // đã kết nối: sáng ổn định
  if (millis() - t >= BLINK_MS) {                     // chưa kết nối: nhấp nháy
    t = millis();
    state = !state;
    ledWrite(state);
  }
}

class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { connected = true; }
  void onDisconnect(BLEServer *s) override {
    connected = false;
    s->startAdvertising();
  }
};

class WriteCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    auto v = c->getValue();
    Serial1.write((const uint8_t *)v.c_str(), v.length());
  }
};

uint8_t buf[20];
size_t n = 0;
uint32_t lastByte = 0;

void flushToBle() {
  if (n && connected) { ch->setValue(buf, n); ch->notify(); }
  n = 0;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  BLEDevice::init("RobotArm");

  // Bảo mật: PIN cố định, ghép nối có MITM + bonding + Secure Connections
  BLESecurity *sec = new BLESecurity();
  sec->setPassKey(true, PASSKEY);          // true = PIN cố định
  sec->setCapability(ESP_IO_CAP_OUT);      // ESP32 "hiển thị" PIN, máy điện thoại/PC nhập
  sec->setAuthenticationMode(true, true, true);   // bonding, MITM, secure connection

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());

  BLEService *svc = server->createService(BLEUUID("FFE0"));
  ch = svc->createCharacteristic(
      BLEUUID("FFE1"),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_NOTIFY);

  // Bắt buộc liên kết đã mã hoá mới được đọc/ghi
  ch->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  ch->addDescriptor(new BLE2902());
  ch->setCallbacks(new WriteCb());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID("FFE0"));
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  ledUpdate();
  // while (Serial1.available()) {
  //   buf[n++] = Serial1.read();
  //   lastByte = millis();
  //   if (n == sizeof(buf)) flushToBle();
  // }
  // if (n && millis() - lastByte > 5) flushToBle();
}