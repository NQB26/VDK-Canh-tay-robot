#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

#define UART_BAUD 9600
#define UART_RX   4
#define UART_TX   5
#define PASSKEY   191106

BLECharacteristic *ch;
bool connected = false;

class SecCb : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override { return PASSKEY; }
  void onPassKeyNotify(uint32_t) override {}
  bool onConfirmPIN(uint32_t) override { return true; }
  bool onSecurityRequest() override { return true; }
  void onAuthenticationComplete(esp_ble_auth_cmpl_t) override {}
};

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
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  BLEDevice::init("RobotArm");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);
  BLEDevice::setSecurityCallbacks(new SecCb());

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());

  BLEService *svc = server->createService(BLEUUID("FFE0"));
  ch = svc->createCharacteristic(
      BLEUUID("FFE1"),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_NOTIFY);

  // Bắt buộc mã hoá mới được đọc/ghi
  ch->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  BLE2902 *cccd = new BLE2902();
  cccd->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  ch->addDescriptor(cccd);
  ch->setCallbacks(new WriteCb());
  svc->start();

  // PIN cố định, ESP32 đóng vai thiết bị chỉ hiển thị
  BLESecurity *sec = new BLESecurity();
  sec->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  sec->setCapability(ESP_IO_CAP_OUT);
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  uint32_t passkey = PASSKEY;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID("FFE0"));
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  while (Serial1.available()) {
    buf[n++] = Serial1.read();
    lastByte = millis();
    if (n == sizeof(buf)) flushToBle();
  }
  if (n && millis() - lastByte > 5) flushToBle();
}