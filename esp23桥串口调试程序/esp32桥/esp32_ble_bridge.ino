/*
 * ESP32 BLE-UART 透传桥 — 替代 JDY-23
 *
 * 架构:
 *   手机小程序 ←→ BLE ←→ ESP32 ←→ UART ←→ MSPM0G3507
 *
 * ESP32 模拟 JDY-23 的 BLE GATT 配置, 小程序和 MCU 固件零改动:
 *   Service UUID:       0000FFE0-0000-1000-8000-00805F9B34FB
 *   Characteristic UUID: 0000FFE1-0000-1000-8000-00805F9B34FB
 *   设备名:             JDY-23
 *
 * 硬件接线:
 *   ESP32 3.3V → 外部 3.3V (或 VIN 接 5V)
 *   ESP32 GND  → MCU GND (共地!)
 *   ESP32 GPIO16 (RX2) → MSPM0 PA10 (UART0 TX)
 *   ESP32 GPIO17 (TX2) → MSPM0 PA11 (UART0 RX)
 *
 * 烧录: Arduino IDE 选 ESP32 Dev Module, Flash Size 4MB
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>

#define SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"

#define BLE_UART_RX  16
#define BLE_UART_TX  17
#define BLE_UART_BAUD 9600

#define DEVICE_NAME   "JDY-23"
#define MAX_MTU       256

BLECharacteristic *pTxChar;
bool deviceConnected = false;
uint8_t txPacket[64];
uint8_t txPacketLen = 0;
uint32_t lastFlush = 0;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
    }
    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
        BLEDevice::startAdvertising();
    }
};

class MyCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        std::string value = pChar->getValue();
        if (value.length() > 0) {
            Serial2.write((const uint8_t *)value.data(), value.length());
        }
    }
};

void setup() {
    Serial2.begin(BLE_UART_BAUD, SERIAL_8N1, BLE_UART_RX, BLE_UART_TX);

    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setMTU(MAX_MTU);

    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE
    );
    pTxChar->setCallbacks(new MyCharCallbacks());
    pTxChar->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    pAdv->setMinPreferred(0x06);
    pAdv->setMaxPreferred(0x0C);
    BLEDevice::startAdvertising();

    txPacketLen = 0;
    lastFlush = millis();
}

void loop() {
    while (Serial2.available() && txPacketLen < sizeof(txPacket)) {
        txPacket[txPacketLen++] = (uint8_t)Serial2.read();
    }

    uint32_t now = millis();

    if (txPacketLen > 0 &&
        (txPacketLen >= 60 || (now - lastFlush) >= 20)) {
        if (deviceConnected) {
            pTxChar->notify(txPacket, txPacketLen);
        }
        txPacketLen = 0;
        lastFlush = now;
    }

    if (txPacketLen > 0 && (now - lastFlush) > 100) {
        txPacketLen = 0;
        lastFlush = now;
    }

    delay(1);
}
