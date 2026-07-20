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
 * v2.0: 增大缓冲区 + 对齐 MicroPython 版超时参数,
 *       适配 MCU 二进制遥测帧（54 字节/帧, 500ms 间隔）
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

/* 增大缓冲区以容纳完整二进制帧 (54 字节 + 余量) */
#define TX_PACKET_SIZE  128
/* 对齐 MicroPython 版: 200ms 冲刷 / 500ms 半包丢弃 */
#define FLUSH_TIMEOUT_MS      20
#define HALF_PACKET_DISCARD_MS 500

BLECharacteristic *pTxChar;
bool deviceConnected = false;
uint8_t txPacket[TX_PACKET_SIZE];
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
    /* 从 UART 读取字节到发送缓冲区 */
    while (Serial2.available() && txPacketLen < TX_PACKET_SIZE) {
        txPacket[txPacketLen++] = (uint8_t)Serial2.read();
    }

    uint32_t now = millis();

    /* 缓冲区满 或 闲置 200ms → 转发到 BLE */
    if (txPacketLen > 0 &&
        (txPacketLen >= TX_PACKET_SIZE || (now - lastFlush) >= FLUSH_TIMEOUT_MS)) {
        if (deviceConnected) {
            pTxChar->notify(txPacket, txPacketLen);
        }
        txPacketLen = 0;
        lastFlush = now;
    }

    /* 500ms 无新数据 → 丢弃半包（对齐 MicroPython 版） */
    if (txPacketLen > 0 && (now - lastFlush) > HALF_PACKET_DISCARD_MS) {
        txPacketLen = 0;
        lastFlush = now;
    }

    delay(1);
}
