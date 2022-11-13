#include <Arduino.h>
#include "ble-lock.h"

#define SERVICE_UUID "58127e7d-6468-47b7-b901-56106ba89358"
#define CHARACTERISTIC_UUID "af6e8a1c-af9a-44ea-8251-216636da9a37"

BLEServer* pServer = NULL;
BLEService *pService = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLEAdvertising *pAdvertising = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void ble_lock_setup() {
  log_i("BLE SETUP start");
  BLEDevice::init(BLE_DEVICE_NAME);

  log_i("BLE SETUP -- create server");
  pServer = BLEDevice::createServer();

  log_i("BLE SETUP -- create service");
  pService = pServer->createService(SERVICE_UUID);

  log_i("BLE SETUP -- create callbacks");
  pServer->setCallbacks(new MyServerCallbacks());

  log_i("BLE SETUP -- create characteristic");
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("S");
  pService->start();

  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  log_i("BLE SETUP done");
}

void ble_lock_loop() {
  if (deviceConnected) {
      pCharacteristic->setValue((uint8_t*)&value, 4);
      pCharacteristic->notify();
      value++;
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      log_d("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}
