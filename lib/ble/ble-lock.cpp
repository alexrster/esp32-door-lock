#include <Arduino.h>
#include "ble-lock.h"

uint8_t scanTime = 5; //In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};

void ble_lock_setup() {
  log_i("BLE SETUP start");

  BLEDevice::init(BLE_DEVICE_NAME);
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

  log_i("BLE SETUP done");
}

void ble_lock_loop(unsigned long now) {
  // pBLEScan->start();
  // BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  // pBLEScan->clearResults();
}
