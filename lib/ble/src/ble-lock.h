#ifndef __BLE_LOCK_H
#define __BLE_LOCK_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEBeacon.h>

#define BLE_DEVICE_NAME "esp32-door-lock"

void ble_lock_setup();
void ble_lock_loop();

#endif