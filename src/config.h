#ifndef __CONFIG_H
#define __CONFIG_H

#define PIN_DOOR_SENSOR           2
#define PIN_DOOR_LOCK             17
#define PIN_DOOR_LOCK_SENSOR      27
#define PIN_BATTERY_LEVEL         32
#define PIN_EXTRA_1               25
#define PIN_EXTRA_2               26

#define PIN_LED                   PIN_EXTRA_1
#define LED_COUNT                 9

#define LED_COLOR_1               CRGB::FairyLight
#define LED_COLOR_1_TIMEOUT_MS    30000
#define LED_COLOR_2               CRGB::Yellow
#define LED_COLOR_2_TIMEOUT_MS    37000
#define LED_COLOR_3               CRGB::DarkOrange
#define LED_TIMEOUT_MS            40000

#define LED_BLINKING_TIMEOUT_MS   660

#define PIN_MOTION_SENSOR         PIN_EXTRA_2

#define BATTERY_VOLTAGE_UPDATE_MS 30000
#define BATTERY_VOLTAGE_READ_MS   5000

#endif
