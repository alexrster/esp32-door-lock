#ifndef __CONFIG_H
#define __CONFIG_H

#define PIN_DOOR_BELL             GPIO_NUM_2
#define PIN_DOOR_LOCK             GPIO_NUM_17
#define PIN_DOOR_LOCK_SENSOR      GPIO_NUM_27
#define PIN_BATTERY_LEVEL         GPIO_NUM_32
#define PIN_EXTRA_1               GPIO_NUM_25
#define PIN_EXTRA_2               GPIO_NUM_26

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

#define DEEP_SLEEP_TIME_US        3 * 1000000
#define MIN_RUN_TIME_MS           200
#define MAX_RUN_TIME_WAIT_WIFI_MS 10 * MIN_RUN_TIME_MS

#define DOOR_LOCK_IMPULSE_MS      500
#define DOOR_LOCK_MIN_DELAY_MS    30000

#endif
