#include "version.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <wifi-mqtt.h>
#include <FastLED.h>
#include <MotionSensor.h>
#include <SwitchRelay.h>

const String pubsub_topic_lock = String(MQTT_TOPIC_PREFIX "/lock/set");
const String pubsub_topic_light = String(MQTT_TOPIC_PREFIX "/light/set");
const String pubsub_topic_restart = String(MQTT_TOPIC_PREFIX "/restart");

MotionSensor mot(PIN_MOTION_SENSOR);
SwitchRelayPin door_lock(PIN_DOOR_LOCK, HIGH);

CRGB 
  leds[LED_COUNT],
  current_color = CRGB::Black;

bool
  publishDoorLock0 = false,
  publishMotion1 = false,
  publishBatteryLevel = false,
  canSleep = true,
  wifiWasConnected = false,
  ledOn = false;

unsigned long
  lastLedOn = 0,
  minRunTimeMs = MIN_RUN_TIME_MS,
  now_global_start = 0,
  now = 0;

RTC_DATA_ATTR unsigned long 
  lastBatteryVoltageReadMs = 0,
  lastBatteryVoltageUpdateMs = 0,
  now_global = 0;

RTC_DATA_ATTR uint16_t
  boot_count = 0,
  batteryLevel = 0;

void light_on(CRGB color) {
  if (ledOn && color == current_color) {
    lastLedOn = now;
    return;
  }

  log_i("light ON");

  ledOn = true;
  lastLedOn = now;
  current_color = color;
  FastLED.showColor(CRGB::Yellow);

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", "1");
}

void light_on() {
  light_on(CRGB::Blue);
}

void light_off() {
  log_i("light OFF");

  ledOn = false;
  FastLED.clear(true);

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", "0");
}

void light_loop() {
  if (ledOn && now - lastLedOn > LED_TIMEOUT_MS) {
    light_off();
  }
}

void door_lock_open() {
  log_i("DOOR LOCK open");
  light_on();

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", "U");
  digitalWrite(PIN_DOOR_LOCK, HIGH);
  // door_lock.setOn();
  delay(500);
  digitalWrite(PIN_DOOR_LOCK, LOW);
  // door_lock.setOff();

  publishDoorLock0 = true;
  canSleep = false;
}

void on_motion_state(MotionState_t state) {
  if (state == MotionState_t::Detected) {
    log_i("Motion detected!");
    light_on();

    publishMotion1 =  true;
    canSleep = false;
  }
}

void battery_voltage_loop() {
  if (lastBatteryVoltageReadMs == 0) {
    lastBatteryVoltageReadMs = now_global;
    batteryLevel = analogRead(PIN_BATTERY_LEVEL);
  }
  else if (now_global - lastBatteryVoltageReadMs > BATTERY_VOLTAGE_READ_MS) {
    lastBatteryVoltageReadMs = now_global;
    batteryLevel = (batteryLevel + analogRead(PIN_BATTERY_LEVEL)) / 2;
  }

  if (now_global - lastBatteryVoltageUpdateMs > BATTERY_VOLTAGE_UPDATE_MS) {
    lastBatteryVoltageUpdateMs = now_global;

    publishBatteryLevel = true;
  }
}

void on_pubsub_message(char* topic, uint8_t* data, unsigned int length) {
  log_d("MQTT message: topic=%s; length=%u; data=%s", topic, length, data);
  if (pubsub_topic_lock.equals(topic) && parse_bool_meesage(data, length)) {
    door_lock_open();
  }
  else if (pubsub_topic_light.equals(topic) && parse_bool_meesage(data, length)) {
    light_on();
  }
  else if (pubsub_topic_restart.equals(topic) && parse_bool_meesage(data, length)) {
    ESP.restart();
  }
}

void on_ota_start() {
  canSleep = false;
}

void on_ota_error(ota_error_t err) {
  log_w("OTA ERROR: code=%u", err);
  esp_restart();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : log_i("- wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : log_i("- wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : log_i("- wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : log_i("- wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : log_i("- wakeup caused by ULP program"); break;
    default : log_i("- wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void setup() {
  log_i("BOOT #%u", ++boot_count);
  print_wakeup_reason();

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    now_global += DEEP_SLEEP_TIME_US / 1000;
  }

  now_global_start = now_global;

  log_i("SETUP START");
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DOOR_LOCK, OUTPUT);
  pinMode(PIN_BATTERY_LEVEL, INPUT);

  mot.onChanged(on_motion_state);

  FastLED.addLeds<WS2812B, PIN_LED, RGB>(leds, LED_COUNT);
  FastLED.setBrightness(255);

  wifi_setup();
  pubSubClient.setCallback(on_pubsub_message);

  ArduinoOTA.onStart(on_ota_start);
  ArduinoOTA.onError(on_ota_error);
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.setMdnsEnabled(false);
  ArduinoOTA.begin();

  log_i("SETUP done");
}

void loop() {
  now = millis();
  now_global = now_global_start + now;
  canSleep = true;

  mot.loop();
  light_loop();
  battery_voltage_loop();

  if (wifi_loop(now)) {
    wifiWasConnected = true;

    if (publishDoorLock0) publishDoorLock0 = !pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", "S");
    if (publishMotion1) publishMotion1 = !pubSubClient.publish(MQTT_TOPIC_PREFIX "/motion", "1");
    if (publishBatteryLevel) publishBatteryLevel = !pubSubClient.publish(MQTT_TOPIC_PREFIX "/battery/raw", String(batteryLevel).c_str());

    ArduinoOTA.handle();
  }

  if (canSleep && now > minRunTimeMs) {
    if ((!wifiWasConnected || publishDoorLock0 || publishMotion1 || publishBatteryLevel) && minRunTimeMs < MAX_RUN_TIME_WAIT_WIFI_MS) {
      minRunTimeMs += 100;
    }
    else {
      if (wifiWasConnected) {
        WiFi.disconnect(false);
      }

      log_i("ENTER DEEP SLEEP: wifi=%s; battery=%u; global_runtime=%u; runtime=%u", wifiWasConnected ? "true" : "false", batteryLevel, now_global, millis());
      esp_sleep_enable_ext0_wakeup(PIN_MOTION_SENSOR, HIGH);
      esp_deep_sleep(DEEP_SLEEP_TIME_US);
    }
  }

  delay(50);
}
