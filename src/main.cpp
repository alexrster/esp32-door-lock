#include "version.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <wifi-mqtt.h>
#include <FastLED.h>
#include <MotionSensor.h>
#include <SwitchRelay.h>
#include <ble-lock.h>

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
  otaStarted = false,
  wifiWasConnected = false;

unsigned int
  messagesReceived = 0;

unsigned long
  minRunTimeMs = MIN_RUN_TIME_MS,
  now_global_start = 0,
  now = 0;

RTC_DATA_ATTR unsigned long 
  lastLedOn = 0,
  lastBatteryVoltageReadMs = 0,
  lastBatteryVoltageUpdateMs = 0,
  lastDoorLockOnMs = 0,
  now_global = 0;

RTC_DATA_ATTR uint16_t
  boot_count = 0,
  batteryLevel = 0;

RTC_DATA_ATTR bool
  ledOn = false;

void light_on(CRGB color) {
  if (ledOn && color == current_color) {
    lastLedOn = now_global;
    return;
  }

  log_i("light ON");
  ledOn = true;
  lastLedOn = now_global;

  digitalWrite(PIN_LED_POWER, LED_POWER_ON);

  current_color = color;
  FastLED.showColor(CRGB::Yellow);

  #ifdef DEBUG
  digitalWrite(LED, HIGH);
  #endif

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", "1");
}

void light_on() {
  light_on(CRGB::Blue);
}

void light_off() {
  log_i("light OFF");

  ledOn = false;
  FastLED.clear(true);

  digitalWrite(PIN_LED_POWER, LED_POWER_OFF);

  #ifdef DEBUG
  digitalWrite(LED, LOW);
  #endif

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", "0");
}

void light_loop() {
  if (ledOn && now_global - lastLedOn > LED_TIMEOUT_MS) {
    light_off();
  }
}

void door_lock_open() {
  log_i("DOOR LOCK open");
  light_on();

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", "U");

  digitalWrite(PIN_DOOR_LOCK, HIGH);
  delay(500);
  digitalWrite(PIN_DOOR_LOCK, LOW);

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", "S");
  // publishDoorLock0 = true;
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
  if (now_global - lastBatteryVoltageReadMs > BATTERY_VOLTAGE_READ_MS) {
    if (lastBatteryVoltageReadMs > 0) batteryLevel = (batteryLevel + analogRead(PIN_BATTERY_LEVEL)) / 2;
    else batteryLevel = analogRead(PIN_BATTERY_LEVEL);
    lastBatteryVoltageReadMs = now_global;
  }

  if (now_global - lastBatteryVoltageUpdateMs > BATTERY_VOLTAGE_UPDATE_MS) {
    lastBatteryVoltageUpdateMs = now_global;

    publishBatteryLevel = true;
  }
}

void on_pubsub_message(char* topic, uint8_t* data, unsigned int length) {
  log_d("MQTT message: topic=%s; length=%u; data=%s", topic, length, data);
  messagesReceived++;

  if (pubsub_topic_lock.equals(topic) && parse_bool_meesage(data, length)) {
    if (lastDoorLockOnMs == 0 || now_global - lastDoorLockOnMs > DOOR_LOCK_MIN_DELAY_MS) {
      lastDoorLockOnMs = now_global;

      // pubSubClient.publish(pubsub_topic_lock.c_str(), "0");
      door_lock_open();
    }
  }
  else if (pubsub_topic_light.equals(topic) && parse_bool_meesage(data, length)) {
    // pubSubClient.publish(pubsub_topic_light.c_str(), "0");
    light_on();
  }
  // else if (pubsub_topic_restart.equals(topic) && parse_bool_meesage(data, length)) {
  //   ESP.restart();
  // }
}

void on_ota_start() {
  otaStarted = true;
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
  pinMode(PIN_LED_POWER, OUTPUT);

  #ifdef DEBUG
  pinMode(LED, OUTPUT);
  digitalWrite(LED, ledOn);
  #endif

  // ble_lock_setup();

  mot.onChanged(on_motion_state);

  FastLED.addLeds<WS2812B, PIN_LED, RGB>(leds, LED_COUNT);
  FastLED.setBrightness(255);

  wifi_setup(boot_count == 1);
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
  // ble_lock_loop();

  if (wifi_loop(now)) {
    if (!wifiWasConnected) canSleep = false;

    wifiWasConnected = true;

    ArduinoOTA.handle();
    if (otaStarted) return;

    if (mqtt_loop(now)) {
      if (publishDoorLock0) publishDoorLock0 = !pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", "S");
      if (publishMotion1) publishMotion1 = !pubSubClient.publish(MQTT_TOPIC_PREFIX "/motion", "1");
      if (publishBatteryLevel) publishBatteryLevel = 
        !pubSubClient.publish(MQTT_TOPIC_PREFIX "/battery/raw", String(batteryLevel).c_str()) ||
        !pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", ledOn ? "1" : "0") ||
        !pubSubClient.publish(MQTT_TOPIC_PREFIX "/motion", mot.getState() == Detected ? "1" : "0");
    }
  }

  if (canSleep && now > minRunTimeMs) {
    if ((!wifiWasConnected || publishDoorLock0 || publishMotion1 || publishBatteryLevel) && minRunTimeMs < MAX_RUN_TIME_WAIT_WIFI_MS) {
      minRunTimeMs += 100;
    }
    else {
      if (wifiWasConnected) {
        if (pubSubClient.connected()) {
          log_d("MQTT flush");
          for (uint8_t i=0; i<20 && !pubSubClient.loop(); i++) { delay(5); }
          pubSubClient.flush();
        }

        // log_d("WIFI disconnect");
        // for (uint8_t i=0; i<20 && !WiFi.disconnect(false); i++) { delay(5); }
      }

      auto m = millis();
      now_global = now_global_start + m;

#ifdef DEBUG
      log_d("ENTER DEEP SLEEP: wifi=%s; battery=%u; messages=%u; led=%s; global_runtime=%u; runtime=%u", 
        wifiWasConnected ? "true" : "false", 
        batteryLevel, 
        messagesReceived, 
        ledOn ? "on" : "off", 
        now_global, 
        m);

      Serial.flush(true);
      delay(10);
#endif

      esp_sleep_enable_ext0_wakeup(PIN_MOTION_SENSOR, HIGH);
      esp_deep_sleep(DEEP_SLEEP_TIME_US);
    }
  }

  delay(50);
}
