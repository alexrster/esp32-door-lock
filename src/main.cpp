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
  ledOn = false;

uint16_t
  batteryLevel = 0;

unsigned long
  lastLedOn = 0,
  lastBatteryVoltageReadMs = 0,
  lastBatteryVoltageUpdateMs = 0,
  now = 0;

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
}

void on_motion_state(MotionState_t state) {
  if (state == MotionState_t::Detected) {
    log_i("Motion detected!");
    light_on();
  }

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/motion", state == Detected ? "1" : "0");
}

void battery_voltage_loop() {
  if (now - lastBatteryVoltageReadMs > BATTERY_VOLTAGE_READ_MS) {
    lastBatteryVoltageReadMs = now;
    batteryLevel = (batteryLevel + analogRead(PIN_BATTERY_LEVEL)) / 2;
  }

  if (now - lastBatteryVoltageUpdateMs > BATTERY_VOLTAGE_UPDATE_MS) {
    lastBatteryVoltageUpdateMs = now;

    pubSubClient.publish(MQTT_TOPIC_PREFIX "/battery/raw", String(batteryLevel).c_str());
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

void setup() {
  log_i("SETUP start");
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DOOR_LOCK, OUTPUT);
  pinMode(PIN_BATTERY_LEVEL, INPUT);

  ble_lock_setup();

  mot.onChanged(on_motion_state);

  // FastLED.addLeds<WS2812B, PIN_LED, RGB>(leds, LED_COUNT);
  FastLED.addLeds<WS2812B, PIN_LED, RGB>(leds, LED_COUNT);
  // FastLED.clearData(true);
  FastLED.setBrightness(255);

  wifi_setup();
  pubSubClient.setCallback(on_pubsub_message);
  // ArduinoOTA.begin();

  // light_on();
  log_i("SETUP done");
}

void loop() {
  now = millis();

  mot.loop();
  light_loop();
  battery_voltage_loop();
  ble_lock_loop();

  if (wifi_loop(now)) {
    if (publishDoorLock0) {
      publishDoorLock0 = !pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", "S");
    }

    // ArduinoOTA.handle();
  }

  delay(200);
}
