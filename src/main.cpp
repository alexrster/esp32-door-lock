#include "version.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <wifi-mqtt.h>
#include <FastLED.h>
#include <MotionSensor.h>
#include <SwitchRelay.h>
#include <PushButton.h>
// #include <ble-lock.h>

const String pubsub_topic_lock = String(MQTT_TOPIC_PREFIX "/lock/set");
const String pubsub_topic_light = String(MQTT_TOPIC_PREFIX "/light/set");
const String pubsub_topic_restart = String(MQTT_TOPIC_PREFIX "/restart");

MotionSensor mot(PIN_MOTION_SENSOR);
SwitchRelayPin door_lock(PIN_DOOR_LOCK, HIGH);
PushButton door_lock_sensor(PIN_DOOR_LOCK_SENSOR, INPUT_PULLUP, LOW);

CRGB 
  leds[LED_COUNT],
  current_color = CRGB::Black,
  blinking_target_color = CRGB::Red,
  blinking_current_color = CRGB::Black;

bool
  otaStarted = false,
  wifiWasConnected = false,
  ledBlinking = false,
  connectionRestored = true;

unsigned int
  batteryLevel = 0,
  messagesReceived = 0;

unsigned long
  now = 0;

RTC_DATA_ATTR unsigned long 
  lastLedOn = 0,
  lastDoorLockActivatedMs = 0,
  lastBatteryVoltageReadMs = 0,
  lastBatteryVoltageUpdateMs = 0,
  lastDoorLockOnMs = 0,
  lastLedBlinkingMs = 0,
  lastMotionDetectedMs = 0,
  now_global = 0;

RTC_DATA_ATTR bool
  ledOn = false;

void light_set_color(CRGB color) {
  if (!ledOn || color == current_color) {
    return;
  }

  current_color = color;
  FastLED.showColor(current_color);
}

void light_on(CRGB color) {
  if (ledOn && color == current_color) {
    lastLedOn = now_global;
    lastLedOn = now_global;
    return;
  }

  log_i("light ON");
  ledOn = true;
  lastLedOn = now_global;

  current_color = color;
  FastLED.showColor(CRGB::Yellow);

  #ifdef DEBUG
  digitalWrite(LED, HIGH);
  #endif

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", "1");
}

void light_on() {
  light_on(LED_COLOR_1);
}

void light_off() {
  log_i("light OFF");

  ledOn = false;
  FastLED.showColor(CRGB::Black);
  FastLED.clear(true);

  digitalWrite(PIN_LED_POWER, LED_POWER_OFF);

  #ifdef DEBUG
  digitalWrite(LED, LOW);
  #endif

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", "0");
}

void light_blink_start(CRGB color) {
  log_i("light BLINK");

  blinking_current_color = CRGB::Black;
  blinking_target_color = color;

  ledBlinking = true;
  ledOn = false;
}

void light_blink_start() {
  light_blink_start(CRGB::Red);
}

void light_blink_stop() {
  ledBlinking = false;
  ledOn = now - lastLedOn < LED_TIMEOUT_MS;
}

__inline void light_blink_loop() {
  if (!ledBlinking) return;

  if (now - lastLedBlinkingMs > LED_BLINKING_TIMEOUT_MS) {
    lastLedBlinkingMs = now;

    if (blinking_current_color != blinking_target_color) {
      blinking_current_color = blinking_target_color;
      FastLED.showColor(blinking_current_color);
    } else {
      blinking_current_color = CRGB::Black;
      FastLED.clear(true);
    }
  }
}

__inline void light_loop() {
  light_blink_loop();
  if (!ledOn) return;

  if (now - lastLedOn > LED_TIMEOUT_MS) light_off();
  else if (now - lastLedOn > LED_COLOR_2_TIMEOUT_MS) light_set_color(LED_COLOR_3);
  else if (now - lastLedOn > LED_COLOR_1_TIMEOUT_MS) light_set_color(LED_COLOR_2);
}

void door_lock_open() {
  log_i("DOOR LOCK open");
  door_lock.setOn();

  lastDoorLockActivatedMs = millis();
  light_on();
}

void on_door_lock_state(PushButton *btn) {
  auto state = btn->getState();
  if (state == ButtonState::On) {
    door_lock.setOff();
    lastDoorLockActivatedMs = 0;

    light_blink_start();
  } else  {
    light_blink_stop();

    if (now - lastMotionDetectedMs > 5000) {
      light_on();
    }
  }
  
  pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", state == ButtonState::On ? "U" : "S", true);
}

void on_motion_state(MotionState state) {
  if (state == MotionState::Detected) {
    log_i("Motion detected!");
    lastMotionDetectedMs = now;
    light_on();

    publishMotion1 =  true;
    canSleep = false;
  }

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/motion", state == MotionState::Detected ? "1" : "0");
}

void battery_voltage_loop() {
  if (now_global - lastBatteryVoltageReadMs > BATTERY_VOLTAGE_READ_MS) {
    if (lastBatteryVoltageReadMs > 0) batteryLevel = (batteryLevel + analogRead(PIN_BATTERY_LEVEL)) / 2;
    else batteryLevel = analogRead(PIN_BATTERY_LEVEL);
    lastBatteryVoltageReadMs = now_global;
  if (now_global - lastBatteryVoltageReadMs > BATTERY_VOLTAGE_READ_MS) {
    if (lastBatteryVoltageReadMs > 0) batteryLevel = (batteryLevel + analogRead(PIN_BATTERY_LEVEL)) / 2;
    else batteryLevel = analogRead(PIN_BATTERY_LEVEL);
    lastBatteryVoltageReadMs = now_global;
  }

  if (now_global - lastBatteryVoltageUpdateMs > BATTERY_VOLTAGE_UPDATE_MS) {
    lastBatteryVoltageUpdateMs = now_global;

    pubSubClient.publish(MQTT_TOPIC_PREFIX "/battery/raw", String(batteryLevel).c_str());
  }
}

void door_lock_loop() {
  if (lastDoorLockActivatedMs > 0 && now - lastDoorLockActivatedMs > 500) {
    lastDoorLockActivatedMs = 0;

    door_lock.setOff();
  }
}

void on_pubsub_message(char* topic, uint8_t* data, unsigned int length) {
  // log_d("MQTT message: topic=%s; length=%u; data=%s", topic, length, data);
  if (pubsub_topic_lock.equals(topic)) {
    if (parse_bool_meesage(data, length)) door_lock_open();
  }
  else if (pubsub_topic_light.equals(topic) && parse_bool_meesage(data, length)) {
    // pubSubClient.publish(pubsub_topic_light.c_str(), "0");
    // pubSubClient.publish(pubsub_topic_light.c_str(), "0");
    light_on();
  }
  // else if (pubsub_topic_restart.equals(topic) && parse_bool_meesage(data, length)) {
  //   ESP.restart();
  // }
}

void on_ota_start() {
  otaStarted = true;
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
  print_wakeup_reason();

  log_i("SETUP START");
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DOOR_LOCK, OUTPUT);
  pinMode(PIN_BATTERY_LEVEL, INPUT);

  // ble_lock_setup();

  mot.onChanged(on_motion_state);
  door_lock_sensor.onChanged(on_door_lock_state);

  FastLED.addLeds<WS2812B, PIN_LED, RGB>(leds, LED_COUNT);
  FastLED.setBrightness(255);

  wifi_setup(true);
  pubSubClient.setCallback(on_pubsub_message);

  ArduinoOTA.onStart(on_ota_start);
  ArduinoOTA.onError(on_ota_error);
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.setMdnsEnabled(false);

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
  door_lock_sensor.loop(now);

  now = millis();
  light_loop();
  battery_voltage_loop();
  // ble_lock_loop();
  // ble_lock_loop();

  if (wifi_loop(now)) {
    wifiWasConnected = true;

    ArduinoOTA.handle();

    if (connectionRestored) {
      connectionRestored = false;

      pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", door_lock_sensor.getState() == ButtonState::On ? "U" : "S", true);
    }
  } else {
    connectionRestored = true;
  }

  delay(10);
}
