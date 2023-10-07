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
const String pubsub_topic_alarm = String(MQTT_TOPIC_PREFIX "/alarm/set");
const String pubsub_topic_restart = String(MQTT_TOPIC_PREFIX "/restart");

MotionSensor mot(PIN_MOTION_SENSOR);
SwitchRelayPin door_lock(PIN_DOOR_LOCK);
PushButton door_lock_sensor(PIN_DOOR_LOCK_SENSOR, INPUT_PULLUP, LOW);
PushButton door_sensor(PIN_DOOR_SENSOR, INPUT_PULLUP, LOW);

const CRGB black = CRGB::Black;

CRGB 
  leds[LED_COUNT],
  current_color = black,
  target_color = black,
  blinking_target_color = CRGB::Red,
  blinking_current_color = black;

bool
  ledOn = false,
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
  lastMotionDetectedMs = 0;

void light_set_color(CRGB color) {
  target_color = color;
  // FastLED.showColor(target_color);
}

void light_on(CRGB color) {
  if (ledOn && color == target_color) {
    lastLedOn = now;
    return;
  }

  log_i("light ON");
  lastLedOn = now;

  if (!ledBlinking) {
    lastLedOn = now;
    light_set_color(color);
  }

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", "1");
}

void light_on() {
  light_on(LED_COLOR_1);
}

void light_off() {
  log_i("light OFF");

  ledOn = false;
  // FastLED.clear(true);
  target_color = black;

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/light", "0");
}

void light_blink_start(CRGB color) {
  log_i("light BLINK start");

  blinking_current_color = black;
  blinking_target_color = color;

  ledBlinking = true;
}

void light_blink_start() {
  light_blink_start(CRGB::Yellow);
}

void light_blink_stop() {
  ledBlinking = false;
  ledOn = now - lastLedOn < LED_TIMEOUT_MS;

  if (ledOn) {
    light_set_color(target_color);
  } else {
    light_off();
  }
}

__inline void light_blink_loop() {
  if (now - lastLedBlinkingMs > LED_BLINKING_TIMEOUT_MS) {
    lastLedBlinkingMs = now;

    auto tmp = blinking_current_color;
    blinking_current_color = blinking_target_color;
    blinking_target_color = tmp;

    target_color = blinking_current_color;
    // FastLED.showColor(blinking_current_color);
  }
}

__inline void light_loop() {
  if (otaStarted) return;
  else if (ledBlinking) {
    light_blink_loop();
  }
  else if (ledOn) {
    if (now - lastLedOn > LED_TIMEOUT_MS) light_off();
    else if (now - lastLedOn > LED_COLOR_2_TIMEOUT_MS) light_set_color(LED_COLOR_3);
    else if (now - lastLedOn > LED_COLOR_1_TIMEOUT_MS) light_set_color(LED_COLOR_2);
  }

  EVERY_N_MILLIS(75) {
    if (current_color != target_color) {
      current_color = nblend(current_color, target_color, 64);
      FastLED.showColor(current_color);
    }
    else {
      if (!ledOn && !ledBlinking && current_color == black) {
        FastLED.clear(true);
      }
    }
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

void on_door_state(PushButton *btn) {
  auto state = btn->getState();
  pubSubClient.publish(MQTT_TOPIC_PREFIX "/door", state == ButtonState::On ? "C" : "O", true);
}

void on_motion_state(MotionSensor *sensor, MotionState state) {
  if (state == MotionState::Detected) {
    log_i("Motion detected!");
    lastMotionDetectedMs = now;
    light_on();
  }

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/motion", state == MotionState::Detected ? "1" : "0");
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
  else if (pubsub_topic_light.equals(topic)) {
    if (parse_bool_meesage(data, length)) light_on();
    else light_off();
  }
  else if (pubsub_topic_alarm.equals(topic)) {
    if (parse_bool_meesage(data, length)) light_blink_start(CRGB::Red);
    else light_blink_stop();
  }
  else if (pubsub_topic_restart.equals(topic) && parse_bool_meesage(data, length)) {
    ESP.restart();
  }
}

void on_ota_start() {
  door_lock.setOff();
  otaStarted = true;
  FastLED.clear(true);
  FastLED.showColor(black);
}

void on_ota_error(ota_error_t err) {
  log_w("OTA ERROR: code=%u", err);
  esp_restart();
}

void on_ota_progress(unsigned int progress, unsigned int total) {
  uint16_t ledCount = progress * LED_COUNT / total;
  if (ledCount >= LED_COUNT) ledCount = LED_COUNT - 1;
  FastLED.leds()[ledCount] = CRGB::SkyBlue;
  FastLED.show();
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
  door_sensor.onChanged(on_door_state);

  FastLED.addLeds<WS2812B, PIN_LED, RGB>(leds, LED_COUNT);
  FastLED.setBrightness(255);
  FastLED.clear(true);
  FastLED.showColor(CRGB::SkyBlue);

  wifi_setup(true);
  pubSubClient.setCallback(on_pubsub_message);

  ArduinoOTA.onStart(on_ota_start);
  ArduinoOTA.onError(on_ota_error);
  ArduinoOTA.onProgress(on_ota_progress);
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.setMdnsEnabled(false);
  ArduinoOTA.begin();

  FastLED.showColor(black);
  delay(333);
  FastLED.showColor(CRGB::SkyBlue);
  delay(333);
  FastLED.showColor(black);

  log_i("SETUP done");
}

void loop() {
  now = millis();

  mot.loop(now);
  door_lock_sensor.loop(now);
  door_sensor.loop(now);

  now = millis();
  light_loop();
  // battery_voltage_loop();
  door_lock_loop();
  // ble_lock_loop();

  if (wifi_loop(now)) {
    wifiWasConnected = true;

    ArduinoOTA.handle();

    if (mqtt_loop(now)) {
      if (connectionRestored) {
        connectionRestored = false;

        pubSubClient.publish(MQTT_TOPIC_PREFIX "/lock", door_lock_sensor.getState() == ButtonState::On ? "U" : "S", true);
      }
    }
  } else {
    connectionRestored = true;
  }

  door_lock_loop();
  delay(10);
}
