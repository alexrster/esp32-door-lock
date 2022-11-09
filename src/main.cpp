#include "version.h"
#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <FastLED.h>

CRGB leds[LED_COUNT];

void setup() {
  pinMode(PIN_DOOR_LOCK, OUTPUT);

  FastLED.addLeds<WS2813, PIN_LED, RGB>(leds, LED_COUNT); 
  FastLED.setBrightness(255);
}

void loop() {
	leds[0] = CRGB::Cyan; 
  FastLED.show(); 
  delay(30);
	
  // leds[0] = CRGB::Black; 
  // FastLED.show(); 
  // delay(30);
}
