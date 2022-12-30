#ifndef __WIFI_MQTT
#define __WIFI_MQTT

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME                 "esp32-door-lock"
#endif

#define WIFI_RECONNECT_MILLIS         800
#define WIFI_WATCHDOG_MILLIS          60000

#define MQTT_SERVER_NAME              "10.9.9.96"
#define MQTT_SERVER_PORT              1883
#define MQTT_USERNAME                 NULL
#define MQTT_PASSWORD                 NULL
#define MQTT_RECONNECT_MILLIS         200

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID                WIFI_HOSTNAME
#endif

#define MQTT_TOPIC_PREFIX             "dev/" MQTT_CLIENT_ID
#define MQTT_STATUS_TOPIC             MQTT_TOPIC_PREFIX "/status"
#define MQTT_VERSION_TOPIC            MQTT_TOPIC_PREFIX "/version"
#define MQTT_STATUS_ONLINE_MSG        "online"
#define MQTT_STATUS_OFFLINE_MSG       "offline"

extern WiFiClient wifiClient;
extern PubSubClient pubSubClient;

void wifi_setup(bool cleanSetup = true);
bool wifi_loop(unsigned long now);
bool mqtt_loop(unsigned long now);
boolean parse_bool_meesage(uint8_t* payload, unsigned int length, boolean defaultValue = false);

#endif