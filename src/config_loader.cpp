#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>

#include "main.h"

// === Macros ===

#define JSON_MAX_SIZE 1024

// === Externally Defined Variables ===

String wifiSSID;
String wifiPass;

String deviceName;
IPAddress brokerIP;
uint16_t brokerPort;
IPAddress coordinatorIP;
uint16_t coordinatorPort;

// === Function Declarations ===

bool load_device_configs(fs::FS &fs);

bool load_device_configs(fs::FS &fs) {
  ArduinoJson::JsonDocument json;
  static char buf[JSON_MAX_SIZE];

  fs::File file = fs.open("/config.json");
  if (!file) {
    return false;
  }

  memset(buf, 0, JSON_MAX_SIZE);
  file.readBytes(buf, JSON_MAX_SIZE - 1);
  if (deserializeJson(json, buf) != DeserializationError::Ok) {
    return false;
  }

  // Error checking
  if (!json["WiFiSSID"].is<const char *>() ||
      !json["WiFiPassword"].is<const char *>() ||
      !json["DeviceName"].is<const char *>() ||
      !json["MQTT"]["IP"].is<const char *>() ||
      !json["MQTT"]["Port"].is<uint16_t>() ||
      !json["HTTP"]["IP"].is<const char *>() ||
      !json["HTTP"]["Port"].is<uint16_t>()) {
    return false;
  }

  wifiSSID = json["WiFiSSID"].as<String>();
  wifiPass = json["WiFiPassword"].as<String>();

  deviceName = json["DeviceName"].as<String>();

  brokerIP.fromString((json["MQTT"]["IP"].as<const char *>()));
  brokerPort = json["MQTT"]["Port"].as<uint16_t>();

  coordinatorIP.fromString((json["HTTP"]["IP"].as<const char *>()));
  coordinatorPort = json["HTTP"]["Port"].as<uint16_t>();

  return true;
}