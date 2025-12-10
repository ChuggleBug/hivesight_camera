#include "main.h"

#include <Arduino.h>
#include <ArduinoJson.h>

#include <algorithm>
#include <vector>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "time.h"

// Network interfaces
WiFiClient netif;
WiFiUDP netifUDP;
PubSubClient mqttClient(netif);
NTPClient timeClient(netifUDP);
HTTPClient http;

// Topics to subscribe to
const char* sensor_topic_prefix = "sensor/";
String sensor_topic = sensor_topic_prefix + String("+");
String mapping_topic;  // = "mapping/" + deviceName
std::vector<String> mapped_sensors;

// Functions
void coordinator_register_device();
void mqtt_broker_sub_cb(char* topic, uint8_t* payload, unsigned int len);

// === Macros ===

#define panic(fmt, ...)                \
  do {                                 \
    Serial.printf(fmt, ##__VA_ARGS__); \
    Serial.println();                  \
    vTaskSuspend(NULL);                \
  } while (0)

// === Static / Local Variables ===

// === Function Declarations ===

void coordinator_register_device();
extern bool load_device_configs(fs::FS& fs);
extern void camera_svc_start();

static void delete_dir_recursive(fs::FS &fs, const char* path);
static void _delete_dir_r(fs::FS &fs, const char* path, fs::File dir);

void IRAM_ATTR mqtt_svc_signal_event();
void mqtt_notif_loop(void* args);

// === Code Begin ===

void setup() {
  Serial.begin(CONFIG_BAUD_RATE);

  if (!LittleFS.begin()) {
    panic("Failed to init Flash FS");
  }

  if (!SD_MMC.begin()) {
    panic("Failed to init SD FS");
  }
  
  if (!load_device_configs(LittleFS)) {
    panic("Failed to load configurations");
  }


  Serial.println();
  Serial.println("Configurations");
  Serial.print("Wifi SSID:        ");
  Serial.println(wifiSSID);
  Serial.print("Wifi password:    ");
  for (char c : wifiPass) {
    Serial.print('*');
  }
  Serial.println();
  Serial.print("Device Name:      ");
  Serial.println(deviceName);
  Serial.print("Broker IP:        ");
  Serial.println(brokerIP);
  Serial.print("Broker Port:      ");
  Serial.println(brokerPort);
  Serial.print("Coordinator IP:   ");
  Serial.println(coordinatorIP);
  Serial.print("Coordinator Port: ");
  Serial.println(coordinatorPort);
  Serial.println();

  Serial.print("Connecting to network over Wi-Fi");
  WiFi.begin(wifiSSID, wifiPass);
  while (!WiFi.isConnected()) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();

  Serial.println("Connected!");

  // Clear from memory
  wifiPass.clear();

  Serial.println("Configuring mqtt...");
  mqttClient.setServer(brokerIP, brokerPort);
  mqttClient.connect(deviceName.c_str());
  Serial.println("Connected to broker!");

  timeClient.begin();

  // Topics to listen to
  mapping_topic = "mapping/" + deviceName;
  Serial.printf("Subscribing to topic %s...", sensor_topic.c_str());
  Serial.println();
  Serial.printf("Subscribing to topic %s...", mapping_topic.c_str());
  Serial.println();

  if (!mqttClient.subscribe(sensor_topic.c_str()) ||
      !mqttClient.subscribe(mapping_topic.c_str())) {
    Serial.println("Failed to set subscribe");
    vTaskSuspend(NULL);
  }
  mqttClient.setCallback(mqtt_broker_sub_cb);

  coordinator_register_device();

  camera_svc_start();
}

void loop() {
  mqttClient.loop();
  timeClient.update();

  if (!mqttClient.connected()) {
    Serial.println("Lost connection to broker");
    if (mqttClient.connect(deviceName.c_str())) {
      Serial.println("Reconnected to broker!");
    } else {
      Serial.println("Failed to reconnect...");
    }
  }

  delay(10);
}

void coordinator_register_device() {
  ArduinoJson::JsonDocument json;
  int resp_code = 0;
  static char buf[256];
  memset(buf, 0, 256);

  json["name"] = deviceName;
  json["type"] = DEVICE_TYPE;

  Serial.println("Registering devivce...");
  while (resp_code != HTTP_CODE_NO_CONTENT) {
    http.begin(coordinatorIP.toString(), coordinatorPort,
               "/api/device/register");
    http.addHeader("Content-Type", "application/json");
    serializeJson(json, buf);
    resp_code = http.PUT(buf);

    Serial.printf("HTTP Response: %d", resp_code);
    Serial.println();
  }
}

void mqtt_broker_sub_cb(char* topic, uint8_t* payload, unsigned int len) {
  // Handle known topics
  Serial.printf("Got topic \"%s\"!", topic);
  Serial.println();

  // Expect data such as { "camName": ["sensor1", "sensor2", ...] }
  if (strcmp(topic, mapping_topic.c_str()) == 0) {
    ArduinoJson::JsonDocument json;
    deserializeJson(json, (char*)payload, len);
    ArduinoJson::JsonArray sensors = json[deviceName];
    Serial.printf("Registering new devices: ");
    for (JsonVariant v : sensors) {
      String sensor = v.as<String>();
      Serial.printf("%s, ", sensor.c_str());
      mapped_sensors.push_back(v.as<String>());
    }
    Serial.printf("\b\b ");
    Serial.println();
  }
  // Matches "sensor/+"
  else if (strncmp(topic, sensor_topic_prefix, strlen(sensor_topic_prefix)) ==
           0) {
    // topic starts with "sensor/"
    String sensorName = String(
        topic + strlen(sensor_topic_prefix));  // get the part after "sensor/"

    if (std::none_of(mapped_sensors.begin(), mapped_sensors.end(),
                     [=](const String& s) { return s == sensorName; })) {
      // Not a sensor to respond to
      return;
    }

    Serial.printf("%s is a mapped sensor!", sensorName.c_str());
    Serial.println();

    // Get timestamp of event for the camera to later send over
    ArduinoJson::JsonDocument json;
    deserializeJson(json, (char*)payload, len);
    if (!json["time"].is<uint32_t>()) {
      Serial.println("Topic received, but message was not properly formatted!");
      return;
    }
    xTaskNotify(CameraServiceEventTask, json["time"].as<uint32_t>(), eSetValueWithOverwrite);

  } else {
    Serial.printf("Uknown topic: %s", topic);
    Serial.println();
  }
}

static void delete_dir_recursive(fs::FS &fs, const char* path) {
  fs::File dir = fs.open(path);
  _delete_dir_r(fs, path, dir);
}

static void _delete_dir_r(fs::FS &fs, const char* path, fs::File dir) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      // No more files
      break;
    }

    if (entry.isDirectory()) {
      Serial.print("Deleting directory: ");
      Serial.println(entry.name());
      _delete_dir_r(fs, entry.name(), entry);  // Recurse into subdirectories
      fs.rmdir(entry.name());                     // Delete the empty directory
    } else {
      Serial.print("Deleting file: ");
      Serial.println(entry.name());
      fs.remove(entry.name());  // Delete the file
    }
    entry.close();
  }
  dir.close();
}