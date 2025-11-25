#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <vector>
#include <algorithm>

#include "time.h"

#include "freertos/FreeRTOS.h"
#include "esp_attr.h"

#include "main.h"
#include "wifi_man.h"

// main.h extern defined variables
const uint16_t brokerPort = BROKER_PORT;
const uint16_t httpPort = COORDINATOR_HTTP_PORT;
Preferences prefs;

// Application specific objects
// (exclusive to this file)
WiFiClient netif;
WiFiUDP netifUDP;
PubSubClient mqttClient(netif);
NTPClient timeClient(netifUDP);
HTTPClient http;

bool wifi_man_complete = false;

// Topics to subscribe to
const char *sensor_topic_prefix = "sensor/";
String sensor_topic = sensor_topic_prefix + String("+");
String mapping_topic; // = "mapping/" + deviceName
std::vector<String> mapped_sensors;

// Functions
void coordinator_register_device();
void mqtt_broker_sub_cb(char* topic, uint8_t* payload, unsigned int len);

// Interrupts
void IRAM_ATTR mqtt_svc_signal_event();
void IRAM_ATTR reset_wifi_man_configs();

void setup() {
  Serial.begin(BAUD_RATE);
  prefs.begin("app");

  // Enable wifi manager reset pin
  pinMode(CONFIG_WIFI_RST_PIN_NO, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(CONFIG_WIFI_RST_PIN_NO),
                  reset_wifi_man_configs, RISING);

  wifi_man_svc_start();
  wifi_man_complete = true;

  Serial.println("Configuring mqtt...");
  mqttClient.setServer(coordinatorIP, brokerPort);
  mqttClient.connect(deviceName.c_str());
  Serial.println("Connected to broker!");

  // Topics to listen to
  mapping_topic = "mapping/" + deviceName;
  Serial.printf("Subscribing to topic %s...", sensor_topic.c_str());
  Serial.println();
  Serial.printf("Subscribing to topic %s...", mapping_topic.c_str());
  Serial.println();

  if (!mqttClient.subscribe(sensor_topic.c_str()) || !mqttClient.subscribe(mapping_topic.c_str())){
    Serial.println("Failed to set subscribe");
    vTaskSuspend( NULL );
  }
  mqttClient.setCallback(mqtt_broker_sub_cb);

  coordinator_register_device();
}

void loop() {
  mqttClient.loop(); 
  timeClient.update();

  if (!mqttClient.connected()) {
    mqttClient.connect(deviceName.c_str());
    Serial.println("Reconnected to broker!");
  }

  delay(10);
}

void mqtt_notif_loop(void* args) {
  String topic = "sensor/" + deviceName;
  ArduinoJson::JsonDocument json;
  static char buf[512];
  memset(buf, 0, 512);
  while (1) {
    // This task will block until something else notifies it
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

    // Report the time of event and send a json
    json["time"] = timeClient.getEpochTime();
    serializeJson(json, buf);

    Serial.println("Sending topic " + topic);
    mqttClient.publish(topic.c_str(), buf);
  }
}

void IRAM_ATTR reset_wifi_man_configs() {
  Serial.println("Resetting preferences and core...");
  Serial.flush();
  if (wifi_man_complete) {
    wifi_man_reset();
  }
  ESP.restart();
}

void coordinator_register_device() {
  ArduinoJson::JsonDocument json;
  int resp_code;
  static char buf[256];
  memset(buf, 0, 256);

  json["name"] = deviceName;
  json["type"] = "camera";

  Serial.println(coordinatorIP.toString());
  Serial.println(httpPort);
  http.begin(coordinatorIP.toString(), httpPort, "/api/device/register");
  http.addHeader("Content-Type", "application/json");
  serializeJson(json, buf);
  resp_code = http.PUT(buf);

  Serial.printf("HTTP Response: %d", resp_code);
  Serial.println();
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
  // Matches "sensor/"
  else if (strncmp(topic, sensor_topic_prefix, strlen(sensor_topic_prefix)) == 0) {
     // topic starts with "sensor/"
    String sensorName = String(topic + strlen(sensor_topic_prefix)); // get the part after "sensor/"

    if (std::none_of(mapped_sensors.begin(), mapped_sensors.end(), 
      [=](const String& s) {
        return s == sensorName;
    })) { 
      // Not a sensor to respond to
      return; 
    }

    Serial.printf("%s is a mapped sensor!", sensorName.c_str());
    Serial.println();

  }
  else {
    Serial.printf("Uknown topic: %s", topic);
    Serial.println();
  }

}