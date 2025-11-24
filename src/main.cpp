#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

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


// Functions
void coordinator_register_device();

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

  timeClient.begin();

  coordinator_register_device();
}

void loop() {
  mqttClient.loop(); 
  timeClient.update();

  if (!mqttClient.connected()) {
    mqttClient.connect(deviceName.c_str());
    Serial.println("Reconnected to broker!");
  }

  delay(5000);
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