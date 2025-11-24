#ifndef __MAIN_H
#define __MAIN_H

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>

#include "app_config.h"

// Destination of MQTT Broker
// IP adress to be configured via WiFi Manager
extern IPAddress coordinatorIP;
extern const uint16_t brokerPort;
extern const uint16_t httpPort;

// Name of the sensor defiend with WiFi Manager
// Will be used when publishing a topic
extern String deviceName;

// Storage for options that are not automatically
// stored by WiFi Manager
extern Preferences prefs;


#endif // __MAIN_H