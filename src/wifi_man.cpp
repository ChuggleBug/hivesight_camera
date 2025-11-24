
#include <Arduino.h>
#include <WiFiManager.h>

#include "main.h"

// main.h extern defined variables
IPAddress coordinatorIP;
String deviceName;

// Application specific objects
// (exclusive to this file)

WiFiManager wm;
WiFiManagerParameter hostIPParam;
WiFiManagerParameter deviceNameParam;


void saveParams();

void wifi_man_reset() {
  prefs.remove("host_ip");
  prefs.remove("device_name");
  wm.erase();
}

void wifi_man_svc_start() {
  std::vector<const char *> menu;
  const char *ip_html = NULL;
  bool autoconn = false;
  bool has_appconfs = false;

  wm.setConfigPortalBlocking(true);

  // --- Device Name param ---
  new (&deviceNameParam) WiFiManagerParameter(
      "device_name", "Device Name", "", 32,
      "pattern=\"[A-Za-z0-9_]+\" placeholder=\"MyDevice\"");

  // --- IP Address param ---
  ip_html = "<br><label for='ip1'>Host IP</label><br>"
            "<div style=\"display: flex; flex-direction: row;\">"
            "<input name='ip1' maxlength='3' size='3'> ."
            "<input name='ip2' maxlength='3' size='3'> ."
            "<input name='ip3' maxlength='3' size='3'> ."
            "<input name='ip4' maxlength='3' size='3'>"
            "</div>";

  new (&hostIPParam) WiFiManagerParameter(ip_html);

  wm.addParameter(&deviceNameParam);
  wm.addParameter(&hostIPParam);

  wm.setSaveParamsCallback(saveParams);

  menu = {"wifi", "param", "sep", "info", "restart", "exit"};
  wm.setMenu(menu);

  Serial.println("Testing Autoconnect...");
  wm.setConnectTimeout(10);
  autoconn = wm.autoConnect();
  // Considered the special case where ip address if not properly formatted
  // If the value was properly parses, it will be overwritten
  // with the same value
  has_appconfs = prefs.isKey("device_name") && prefs.isKey("host_ip") &&
                 coordinatorIP.fromString(prefs.getString("host_ip"));

  // Start portal if not all configs are present
  if (!(autoconn && has_appconfs)) {
    wm.startConfigPortal();
  } else {
    Serial.println("Skipping portal. All configs present and valid");
  }

  Serial.println("Setting saved preferences for use");
  deviceName = prefs.getString("device_name");
}

void saveParams() {
  Serial.println("Saving parameters...");

  String deviceName = wm.server->arg("device_name");
  String ip1 = wm.server->arg("ip1");
  String ip2 = wm.server->arg("ip2");
  String ip3 = wm.server->arg("ip3");
  String ip4 = wm.server->arg("ip4");

  String ipString = ip1 + "." + ip2 + "." + ip3 + "." + ip4;

  Serial.println("Saving device name as " + deviceName);
  prefs.putString("device_name", deviceName.c_str());
  Serial.println("Saving broker ip as " + ipString);
  prefs.putString("host_ip", ipString.c_str());
}
