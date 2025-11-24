#ifndef __WIFI_MAN_H
#define __WIFI_MAN_H

// Start the wifi manager service
// initializing the AP if required
void wifi_man_svc_start();

// Reset all configurations associated 
// with the WiFi Manager
void wifi_man_reset();

#endif // __WIFI_MAN_H