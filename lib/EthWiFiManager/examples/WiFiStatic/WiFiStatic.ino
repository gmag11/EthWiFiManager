#include <Arduino.h>
#include <EthWiFiManager.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "SSID"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "PASSWORD"
#endif

EthWiFiManager network;

void setup()
{
    Serial.begin(115200);
    delay(1500);

    EthWiFiManager::Config config;
    config.logTag = "network";
    config.wifi.ssid = WIFI_SSID;
    config.wifi.password = WIFI_PASS;
    config.wifi.autoReconnect = true;

    config.ethernet.enabled = false;

    config.wifi.config(
        IPAddress(192, 168, 5, 50),
        IPAddress(192, 168, 5, 1),
        IPAddress(255, 255, 255, 0),
        IPAddress(1, 1, 1, 1),
        IPAddress(8, 8, 8, 8));

    network.begin(config);
}

void loop()
{
    delay(100);
}
