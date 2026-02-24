#include <Arduino.h>
#include <EthWiFiManager.h>

#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"

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
        IPAddress(192, 168, 1, 50),
        IPAddress(192, 168, 1, 1),
        IPAddress(255, 255, 255, 0),
        IPAddress(1, 1, 1, 1),
        IPAddress(8, 8, 8, 8));

    network.begin(config);
}

void loop()
{
    delay(100);
}
