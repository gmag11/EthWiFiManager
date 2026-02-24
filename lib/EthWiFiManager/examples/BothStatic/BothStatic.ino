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

    config.ethernet.enabled = true;
    config.ethernet.spiHost = SPI2_HOST;
    config.ethernet.sckPin = GPIO_NUM_13;
    config.ethernet.misoPin = GPIO_NUM_12;
    config.ethernet.mosiPin = GPIO_NUM_11;
    config.ethernet.csPin = GPIO_NUM_14;
    config.ethernet.intPin = GPIO_NUM_10;

    config.wifi.config(
        IPAddress(192, 168, 5, 50),
        IPAddress(192, 168, 5, 1),
        IPAddress(255, 255, 255, 0),
        IPAddress(1, 1, 1, 1),
        IPAddress(8, 8, 8, 8));

    config.ethernet.config(
        IPAddress(192, 168, 5, 60),
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
