/**
 * W5500 — SPI Ethernet module example
 *
 * Connects using a WIZnet W5500 SPI module with WiFi as fallback.
 * Adjust SPI pins and WiFi credentials to match your hardware.
 *
 * Compatible targets: ESP32, ESP32-S3, and any target with SPI.
 */

#include <Arduino.h>
#include <EthWiFiManager.h>

#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"

EthWiFiManager network;

static void printStatus()
{
    if (network.status() == WL_CONNECTED)
    {
        Serial.printf("[%s] IP: %s  GW: %s\n",
                      network.activeInterfaceName(),
                      network.localIP().toString().c_str(),
                      network.gatewayIP().toString().c_str());
    }
    else
    {
        Serial.println("Not connected");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    EthWiFiManager::Config config;
    config.logTag = "W5500";

    config.wifi.ssid          = WIFI_SSID;
    config.wifi.password      = WIFI_PASS;
    config.wifi.autoReconnect = true;

    config.ethernet.enabled   = true;
    config.ethernet.mode      = EthWiFiManager::EthernetMode::Spi;
    config.ethernet.spiModule = EthWiFiManager::SpiModule::W5500;
    config.ethernet.spiHost   = SPI2_HOST;
    config.ethernet.sckPin    = GPIO_NUM_13;
    config.ethernet.misoPin   = GPIO_NUM_12;
    config.ethernet.mosiPin   = GPIO_NUM_11;
    config.ethernet.csPin     = GPIO_NUM_14;
    config.ethernet.intPin    = GPIO_NUM_10; // must be physically wired

    network.begin(config);
}

void loop()
{
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 5000)
    {
        lastPrint = millis();
        printStatus();
    }
    delay(100);
}
