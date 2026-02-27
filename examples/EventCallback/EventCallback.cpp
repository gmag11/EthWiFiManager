/**
 * EventCallback — network event callback example
 *
 * Demonstrates how to use onEvent() to receive network state changes
 * using a W5500 SPI module with WiFi fallback.
 *
 * Compatible targets: ESP32, ESP32-S3, and any target with SPI.
 */

#include <Arduino.h>
#include <EthWiFiManager.h>

#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"

EthWiFiManager network;

static const char *eventName(EthWiFiManager::Event ev)
{
    switch (ev)
    {
    case EthWiFiManager::Event::EthLinkUp:        return "EthLinkUp";
    case EthWiFiManager::Event::EthLinkDown:      return "EthLinkDown";
    case EthWiFiManager::Event::EthGotIP:         return "EthGotIP";
    case EthWiFiManager::Event::WiFiConnected:    return "WiFiConnected";
    case EthWiFiManager::Event::WiFiDisconnected: return "WiFiDisconnected";
    case EthWiFiManager::Event::WiFiGotIP:        return "WiFiGotIP";
    case EthWiFiManager::Event::InterfaceChanged: return "InterfaceChanged";
    case EthWiFiManager::Event::EthernetDisabled: return "EthernetDisabled";
    default:                                      return "Unknown";
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    // Register event callback before begin() so no events are missed.
    network.onEvent([](EthWiFiManager::Event event, IPAddress ip) {
        Serial.printf("[EVENT] %-20s  ip=%s\n", eventName(event), ip.toString().c_str());
    });

    EthWiFiManager::Config config;
    config.logTag = "EventCB";

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
    config.ethernet.intPin    = GPIO_NUM_10;
    // config.ethernet.resetPin = GPIO_NUM_9;  // uncomment if reset pin is wired

    network.begin(config);
}

void loop()
{
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 5000)
    {
        lastPrint = millis();
        const char *iface = network.activeInterfaceName();
        if (network.status() == WL_CONNECTED)
        {
            Serial.printf("[%s] IP: %s\n", iface, network.localIP().toString().c_str());
        }
        else
        {
            Serial.printf("[%s] Not connected\n", iface);
        }
    }
    delay(100);
}
