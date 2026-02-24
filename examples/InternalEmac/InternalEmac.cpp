/**
 * InternalEmac — ESP32 internal RMII EMAC example
 *
 * Connects using the ESP32's built-in Ethernet MAC paired with an external
 * PHY (default: LAN8720) over RMII, with WiFi as fallback.
 *
 * Adjust PHY type, pins, and WiFi credentials to match your hardware.
 * Common boards: WT32-ETH01, Olimex ESP32-EVB, ESP32-Gateway, etc.
 *
 * Compatible targets: ESP32 classic ONLY (CONFIG_ETH_USE_ESP32_EMAC).
 * This file compiles to an empty unit on targets without an internal EMAC.
 * Use the esp32_internal_emac PlatformIO environment to build it.
 */

// Guard the entire file: only compile on targets with an internal EMAC.
// The library scanner compiles every .cpp in examples/ for all environments;
// this guard prevents any code from being emitted on ESP32-S3/C3/etc.
// sdkconfig.h must be included first so CONFIG_ETH_USE_ESP32_EMAC is defined.
#include <sdkconfig.h>
#if CONFIG_ETH_USE_ESP32_EMAC

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
    config.logTag = "InternalEmac";

    config.wifi.ssid          = WIFI_SSID;
    config.wifi.password      = WIFI_PASS;
    config.wifi.autoReconnect = true;

    config.ethernet.enabled               = true;
    config.ethernet.mode                  = EthWiFiManager::EthernetMode::InternalEmac;
    config.ethernet.emacPhyChip           = EthWiFiManager::EmacPhyChip::LAN8720;
    config.ethernet.emacPhyAddr           = 1;           // SMI address (0 or 1 on most boards)
    config.ethernet.emacMdcPin            = GPIO_NUM_23; // MDC
    config.ethernet.emacMdioPin           = GPIO_NUM_18; // MDIO
    config.ethernet.emacPhyResetPin       = GPIO_NUM_NC; // set to reset pin if wired
    config.ethernet.emacRmiiRefClkPin     = GPIO_NUM_0;  // REF_CLK input from PHY → GPIO0
    config.ethernet.emacRmiiClockExtInput = true;         // PHY provides the 50 MHz REF_CLK

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

#endif // CONFIG_ETH_USE_ESP32_EMAC
