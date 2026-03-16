/**
 * InternalEmac — ESP32 internal RMII EMAC example
 *
 * Connects using the ESP32's built-in Ethernet MAC paired with an external
 * PHY (LAN8720A) over RMII, with WiFi as fallback.
 *
 * Default pin assignment matches the WT32-ETH01 / WT32-ETH02 boards:
 *   MDC      → GPIO23
 *   MDIO     → GPIO18
 *   REF_CLK  → GPIO0  (50 MHz input from on-board oscillator via LAN8720A)
 *   PHY rst  → GPIO16
 *   PHY addr → 1
 *
 * For other boards (Olimex ESP32-EVB, ESP32-Gateway, etc.) adjust the pins
 * and clock direction below.
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
    config.ethernet.emacPhyAddr           = 1;            // SMI address — 1 on WT32-ETH01/ETH02
    config.ethernet.emacMdcPin            = GPIO_NUM_23;  // MDC
    config.ethernet.emacMdioPin           = GPIO_NUM_18;  // MDIO
    config.ethernet.emacPhyResetPin       = GPIO_NUM_16;  // Power/reset — GPIO16 on WT32-ETH01/ETH02
    config.ethernet.emacRmiiRefClkPin     = GPIO_NUM_0;   // REF_CLK input from on-board oscillator
    config.ethernet.emacRmiiClockExtInput = true;          // External 50 MHz oscillator drives REF_CLK
    // Reduce auto-neg timeout and link-check period to speed up link detection and recovery.
    // With IDF defaults (4000 + 2000 ms), each failed auto-neg cycle takes 6 s;
    // ~5 cycles on cold start = ~30 s delay. Values below reduce that to ~5 s.
    config.ethernet.emacAutoNegoTimeoutMs = 1500; // ms (IDF default: 4000)
    config.ethernet.emacLinkCheckPeriodMs = 500;  // ms (IDF default: 2000)

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
