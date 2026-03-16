/**
 * EthApRouterInternalEmac — AP-Router mode with ESP32 internal RMII EMAC
 *
 * The ESP32 acts as a WiFi access point and shares its Ethernet connection
 * with connected clients, just like a home router.
 *
 * Network topology:
 *   Internet ──► Ethernet (LAN8720A) ──► ESP32 (NAT) ──► WiFi AP ──► client devices
 *
 * AP subnet  : 192.168.4.0/24  (assigned by the built-in DHCP server)
 * Ethernet   : whatever DHCP assigns from the upstream router
 *
 * Default pin assignment matches the WT32-ETH01 / WT32-ETH02 boards:
 *   MDC      → GPIO23
 *   MDIO     → GPIO18
 *   REF_CLK  → GPIO0  (50 MHz input from on-board oscillator via LAN8720A)
 *   PHY rst  → GPIO16
 *   PHY addr → 1
 *
 * Prerequisites (build_flags in platformio.ini):
 *   -DETHWIFI_AP_ROUTER
 *
 * Prerequisites (sdkconfig / board_build.cmake_extra_args):
 *   CONFIG_LWIP_IP_FORWARD=y
 *   CONFIG_LWIP_IPV4_NAPT=y
 *
 * Compatible targets: ESP32 classic ONLY (CONFIG_ETH_USE_ESP32_EMAC).
 */

#include <sdkconfig.h>
#if CONFIG_ETH_USE_ESP32_EMAC

#include <Arduino.h>
#include <EthWiFiManager.h>

#define AP_SSID "ESP32-Router"
#define AP_PASS "mypassword"  // min 8 chars for WPA2-PSK; set nullptr for open network

EthWiFiManager network;

void setup()
{
    Serial.begin(115200);
    delay(1500);

    EthWiFiManager::ApRouterConfig config;
    config.logTag = "router";

    // --- Ethernet upstream (internal EMAC + LAN8720A) ---
    config.ethernet.enabled               = true;
    config.ethernet.mode                  = EthWiFiManager::EthernetMode::InternalEmac;
    config.ethernet.emacPhyChip           = EthWiFiManager::EmacPhyChip::LAN8720;
    config.ethernet.emacPhyAddr           = 1;            // SMI address — 1 on WT32-ETH01/ETH02
    config.ethernet.emacMdcPin            = GPIO_NUM_23;  // MDC
    config.ethernet.emacMdioPin           = GPIO_NUM_18;  // MDIO
    config.ethernet.emacPhyResetPin       = GPIO_NUM_16;  // Power/reset — GPIO16 on WT32-ETH01/ETH02
    config.ethernet.emacRmiiRefClkPin     = GPIO_NUM_0;   // REF_CLK input from on-board oscillator
    config.ethernet.emacRmiiClockExtInput = true;          // External 50 MHz oscillator drives REF_CLK
    config.ethernet.emacAutoNegoTimeoutMs = 1500;          // ms (IDF default: 4000)
    config.ethernet.emacLinkCheckPeriodMs = 500;           // ms (IDF default: 2000)
    // Ethernet uses DHCP by default; call config.ethernet.config(...) for a static IP.

    // --- WiFi AP ---
    config.apSsid           = AP_SSID;
    config.apPassword       = AP_PASS;  // nullptr → open network
    config.apChannel        = 1;
    config.apMaxConnections = 4;

    // AP subnet (must differ from the Ethernet subnet)
    config.apLocalIP = IPAddress(192, 168, 4, 1);
    config.apGateway = IPAddress(192, 168, 4, 1);
    config.apSubnet  = IPAddress(255, 255, 255, 0);

    if (!network.beginApRouter(config))
    {
        Serial.println("beginApRouter failed — check wiring and sdkconfig NAPT flags");
    }
}

void loop()
{
    if (network.ethernetHasIP())
    {
        Serial.print("[ETH] IP: ");
        Serial.println(network.localIP());
    }
    delay(5000);
}

#endif // CONFIG_ETH_USE_ESP32_EMAC
