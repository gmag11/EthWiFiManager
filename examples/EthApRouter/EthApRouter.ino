/**
 * EthApRouter — AP-Router mode example
 *
 * The ESP32 acts as a WiFi access point and shares its Ethernet connection
 * with connected clients, just like a home router.
 *
 * Network topology:
 *   Internet ──► Ethernet ──► ESP32 (NAT) ──► WiFi AP ──► client devices
 *
 * AP subnet  : 192.168.4.0/24  (assigned by the built-in DHCP server)
 * Ethernet   : whatever DHCP assigns from the upstream router
 *
 * Prerequisites (sdkconfig / board_build.cmake_extra_args):
 *   CONFIG_LWIP_IP_FORWARD=y
 *   CONFIG_LWIP_IP_NAPT=y
 *
 * For PlatformIO add to platformio.ini:
 *   board_build.cmake_extra_args =
 *       -DCONFIG_LWIP_IP_FORWARD=y
 *       -DCONFIG_LWIP_IP_NAPT=y
 */

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

    // --- Ethernet (upstream) ---
    config.ethernet.enabled = true;
    config.ethernet.spiHost  = SPI2_HOST;
    config.ethernet.sckPin   = GPIO_NUM_13;
    config.ethernet.misoPin  = GPIO_NUM_12;
    config.ethernet.mosiPin  = GPIO_NUM_11;
    config.ethernet.csPin    = GPIO_NUM_14;
    config.ethernet.intPin   = GPIO_NUM_10;
    // Ethernet uses DHCP by default; call config.ethernet.config(...) for a static IP.

    // --- WiFi AP ---
    config.apSsid           = AP_SSID;
    config.apPassword       = AP_PASS;   // nullptr → open network
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
