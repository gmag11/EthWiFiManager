# EthWiFiManager

ESP32 library that manages connectivity with a preference for W5500 Ethernet and automatic fallback to WiFi.

## Features

- Combined mode: Ethernet preferred with WiFi fallback.
- WiFi-only mode (disable Ethernet via configuration).
- DHCP by default on both WiFi and Ethernet.
- Optional static IP for WiFi and Ethernet via a `config(...)` style API.
- Simple API similar to the basic `WiFi` usage:
  - `begin()`
  - `status()`
  - `localIP()`
  - `gatewayIP()`
  - `subnetMask()`
  - `RSSI()`

## Integration

Use the library from `lib/EthWiFiManager/src/EthWiFiManager.h`.

See examples in `lib/EthWiFiManager/examples`.

Included examples:

- `EthWiFiFailover` (DHCP on both interfaces)
- `WiFiOnly` (WiFi only, DHCP)
- `WiFiStatic` (WiFi with static IP)
- `EthernetStaticFallbackWiFi` (static Ethernet + WiFi fallback)
- `BothStatic` (static WiFi and Ethernet)

## Supported Ethernet Hardware

ESP-IDF supports two main categories of Ethernet interfaces; this library uses the ESP-IDF ethernet drivers underneath.

- SPI Ethernet modules (work on all Espressif targets):
  - W5500 (`esp_eth_mac_new_w5500`) — official and commonly used (WIZnet).
  - DM9051 (`esp_eth_mac_new_dm9051`) — official driver for DM9051 modules.
  - KSZ8851SNL / similar SPI MACs (`esp_eth_mac_new_ksz8851snl`) — official in ESP-IDF.

  Note: SPI modules are interrupt-driven and require a physical INT GPIO wired to the MCU; the driver validates the pin number at init.

- Internal EMAC + external PHY (ESP32 with RMII only):
  - IP101 (`esp_eth_phy_new_ip101`)
  - RTL8201 (`esp_eth_phy_new_rtl8201`)
  - LAN87xx / LAN8720 (`esp_eth_phy_new_lan87xx`)
  - DP83848 (`esp_eth_phy_new_dp83848`)
  - KSZ80xx family (`esp_eth_phy_new_ksz80xx`)
  - Generic IEEE-802.3 PHY (`esp_eth_phy_new_generic`) for chips without a specific driver

  Note: the internal EMAC + PHY flow applies to chips with a built-in EMAC (original ESP32 family using RMII). When using RMII you must pay attention to RMII clock source (REF_CLK) and pin assignments.

## DHCP and Static IPs

Both interfaces use DHCP by default.

```cpp
EthWiFiManager::Config cfg;
cfg.wifi.ssid = "SSID";
cfg.wifi.password = "PASS";

// Static WiFi (optional)
cfg.wifi.config(IPAddress(192,168,5,50), IPAddress(192,168,5,1), IPAddress(255,255,255,0), IPAddress(1,1,1,1), IPAddress(8,8,8,8));

// Static Ethernet (optional)
cfg.ethernet.config(IPAddress(192,168,5,60), IPAddress(192,168,5,1), IPAddress(255,255,255,0), IPAddress(1,1,1,1), IPAddress(8,8,8,8));

// To explicitly revert to DHCP:
cfg.wifi.useDHCP();
cfg.ethernet.useDHCP();
```
