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
