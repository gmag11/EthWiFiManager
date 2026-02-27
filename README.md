# EthWiFiManager

ESP32 library that manages connectivity with a preference for SPI Ethernet (W5500, DM9051, or KSZ8851SNL) and automatic fallback to WiFi.

## Features

- Combined mode: Ethernet preferred with WiFi fallback.
- **AP-Router mode** (opt-in with `-DETHWIFI_AP_ROUTER`): WiFi access point that shares an Ethernet connection via NAT — turn the ESP32 into a router. Includes a built-in DNS forwarder on port 53.
- WiFi-only mode (disable Ethernet via configuration).
- DHCP by default on both WiFi and Ethernet.
- Optional static IP for WiFi and Ethernet via a `config(...)` style API.
- Simple API similar to the basic `WiFi` usage:
  - `begin()`
  - `beginApRouter()`
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
- `EthApRouter` (AP-Router: WiFi AP sharing Ethernet via NAT)

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

## Configuring the SPI Ethernet Module

The library supports all three SPI Ethernet modules available in ESP-IDF.  
Set `cfg.ethernet.spiModule` to select the chip; the default is `W5500`.

| Value | Chip | SPI framing |
| --- | --- | --- |
| `SpiModule::W5500` (default) | WIZnet W5500 | `command_bits=16`, `address_bits=8` |
| `SpiModule::DM9051` | DAVICOM DM9051 | `command_bits=1`, `address_bits=7` |
| `SpiModule::KSZ8851SNL` | Microchip KSZ8851SNL | `command_bits=16`, `address_bits=0` |

```cpp
EthWiFiManager::Config cfg;

// Select Ethernet chip (default is W5500)
cfg.ethernet.spiModule = EthWiFiManager::SpiModule::DM9051;

// SPI pins
cfg.ethernet.spiHost  = SPI2_HOST;
cfg.ethernet.mosiPin  = GPIO_NUM_11;
cfg.ethernet.misoPin  = GPIO_NUM_12;
cfg.ethernet.sckPin   = GPIO_NUM_13;
cfg.ethernet.csPin    = GPIO_NUM_14;
cfg.ethernet.intPin   = GPIO_NUM_10; // INT pin is mandatory — must be physically wired
```

> **Note**: The ESP-IDF driver for all three SPI Ethernet chips requires a physical interrupt GPIO.  
> `GPIO_NUM_NC` is rejected at init time; always wire the INT pin.

Chip support is compiled in via Kconfig / `sdkconfig`:

- `CONFIG_ETH_SPI_ETHERNET_W5500`
- `CONFIG_ETH_SPI_ETHERNET_DM9051`
- `CONFIG_ETH_SPI_ETHERNET_KSZ8851SNL`

All three are enabled by default in the Arduino-ESP32 framework.

## Configuring the Internal EMAC (ESP32 classic only)

On the original ESP32 (not S2/S3/C3), the chip includes a built-in RMII Ethernet MAC that can be paired with an external PHY.  
The library defaults to `EthernetMode::InternalEmac` automatically when building for `CONFIG_IDF_TARGET_ESP32`.

### Selecting the mode explicitly

```cpp
cfg.ethernet.mode = EthWiFiManager::EthernetMode::InternalEmac; // internal EMAC + PHY
cfg.ethernet.mode = EthWiFiManager::EthernetMode::Spi;          // SPI module
```

### Supported PHY chips

| Value | Chip |
| --- | --- |
| `EmacPhyChip::LAN8720` (default) | Microchip LAN8720 / LAN87xx family |
| `EmacPhyChip::IP101` | IC Plus IP101 |
| `EmacPhyChip::RTL8201` | Realtek RTL8201 |
| `EmacPhyChip::DP83848` | TI DP83848 |
| `EmacPhyChip::KSZ8041` | Microchip KSZ8041 |
| `EmacPhyChip::KSZ8081` | Microchip KSZ8081 |

### EMAC configuration example

```cpp
EthWiFiManager::Config cfg;

// Mode is InternalEmac by default on ESP32 classic
cfg.ethernet.emacPhyChip          = EthWiFiManager::EmacPhyChip::LAN8720;
cfg.ethernet.emacPhyAddr          = 1;          // SMI address of the PHY (typical: 0 or 1)
cfg.ethernet.emacMdcPin           = GPIO_NUM_23; // MDC - management data clock
cfg.ethernet.emacMdioPin          = GPIO_NUM_18; // MDIO - management data I/O
cfg.ethernet.emacPhyResetPin      = GPIO_NUM_5;  // Active-low PHY reset, or GPIO_NUM_NC
cfg.ethernet.emacRmiiRefClkPin    = GPIO_NUM_0;  // REF_CLK pin (GPIO0 = external input, typical)
cfg.ethernet.emacRmiiClockExtInput = true;        // true = PHY drives REF_CLK (most boards)
```

> **RMII clock**: Most hobbyist modules (e.g. WT32-ETH01, Olimex ESP32-EVB) supply an external 50 MHz crystal to the PHY and route REF_CLK back to GPIO0 (`emacRmiiClockExtInput = true`).  
> Set `emacRmiiClockExtInput = false` if your board instead expects the ESP32 to output the clock.
> **Kconfig guards**: PHY drivers are gated by `CONFIG_ETH_PHY_IP101`, `CONFIG_ETH_PHY_RTL8201`, `CONFIG_ETH_PHY_LAN87XX`, `CONFIG_ETH_PHY_DP83848`, `CONFIG_ETH_PHY_KSZ80XX`. All are enabled by default in Arduino-ESP32.
> **Compile-time safety**: `EthernetMode::InternalEmac` and the entire `EmacPhyChip` enum only exist when `CONFIG_ETH_USE_ESP32_EMAC` is defined (ESP32 classic). Attempting to use them on other targets (ESP32-S3, C3, etc.) results in a compile error.

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

## AP-Router Mode

In AP-Router mode the ESP32 works like a router:

```text
Internet ──► Ethernet (upstream) ──► ESP32 (NAT) ──► WiFi AP ──► client devices
```

- The ESP32's WiFi interface runs as an **Access Point** with a built-in DHCP server.
- Client devices connect to the ESP32's AP and receive an IP address automatically.
- All client traffic is forwarded to the Ethernet interface via **NAT**, so it appears
  to the upstream network as if it came from the ESP32's Ethernet IP.
- A **built-in DNS forwarder** listens on port 53 of the AP IP (`192.168.4.1`) and
  forwards queries to the upstream DNS learned from Ethernet DHCP (or `apFallbackDns`
  until Ethernet comes up). This works regardless of what DNS the DHCP server advertises.
- The AP subnet (default `192.168.4.0/24`) must differ from the Ethernet subnet.

> **AP-Router mode is disabled by default** to keep the binary small.
> Enable it with `-DETHWIFI_AP_ROUTER` in `build_flags`.

### Required build configuration

**`build_flags`** (enables the AP-Router code):

```ini
build_flags = -DETHWIFI_AP_ROUTER
```

**`board_build.cmake_extra_args`** (enables NAPT in lwIP):

| Option | Value |
| --- | --- |
| `CONFIG_LWIP_IP_FORWARD` | `y` |
| `CONFIG_LWIP_IPV4_NAPT` | `y` |

```ini
board_build.cmake_extra_args =
    -DCONFIG_LWIP_IP_FORWARD=y
    -DCONFIG_LWIP_IPV4_NAPT=y
```

### Example

```cpp
#include <EthWiFiManager.h>

// Requires -DETHWIFI_AP_ROUTER in build_flags
EthWiFiManager network;

void setup() {
    EthWiFiManager::ApRouterConfig config;

    // Ethernet upstream (DHCP by default)
    config.ethernet.enabled = true;
    config.ethernet.spiHost  = SPI2_HOST;
    config.ethernet.sckPin   = GPIO_NUM_13;
    config.ethernet.misoPin  = GPIO_NUM_12;
    config.ethernet.mosiPin  = GPIO_NUM_11;
    config.ethernet.csPin    = GPIO_NUM_14;
    config.ethernet.intPin   = GPIO_NUM_10;

    // WiFi AP
    config.apSsid           = "ESP32-Router";
    config.apPassword       = "mypassword";   // >=8 chars = WPA2-PSK; nullptr = open
    config.apMaxConnections = 4;

    // AP addressing — must differ from Ethernet subnet
    config.apLocalIP = IPAddress(192, 168, 4, 1);
    config.apGateway = IPAddress(192, 168, 4, 1);
    config.apSubnet  = IPAddress(255, 255, 255, 0);

    network.beginApRouter(config);
}
```

When the library logs `[AP] NAPT enabled`, AP clients can route traffic through
the Ethernet connection. When the library logs `[DNS] Forwarder listening on port 53`,
DNS queries from AP clients are transparently forwarded to the upstream DNS server.

If NAPT is not available the library logs a warning with the sdkconfig options that
need to be enabled.
