# EthWiFiManager

Librería para ESP32 que gestiona conectividad con preferencia por Ethernet W5500 y fallback automático a WiFi.

## Características

- Modo combinado: Ethernet preferido + fallback WiFi.
- Modo WiFi-only (deshabilitando Ethernet por configuración).
- DHCP por defecto en WiFi y Ethernet.
- IP estática opcional en WiFi y Ethernet con API tipo `config(...)`.
- API sencilla y parecida a uso básico de `WiFi`:
  - `begin()`
  - `status()`
  - `localIP()`
  - `gatewayIP()`
  - `subnetMask()`
  - `RSSI()`

## Integración

Usa la librería desde `lib/EthWiFiManager/src/EthWiFiManager.h`.

Revisa ejemplos en `lib/EthWiFiManager/examples`.

Ejemplos incluidos:
- `EthWiFiFailover` (DHCP en ambos)
- `WiFiOnly` (solo WiFi, DHCP)
- `WiFiStatic` (solo WiFi estática)
- `EthernetStaticFallbackWiFi` (Ethernet estática + WiFi fallback)
- `BothStatic` (WiFi y Ethernet estáticas)

## DHCP y estática

Por defecto ambas interfaces usan DHCP.

```cpp
EthWiFiManager::Config cfg;
cfg.wifi.ssid = "SSID";
cfg.wifi.password = "PASS";

// WiFi estática (opcional)
cfg.wifi.config(IPAddress(192,168,5,50), IPAddress(192,168,5,1), IPAddress(255,255,255,0), IPAddress(1,1,1,1), IPAddress(8,8,8,8));

// Ethernet estática (opcional)
cfg.ethernet.config(IPAddress(192,168,5,60), IPAddress(192,168,5,1), IPAddress(255,255,255,0), IPAddress(1,1,1,1), IPAddress(8,8,8,8));

// Para volver explícitamente a DHCP:
cfg.wifi.useDHCP();
cfg.ethernet.useDHCP();
```
