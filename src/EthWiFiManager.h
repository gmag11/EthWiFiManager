#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include <esp_eth.h>
#include <esp_eth_mac.h>
#include <esp_eth_phy.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_wifi.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <sdkconfig.h>
#include <functional>

// ── AP-Router feature flag ─────────────────────────────────────────────────────
// AP-Router mode (WiFi AP + Ethernet NAT + built-in DNS forwarder) is compiled
// out by default to keep the binary small for non-router builds.
// Enable by adding  -DETHWIFI_AP_ROUTER  to build_flags in platformio.ini.
// ─────────────────────────────────────────────────────────────────────────────
#if defined(ETHWIFI_AP_ROUTER)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <fcntl.h>
#if defined(CONFIG_LWIP_IPV4_NAPT) && CONFIG_LWIP_IPV4_NAPT
#include <lwip/lwip_napt.h>
#include <lwip/tcpip.h>
#endif
#endif // ETHWIFI_AP_ROUTER

// ── EthWiFiManager per-module feature flags ───────────────────────────────────
// Each Ethernet backend is enabled by default when the underlying ESP-IDF
// Kconfig supports it. Opt out of individual backends to save flash by passing
// -DETHWIFI_NO_W5500, -DETHWIFI_NO_DM9051, -DETHWIFI_NO_KSZ8851SNL, or
// -DETHWIFI_NO_INTERNAL_EMAC via build_flags in platformio.ini.
//
// Example (save ~10 KB by disabling unused SPI modules):
//   build_flags = -DETHWIFI_NO_DM9051 -DETHWIFI_NO_KSZ8851SNL
// ─────────────────────────────────────────────────────────────────────────────
#if defined(CONFIG_ETH_SPI_ETHERNET_W5500) && !defined(ETHWIFI_NO_W5500)
  #define ETHWIFI_W5500 1
#endif
#if defined(CONFIG_ETH_SPI_ETHERNET_DM9051) && !defined(ETHWIFI_NO_DM9051)
  #define ETHWIFI_DM9051 1
#endif
#if defined(CONFIG_ETH_SPI_ETHERNET_KSZ8851SNL) && !defined(ETHWIFI_NO_KSZ8851SNL)
  #define ETHWIFI_KSZ8851SNL 1
#endif
#if defined(CONFIG_ETH_USE_ESP32_EMAC) && !defined(ETHWIFI_NO_INTERNAL_EMAC)
  #define ETHWIFI_INTERNAL_EMAC 1
#endif

#if !defined(ETHWIFI_W5500) && !defined(ETHWIFI_DM9051) && \
    !defined(ETHWIFI_KSZ8851SNL) && !defined(ETHWIFI_INTERNAL_EMAC)
  #pragma message("EthWiFiManager: no Ethernet backend is enabled — Ethernet "\
                  "will always be disabled. Enable at least one backend by "\
                  "removing the corresponding ETHWIFI_NO_* flag.")
#endif
// ─────────────────────────────────────────────────────────────────────────────

class EthWiFiManager
{
public:
    enum class ActiveInterface
    {
        None,
        WiFi,
        Ethernet,
    };

    /// Events reported to the user callback registered with onEvent().
    enum class Event
    {
        EthLinkUp,          ///< Ethernet cable connected / carrier detected
        EthLinkDown,        ///< Ethernet cable disconnected (IP is also lost)
        EthGotIP,           ///< Ethernet obtained an IP address
        WiFiConnected,      ///< WiFi STA associated with AP (layer 2 only)
        WiFiDisconnected,   ///< WiFi STA disconnected from AP (IP is also lost)
        WiFiGotIP,          ///< WiFi obtained an IP — only fired when Ethernet is not active
        InterfaceChanged,   ///< Active interface changed (Eth<->WiFi or ->None)
        EthernetDisabled,   ///< SPI module not detected at startup; Ethernet was disabled
    };

    /// Callback invoked on network state changes.
    /// ip is the relevant address (0.0.0.0 if not applicable).
    /// Called from the ESP-IDF event loop task — keep handlers short or defer work.
    using EventCallback = std::function<void(Event event, IPAddress ip)>;

    enum class SpiModule
    {
        W5500,
        DM9051,
        KSZ8851SNL,
    };

    /// Selects between an SPI-attached module and the internal RMII EMAC (ESP32 classic only).
    enum class EthernetMode
    {
        Spi,         ///< External SPI module (W5500, DM9051, KSZ8851SNL)
#if ETHWIFI_INTERNAL_EMAC
        InternalEmac ///< Internal RMII EMAC with external PHY — ESP32 classic only
#endif
    };

#if ETHWIFI_INTERNAL_EMAC
    /// PHY chip wired to the internal EMAC via RMII.
    enum class EmacPhyChip
    {
        IP101,       ///< IC Plus IP101
        RTL8201,     ///< Realtek RTL8201
        LAN8720,     ///< Microchip LAN8720 / LAN87xx family
        DP83848,     ///< TI DP83848
        KSZ8041,     ///< Microchip KSZ8041
        KSZ8081,     ///< Microchip KSZ8081
    };
#endif

    struct WiFiConfig
    {
        const char *ssid = nullptr;
        const char *password = nullptr;
        bool autoReconnect = true;
        bool useDhcp = true;
        IPAddress localIP = IPAddress((uint32_t)0);
        IPAddress gateway = IPAddress((uint32_t)0);
        IPAddress subnet = IPAddress((uint32_t)0);
        IPAddress dns1 = IPAddress((uint32_t)0);
        IPAddress dns2 = IPAddress((uint32_t)0);

        bool config(IPAddress local_ip, IPAddress gateway_ip, IPAddress subnet_mask,
                    IPAddress dns1_ip = IPAddress((uint32_t)0),
                    IPAddress dns2_ip = IPAddress((uint32_t)0))
        {
            localIP = local_ip;
            gateway = gateway_ip;
            subnet = subnet_mask;
            dns1 = dns1_ip;
            dns2 = dns2_ip;
            useDhcp = false;
            return true;
        }

        void useDHCP()
        {
            useDhcp = true;
        }
    };

    struct EthernetConfig
    {
        bool enabled = true;

        // Select Ethernet backend
#if ETHWIFI_INTERNAL_EMAC
        EthernetMode mode = EthernetMode::InternalEmac; ///< Default: internal EMAC when available and not disabled
#else
        EthernetMode mode = EthernetMode::Spi;          ///< Default: SPI module
#endif

        // --- SPI module (used when mode == Spi) ---
        SpiModule spiModule = SpiModule::W5500;
        spi_host_device_t spiHost = SPI2_HOST;
        gpio_num_t sckPin = GPIO_NUM_13;
        gpio_num_t misoPin = GPIO_NUM_12;
        gpio_num_t mosiPin = GPIO_NUM_11;
        gpio_num_t csPin = GPIO_NUM_14;
        gpio_num_t intPin = GPIO_NUM_10;
        int spiClockHz = 20 * 1000 * 1000;
        int spiQueueSize = 20;
        int macRxTaskStackSize = 4096;

#if ETHWIFI_INTERNAL_EMAC
        // --- Internal EMAC + external PHY via RMII (used when mode == InternalEmac; ESP32 only) ---
        EmacPhyChip emacPhyChip = EmacPhyChip::LAN8720; ///< PHY chip model
        int emacPhyAddr = 1;                             ///< SMI PHY address (1 = typical; -1 = auto-detect)
        gpio_num_t emacMdcPin = GPIO_NUM_23;             ///< RMII MDC clock pin
        gpio_num_t emacMdioPin = GPIO_NUM_18;            ///< RMII MDIO data pin
        gpio_num_t emacPhyResetPin = GPIO_NUM_NC;        ///< Active-low PHY reset, or GPIO_NUM_NC
        gpio_num_t emacRmiiRefClkPin = GPIO_NUM_0;       ///< REF_CLK pin — GPIO0 typical for external input
        bool emacRmiiClockExtInput = true;               ///< true = PHY drives REF_CLK (most boards)
#endif

        bool useDhcp = true;
        IPAddress localIP = IPAddress((uint32_t)0);
        IPAddress gateway = IPAddress((uint32_t)0);
        IPAddress subnet = IPAddress((uint32_t)0);
        IPAddress dns1 = IPAddress((uint32_t)0);
        IPAddress dns2 = IPAddress((uint32_t)0);

        bool config(IPAddress local_ip, IPAddress gateway_ip, IPAddress subnet_mask,
                    IPAddress dns1_ip = IPAddress((uint32_t)0),
                    IPAddress dns2_ip = IPAddress((uint32_t)0))
        {
            localIP = local_ip;
            gateway = gateway_ip;
            subnet = subnet_mask;
            dns1 = dns1_ip;
            dns2 = dns2_ip;
            useDhcp = false;
            return true;
        }

        void useDHCP()
        {
            useDhcp = true;
        }
    };

    struct Config
    {
        const char *logTag = "EthWiFiManager";
        WiFiConfig wifi = {};
        EthernetConfig ethernet = {};
    };

#if defined(ETHWIFI_AP_ROUTER)
    /// Configuration for AP-Router mode: Ethernet upstream + WiFi AP with NAT.
    /// Requires CONFIG_LWIP_IP_FORWARD=y and CONFIG_LWIP_IPV4_NAPT=y in sdkconfig.
    /// Also requires -DETHWIFI_AP_ROUTER in build_flags.
    struct ApRouterConfig
    {
        const char *logTag = "EthWiFiManager";
        EthernetConfig ethernet = {};

        // WiFi AP settings
        const char *apSsid = "ESP32-Router";
        const char *apPassword = nullptr; ///< nullptr or <8 chars = open; >=8 chars = WPA2-PSK
        uint8_t apChannel = 1;
        uint8_t apMaxConnections = 4;

        // AP subnet — must differ from the Ethernet subnet
        IPAddress apLocalIP = IPAddress(192, 168, 4, 1);
        IPAddress apGateway = IPAddress(192, 168, 4, 1);
        IPAddress apSubnet  = IPAddress(255, 255, 255, 0);

        /// Initial upstream DNS used by the built-in DNS forwarder before Ethernet
        /// obtains its own DNS from DHCP. Overridden automatically once Ethernet
        /// has an IP. Set to IPAddress(0,0,0,0) to leave unset until Ethernet is up.
        IPAddress apFallbackDns = IPAddress(8, 8, 8, 8);
    };
#endif // ETHWIFI_AP_ROUTER

    bool begin(const Config &config);

#if defined(ETHWIFI_AP_ROUTER)
    /// Start in AP-Router mode: WiFi AP shares the Ethernet connection via NAT.
    /// Compile with -DETHWIFI_AP_ROUTER to enable this mode.
    bool beginApRouter(const ApRouterConfig &config);
#endif // ETHWIFI_AP_ROUTER

    wl_status_t status() const;
    IPAddress localIP() const;
    IPAddress gatewayIP() const;
    IPAddress subnetMask() const;
    int32_t RSSI() const;

    bool ethernetLinkUp() const;
    bool ethernetHasIP() const;
    ActiveInterface activeInterface() const;
    const char *activeInterfaceName() const;

    /// Register a callback invoked on network state changes.
    /// May be called before or after begin(). Replaces any previously registered callback.
    void onEvent(EventCallback cb);

private:
    static EthWiFiManager *s_instance;

    Config m_config = {};
    bool m_started = false;
    bool m_eventsRegistered = false;

    esp_netif_t *m_ethNetif = nullptr;
    esp_eth_handle_t m_ethHandle = nullptr;
    esp_eth_netif_glue_handle_t m_ethGlue = nullptr;

#if defined(ETHWIFI_AP_ROUTER)
    bool m_apRouterMode = false;
    ApRouterConfig m_apRouterConfig = {};
    esp_netif_t *m_apNetif = nullptr;

    // DNS proxy
    static constexpr uint16_t DNS_PROXY_PORT    = 53;
    static constexpr int      DNS_PROXY_PENDING = 8;    ///< max concurrent queries
    static constexpr uint32_t DNS_PROXY_TIMEOUT = 3000; ///< ms per upstream query
    TaskHandle_t              m_dnsProxyTask    = nullptr;
    volatile uint32_t         m_upstreamDns     = 0;   ///< upstream DNS in network byte order
#endif // ETHWIFI_AP_ROUTER

    spi_device_interface_config_t m_spiDevCfg = {};

    volatile bool m_ethHasIp = false;
    volatile bool m_ethLinkUp = false;

    EventCallback m_eventCallback;
    void fireEvent(Event event, IPAddress ip = IPAddress((uint32_t)0));

    bool initCore();
    bool initWiFi();
    bool initEthernet();
    bool probeSpiModule();

    esp_netif_t *wifiNetif() const;
    void startWiFi();
    void stopWiFi();

#if defined(ETHWIFI_AP_ROUTER)
    bool initApRouter();
    void updateApDns();
    void startDnsProxy();
    void stopDnsProxy();
    void dnsProxyLoop();
    static void dnsProxyTaskThunk(void *arg);
#endif // ETHWIFI_AP_ROUTER

    void onEthEvent(int32_t eventId);
    void onIpEvent(int32_t eventId, void *eventData);
    void onWiFiEvent(int32_t eventId);

    static void ethEventThunk(void *arg, esp_event_base_t base, int32_t eventId, void *eventData);
    static void ipEventThunk(void *arg, esp_event_base_t base, int32_t eventId, void *eventData);
    static void wifiEventThunk(void *arg, esp_event_base_t base, int32_t eventId, void *eventData);
};
