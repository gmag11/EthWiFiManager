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

class EthWiFiManager
{
public:
    enum class ActiveInterface
    {
        None,
        WiFi,
        Ethernet,
    };

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
#if CONFIG_ETH_USE_ESP32_EMAC
        InternalEmac ///< Internal RMII EMAC with external PHY — ESP32 classic only
#endif
    };

#if CONFIG_ETH_USE_ESP32_EMAC
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
#if defined(CONFIG_IDF_TARGET_ESP32) && defined(CONFIG_ETH_USE_ESP32_EMAC)
        EthernetMode mode = EthernetMode::InternalEmac; ///< Default: internal EMAC on ESP32 classic
#else
        EthernetMode mode = EthernetMode::Spi;          ///< Default: SPI module on all other targets
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

#if CONFIG_ETH_USE_ESP32_EMAC
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

    bool begin(const Config &config);

    wl_status_t status() const;
    IPAddress localIP() const;
    IPAddress gatewayIP() const;
    IPAddress subnetMask() const;
    int32_t RSSI() const;

    bool ethernetLinkUp() const;
    bool ethernetHasIP() const;
    ActiveInterface activeInterface() const;
    const char *activeInterfaceName() const;

private:
    static EthWiFiManager *s_instance;

    Config m_config = {};
    bool m_started = false;
    bool m_eventsRegistered = false;

    esp_netif_t *m_ethNetif = nullptr;
    esp_eth_handle_t m_ethHandle = nullptr;
    esp_eth_netif_glue_handle_t m_ethGlue = nullptr;
    spi_device_handle_t m_spiHandle = nullptr;

    spi_device_interface_config_t m_spiDevCfg = {};

    volatile bool m_ethHasIp = false;
    volatile bool m_ethLinkUp = false;

    bool initCore();
    bool initWiFi();
    bool initEthernet();
    bool probeSpiModule();

    esp_netif_t *wifiNetif() const;
    void startWiFi();
    void stopWiFi();

    void onEthEvent(int32_t eventId);
    void onIpEvent(int32_t eventId, void *eventData);
    void onWiFiEvent(int32_t eventId);

    static void ethEventThunk(void *arg, esp_event_base_t base, int32_t eventId, void *eventData);
    static void ipEventThunk(void *arg, esp_event_base_t base, int32_t eventId, void *eventData);
    static void wifiEventThunk(void *arg, esp_event_base_t base, int32_t eventId, void *eventData);
};
