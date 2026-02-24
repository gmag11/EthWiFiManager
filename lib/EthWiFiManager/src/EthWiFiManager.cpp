#include "EthWiFiManager.h"

#include <lwip/ip4_addr.h>

EthWiFiManager *EthWiFiManager::s_instance = nullptr;

static esp_ip4_addr_t toEspIp4(const IPAddress &ip)
{
    esp_ip4_addr_t out = {};
    IP4_ADDR(&out, ip[0], ip[1], ip[2], ip[3]);
    return out;
}

bool EthWiFiManager::begin(const Config &config)
{
    if (m_started)
    {
        return true;
    }

    m_config = config;
    if (m_config.logTag == nullptr)
    {
        m_config.logTag = "EthWiFiManager";
    }

    s_instance = this;

    if (!initCore())
    {
        return false;
    }

    if (!initWiFi())
    {
        return false;
    }

    if (m_config.ethernet.enabled)
    {
        if (!probeW5500())
        {
            ESP_LOGW(m_config.logTag, "W5500 not detected - Ethernet disabled, WiFi only");
            m_config.ethernet.enabled = false;
        }
        else if (!initEthernet())
        {
            return false;
        }
    }

    startWiFi();
    ESP_LOGI(m_config.logTag, "Manager started (ethernet=%s)", m_config.ethernet.enabled ? "enabled" : "disabled");
    m_started = true;
    return true;
}

wl_status_t EthWiFiManager::status() const
{
    if (m_ethHasIp)
    {
        return WL_CONNECTED;
    }
    return WiFi.status();
}

IPAddress EthWiFiManager::localIP() const
{
    esp_netif_ip_info_t info = {};
    esp_netif_t *netif = nullptr;

    if (m_ethHasIp && m_ethNetif != nullptr)
    {
        netif = m_ethNetif;
    }
    else
    {
        netif = wifiNetif();
    }

    if (netif != nullptr && esp_netif_get_ip_info(netif, &info) == ESP_OK)
    {
        return IPAddress(info.ip.addr);
    }

    return IPAddress((uint32_t)0);
}

IPAddress EthWiFiManager::gatewayIP() const
{
    esp_netif_ip_info_t info = {};
    esp_netif_t *netif = nullptr;

    if (m_ethHasIp && m_ethNetif != nullptr)
    {
        netif = m_ethNetif;
    }
    else
    {
        netif = wifiNetif();
    }

    if (netif != nullptr && esp_netif_get_ip_info(netif, &info) == ESP_OK)
    {
        return IPAddress(info.gw.addr);
    }

    return IPAddress((uint32_t)0);
}

IPAddress EthWiFiManager::subnetMask() const
{
    esp_netif_ip_info_t info = {};
    esp_netif_t *netif = nullptr;

    if (m_ethHasIp && m_ethNetif != nullptr)
    {
        netif = m_ethNetif;
    }
    else
    {
        netif = wifiNetif();
    }

    if (netif != nullptr && esp_netif_get_ip_info(netif, &info) == ESP_OK)
    {
        return IPAddress(info.netmask.addr);
    }

    return IPAddress((uint32_t)0);
}

int32_t EthWiFiManager::RSSI() const
{
    return WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
}

bool EthWiFiManager::ethernetLinkUp() const
{
    return m_ethLinkUp;
}

bool EthWiFiManager::ethernetHasIP() const
{
    return m_ethHasIp;
}

EthWiFiManager::ActiveInterface EthWiFiManager::activeInterface() const
{
    if (m_ethHasIp)
    {
        return ActiveInterface::Ethernet;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        return ActiveInterface::WiFi;
    }

    return ActiveInterface::None;
}

const char *EthWiFiManager::activeInterfaceName() const
{
    switch (activeInterface())
    {
    case ActiveInterface::Ethernet:
        return "Ethernet";
    case ActiveInterface::WiFi:
        return "WiFi";
    default:
        return "None";
    }
}

bool EthWiFiManager::initCore()
{
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(m_config.logTag, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(m_config.logTag, "esp_netif_init failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

bool EthWiFiManager::initWiFi()
{
    WiFi.mode(WIFI_STA);

    if (!m_eventsRegistered)
    {
        esp_err_t err;
        err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &EthWiFiManager::wifiEventThunk, this);
        if (err != ESP_OK)
        {
            ESP_LOGE(m_config.logTag, "register WIFI_EVENT failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &EthWiFiManager::ipEventThunk, this);
        if (err != ESP_OK)
        {
            ESP_LOGE(m_config.logTag, "register IP_EVENT_STA_GOT_IP failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &EthWiFiManager::ipEventThunk, this);
        if (err != ESP_OK)
        {
            ESP_LOGE(m_config.logTag, "register IP_EVENT_STA_LOST_IP failed: %s", esp_err_to_name(err));
            return false;
        }

        m_eventsRegistered = true;
    }

    return true;
}

bool EthWiFiManager::probeW5500()
{
    const auto &eth = m_config.ethernet;

    spi_bus_config_t busCfg = {};
    busCfg.mosi_io_num = eth.mosiPin;
    busCfg.miso_io_num = eth.misoPin;
    busCfg.sclk_io_num = eth.sckPin;
    busCfg.quadwp_io_num = GPIO_NUM_NC;
    busCfg.quadhd_io_num = GPIO_NUM_NC;

    esp_err_t err = spi_bus_initialize(eth.spiHost, &busCfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(m_config.logTag, "W5500 probe: spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t devCfg = {};
    devCfg.clock_speed_hz = eth.spiClockHz;
    devCfg.mode = 0;
    devCfg.spics_io_num = eth.csPin;
    devCfg.queue_size = 1;

    spi_device_handle_t dev;
    err = spi_bus_add_device(eth.spiHost, &devCfg, &dev);
    if (err != ESP_OK)
    {
        ESP_LOGW(m_config.logTag, "W5500 probe: spi_bus_add_device failed: %s", esp_err_to_name(err));
        spi_bus_free(eth.spiHost);
        return false;
    }

    // W5500 VERSIONR: address 0x0039, common block (BSB=0), read (RWB=0), VDM (OM=0) → control=0x00
    uint8_t tx[4] = {0x00, 0x39, 0x00, 0x00};
    uint8_t rx[4] = {};

    spi_transaction_t t = {};
    t.length = 32; // bits
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    spi_device_acquire_bus(dev, portMAX_DELAY);
    err = spi_device_transmit(dev, &t);
    spi_device_release_bus(dev);

    spi_bus_remove_device(dev);
    spi_bus_free(eth.spiHost);

    if (err != ESP_OK)
    {
        ESP_LOGW(m_config.logTag, "W5500 probe: SPI transmit failed: %s", esp_err_to_name(err));
        return false;
    }

    bool found = (rx[3] == 0x04);
    ESP_LOGI(m_config.logTag, "W5500 probe: VERSIONR=0x%02X -> %s", rx[3], found ? "found" : "not found");
    return found;
}

bool EthWiFiManager::initEthernet()
{
    esp_netif_config_t netifCfg = ESP_NETIF_DEFAULT_ETH();
    m_ethNetif = esp_netif_new(&netifCfg);
    if (m_ethNetif == nullptr)
    {
        ESP_LOGE(m_config.logTag, "esp_netif_new for ethernet failed");
        return false;
    }

    esp_err_t err;
    err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &EthWiFiManager::ethEventThunk, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(m_config.logTag, "register ETH_EVENT failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &EthWiFiManager::ipEventThunk, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(m_config.logTag, "register IP_EVENT_ETH_GOT_IP failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_bus_config_t busCfg = {};
    busCfg.mosi_io_num = m_config.ethernet.mosiPin;
    busCfg.miso_io_num = m_config.ethernet.misoPin;
    busCfg.sclk_io_num = m_config.ethernet.sckPin;
    busCfg.quadwp_io_num = GPIO_NUM_NC;
    busCfg.quadhd_io_num = GPIO_NUM_NC;

    err = spi_bus_initialize(m_config.ethernet.spiHost, &busCfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(m_config.logTag, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(m_config.logTag, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return false;
    }

    m_spiDevCfg = {};
    m_spiDevCfg.command_bits = 16;
    m_spiDevCfg.address_bits = 8;
    m_spiDevCfg.mode = 0;
    m_spiDevCfg.clock_speed_hz = m_config.ethernet.spiClockHz;
    m_spiDevCfg.spics_io_num = m_config.ethernet.csPin;
    m_spiDevCfg.queue_size = m_config.ethernet.spiQueueSize;

    err = spi_bus_add_device(m_config.ethernet.spiHost, &m_spiDevCfg, &m_spiHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(m_config.logTag, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return false;
    }

    m_w5500Cfg = ETH_W5500_DEFAULT_CONFIG(m_spiHandle);
    m_w5500Cfg.int_gpio_num = m_config.ethernet.intPin;

    eth_mac_config_t macCfg = ETH_MAC_DEFAULT_CONFIG();
    macCfg.rx_task_stack_size = m_config.ethernet.macRxTaskStackSize;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&m_w5500Cfg, &macCfg);
    if (mac == nullptr)
    {
        ESP_LOGE(m_config.logTag, "esp_eth_mac_new_w5500 failed");
        return false;
    }

    eth_phy_config_t phyCfg = ETH_PHY_DEFAULT_CONFIG();
    phyCfg.reset_gpio_num = GPIO_NUM_NC;
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phyCfg);
    if (phy == nullptr)
    {
        ESP_LOGE(m_config.logTag, "esp_eth_phy_new_w5500 failed");
        return false;
    }

    esp_eth_config_t ethCfg = ETH_DEFAULT_CONFIG(mac, phy);
    err = esp_eth_driver_install(&ethCfg, &m_ethHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(m_config.logTag, "esp_eth_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t macAddr[6] = {};
    err = esp_read_mac(macAddr, ESP_MAC_ETH);
    if (err != ESP_OK)
    {
        ESP_LOGE(m_config.logTag, "esp_read_mac failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGD(m_config.logTag, "[ETH] MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

    err = esp_eth_ioctl(m_ethHandle, ETH_CMD_S_MAC_ADDR, macAddr);
    if (err != ESP_OK)
    {
        ESP_LOGE(m_config.logTag, "esp_eth_ioctl ETH_CMD_S_MAC_ADDR failed: %s", esp_err_to_name(err));
        return false;
    }

    m_ethGlue = esp_eth_new_netif_glue(m_ethHandle);
    if (m_ethGlue == nullptr)
    {
        ESP_LOGE(m_config.logTag, "esp_eth_new_netif_glue failed");
        return false;
    }

    err = esp_netif_attach(m_ethNetif, m_ethGlue);
    if (err != ESP_OK)
    {
        ESP_LOGE(m_config.logTag, "esp_netif_attach failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGD(m_config.logTag, "[ETH] Starting driver...");
    err = esp_eth_start(m_ethHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(m_config.logTag, "esp_eth_start failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

esp_netif_t *EthWiFiManager::wifiNetif() const
{
    return esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
}

void EthWiFiManager::startWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(m_config.wifi.autoReconnect);

    if (!m_config.wifi.useDhcp)
    {
        const bool ok = WiFi.config(
            m_config.wifi.localIP,
            m_config.wifi.gateway,
            m_config.wifi.subnet,
            m_config.wifi.dns1,
            m_config.wifi.dns2);

        if (!ok)
        {
            ESP_LOGE(m_config.logTag, "[WiFi] Static IP config failed");
        }
        else
        {
            ESP_LOGI(m_config.logTag, "[WiFi] Static IP configured");
        }
    }

    ESP_LOGD(m_config.logTag, "[WiFi] Connecting to '%s'...", m_config.wifi.ssid != nullptr ? m_config.wifi.ssid : "<null>");
    WiFi.begin(m_config.wifi.ssid, m_config.wifi.password);
}

void EthWiFiManager::stopWiFi()
{
    ESP_LOGD(m_config.logTag, "[WiFi] Disconnecting (Ethernet active)");
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false, false);
}

void EthWiFiManager::onEthEvent(int32_t eventId)
{
    switch (eventId)
    {
    case ETHERNET_EVENT_CONNECTED:
        m_ethLinkUp = true;
        m_ethHasIp = false;

        if (m_config.ethernet.useDhcp)
        {
            ESP_LOGI(m_config.logTag, "[ETH] Link UP (DHCP)");
            ESP_LOGD(m_config.logTag, "[ETH] Starting DHCP...");
        }
        else
        {
            ESP_LOGI(m_config.logTag, "[ETH] Link UP (Static IP)");
        }

        if (m_ethNetif != nullptr)
        {
            esp_netif_dhcpc_stop(m_ethNetif);

            if (m_config.ethernet.useDhcp)
            {
                esp_netif_dhcpc_start(m_ethNetif);
            }
            else
            {
                esp_netif_ip_info_t ipInfo = {};
                ipInfo.ip = toEspIp4(m_config.ethernet.localIP);
                ipInfo.gw = toEspIp4(m_config.ethernet.gateway);
                ipInfo.netmask = toEspIp4(m_config.ethernet.subnet);

                esp_err_t setIpErr = esp_netif_set_ip_info(m_ethNetif, &ipInfo);
                if (setIpErr != ESP_OK)
                {
                    ESP_LOGE(m_config.logTag, "[ETH] Static IP set failed: %s", esp_err_to_name(setIpErr));
                }
                else
                {
                    esp_netif_dns_info_t dnsMain = {};
                    dnsMain.ip.u_addr.ip4 = toEspIp4(m_config.ethernet.dns1);
                    dnsMain.ip.type = ESP_IPADDR_TYPE_V4;

                    esp_netif_dns_info_t dnsBackup = {};
                    dnsBackup.ip.u_addr.ip4 = toEspIp4(m_config.ethernet.dns2);
                    dnsBackup.ip.type = ESP_IPADDR_TYPE_V4;

                    if (m_config.ethernet.dns1 != IPAddress((uint32_t)0))
                    {
                        esp_netif_set_dns_info(m_ethNetif, ESP_NETIF_DNS_MAIN, &dnsMain);
                    }
                    if (m_config.ethernet.dns2 != IPAddress((uint32_t)0))
                    {
                        esp_netif_set_dns_info(m_ethNetif, ESP_NETIF_DNS_BACKUP, &dnsBackup);
                    }

                    m_ethHasIp = true;
                    ESP_LOGI(m_config.logTag, "[ETH] Static IP applied");
                    stopWiFi();
                }
            }
        }
        break;

    case ETHERNET_EVENT_DISCONNECTED:
        m_ethLinkUp = false;
        m_ethHasIp = false;
        ESP_LOGW(m_config.logTag, "[ETH] Link DOWN");
        ESP_LOGI(m_config.logTag, "[WiFi] Fallback active");
        startWiFi();
        break;

    case ETHERNET_EVENT_START:
        ESP_LOGD(m_config.logTag, "[ETH] Driver started");
        break;

    default:
        break;
    }
}

void EthWiFiManager::onIpEvent(int32_t eventId, void *eventData)
{
    if (eventId == IP_EVENT_ETH_GOT_IP)
    {
        auto *ev = static_cast<ip_event_got_ip_t *>(eventData);
        ESP_LOGI(m_config.logTag, "[ETH] IP=" IPSTR " GW=" IPSTR " MASK=" IPSTR " route=Ethernet",
                 IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.gw), IP2STR(&ev->ip_info.netmask));
        m_ethHasIp = true;
        stopWiFi();
        return;
    }

    if (eventId == IP_EVENT_STA_GOT_IP)
    {
        auto *ev = static_cast<ip_event_got_ip_t *>(eventData);
        if (!m_ethHasIp)
        {
            ESP_LOGI(m_config.logTag, "[WiFi] IP=" IPSTR " GW=" IPSTR " MASK=" IPSTR " route=WiFi",
                     IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.gw), IP2STR(&ev->ip_info.netmask));
        }
        else
        {
            ESP_LOGD(m_config.logTag, "[WiFi] IP=" IPSTR " (Ethernet preferred)", IP2STR(&ev->ip_info.ip));
        }
        return;
    }

    if (eventId == IP_EVENT_STA_LOST_IP)
    {
        ESP_LOGW(m_config.logTag, "[WiFi] Lost IP");
    }
}

void EthWiFiManager::onWiFiEvent(int32_t eventId)
{
    if (eventId != WIFI_EVENT_STA_DISCONNECTED)
    {
        return;
    }

    if (!m_ethHasIp)
    {
        ESP_LOGW(m_config.logTag, "[WiFi] Disconnected, reconnecting...");
        esp_wifi_connect();
        return;
    }

    ESP_LOGD(m_config.logTag, "[WiFi] Disconnected (Ethernet active, no reconnect)");
}

void EthWiFiManager::ethEventThunk(void *arg, esp_event_base_t, int32_t eventId, void *)
{
    auto *self = static_cast<EthWiFiManager *>(arg);
    if (self != nullptr)
    {
        self->onEthEvent(eventId);
    }
}

void EthWiFiManager::ipEventThunk(void *arg, esp_event_base_t, int32_t eventId, void *eventData)
{
    auto *self = static_cast<EthWiFiManager *>(arg);
    if (self != nullptr)
    {
        self->onIpEvent(eventId, eventData);
    }
}

void EthWiFiManager::wifiEventThunk(void *arg, esp_event_base_t, int32_t eventId, void *)
{
    auto *self = static_cast<EthWiFiManager *>(arg);
    if (self != nullptr)
    {
        self->onWiFiEvent(eventId);
    }
}
