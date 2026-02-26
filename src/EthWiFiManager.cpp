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
        bool ethernetOk = true;
#if ETHWIFI_INTERNAL_EMAC
        if (m_config.ethernet.mode == EthernetMode::Spi)
#endif
        {
            ethernetOk = probeSpiModule();
            if (!ethernetOk)
            {
                ESP_LOGW(m_config.logTag, "SPI Ethernet module not detected - Ethernet disabled, WiFi only");
                m_config.ethernet.enabled = false;
            }
        }

        if (ethernetOk && !initEthernet())
        {
            return false;
        }
    }

    startWiFi();
    ESP_LOGI(m_config.logTag, "Manager started (ethernet=%s)", m_config.ethernet.enabled ? "enabled" : "disabled");
    m_started = true;
    return true;
}

bool EthWiFiManager::beginApRouter(const ApRouterConfig &config)
{
    if (m_started)
    {
        return true;
    }

    m_apRouterConfig = config;
    m_config.logTag = (config.logTag != nullptr) ? config.logTag : "EthWiFiManager";
    m_config.ethernet = config.ethernet;
    s_instance = this;
    m_apRouterMode = true;

    if (!initCore())
    {
        return false;
    }

    if (m_config.ethernet.enabled)
    {
        bool ethernetOk = true;
#if ETHWIFI_INTERNAL_EMAC
        if (m_config.ethernet.mode == EthernetMode::Spi)
#endif
        {
            ethernetOk = probeSpiModule();
            if (!ethernetOk)
            {
                ESP_LOGW(m_config.logTag, "SPI Ethernet module not detected - Ethernet disabled");
                m_config.ethernet.enabled = false;
            }
        }

        if (ethernetOk && !initEthernet())
        {
            return false;
        }
    }

    if (!initApRouter())
    {
        return false;
    }

    ESP_LOGI(m_config.logTag, "AP Router started (ethernet=%s)", m_config.ethernet.enabled ? "enabled" : "disabled");
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

bool EthWiFiManager::initApRouter()
{
    const auto &cfg = m_apRouterConfig;

    // Configure AP IP addressing and DHCP server range before starting the AP.
    // softAPConfig stops/restarts the DHCP server with the new address range.
    if (!WiFi.softAPConfig(cfg.apLocalIP, cfg.apGateway, cfg.apSubnet))
    {
        ESP_LOGE(m_config.logTag, "[AP] softAPConfig failed");
        return false;
    }

    // Start the WiFi AP (initialises the WiFi driver, sets AP mode, applies config).
    const bool hasPassword = (cfg.apPassword != nullptr && strlen(cfg.apPassword) >= 8);
    const bool started = WiFi.softAP(
        cfg.apSsid,
        hasPassword ? cfg.apPassword : nullptr,
        cfg.apChannel,
        0,                      // ssid_hidden = 0 (broadcast)
        cfg.apMaxConnections);

    if (!started)
    {
        ESP_LOGE(m_config.logTag, "[AP] WiFi.softAP failed");
        return false;
    }

    esp_ip4_addr_t apIp = toEspIp4(cfg.apLocalIP);
    ESP_LOGI(m_config.logTag, "[AP] Started: SSID=%s IP=" IPSTR " auth=%s",
             cfg.apSsid, IP2STR(&apIp), hasPassword ? "WPA2-PSK" : "open");

#if defined(CONFIG_LWIP_IP_NAPT) && CONFIG_LWIP_IP_NAPT
    // Enable NAPT on the AP interface so clients reach the internet via Ethernet.
    ip_napt_enable(apIp.addr, 1);
    ESP_LOGI(m_config.logTag, "[AP] NAPT enabled — AP clients routed via Ethernet IP");
#else
    ESP_LOGW(m_config.logTag,
             "[AP] NAPT not available — add CONFIG_LWIP_IP_FORWARD=y and "
             "CONFIG_LWIP_IP_NAPT=y to sdkconfig (or board_build.cmake_extra_args)");
#endif

    return true;
}

bool EthWiFiManager::probeSpiModule()
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
        ESP_LOGW(m_config.logTag, "SPI probe: spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    // Probe uses raw bytes (no command/address phase separation) so all chips can be read uniformly
    spi_device_interface_config_t devCfg = {};
    devCfg.clock_speed_hz = eth.spiClockHz;
    devCfg.mode = 0;
    devCfg.spics_io_num = eth.csPin;
    devCfg.queue_size = 1;

    spi_device_handle_t dev;
    err = spi_bus_add_device(eth.spiHost, &devCfg, &dev);
    if (err != ESP_OK)
    {
        ESP_LOGW(m_config.logTag, "SPI probe: spi_bus_add_device failed: %s", esp_err_to_name(err));
        spi_bus_free(eth.spiHost);
        return false;
    }

    uint8_t tx[4] = {};
    uint8_t rx[4] = {};
    bool found = false;

    spi_transaction_t t = {};
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    switch (eth.spiModule)
    {
#if ETHWIFI_DM9051
    case SpiModule::DM9051:
        // Read PIDL register (0x28). DM9051 SPI frame: bit7=R/W, bits[6:0]=addr
        // Read command: 0x80 | 0x28 = 0xA8. Response: rx[1] = PIDL (expected 0x51)
        tx[0] = 0xA8;
        t.length = 16;
        spi_device_acquire_bus(dev, portMAX_DELAY);
        err = spi_device_transmit(dev, &t);
        spi_device_release_bus(dev);
        if (err == ESP_OK)
        {
            found = (rx[1] == 0x51);
            ESP_LOGI(m_config.logTag, "DM9051 probe: PIDL=0x%02X -> %s", rx[1], found ? "found" : "not found");
        }
        break;
#endif // ETHWIFI_DM9051
#if ETHWIFI_KSZ8851SNL
    case SpiModule::KSZ8851SNL:
        // Read CIDER register (0xC0): 16-bit header = SOP(read)=0b10 | BE=0b1111 | addr=0xC0>>2=0x30
        // Header = 0b10_1111_0011_0000_00 = 0xBCC0
        // Expected CIDER = 0x8872: rx[2]=0x88, rx[3]=0x72 (or rev variant 0x7x)
        tx[0] = 0xBC; tx[1] = 0xC0;
        t.length = 32;
        spi_device_acquire_bus(dev, portMAX_DELAY);
        err = spi_device_transmit(dev, &t);
        spi_device_release_bus(dev);
        if (err == ESP_OK)
        {
            found = (rx[2] == 0x88 && (rx[3] & 0xF0) == 0x70);
            ESP_LOGI(m_config.logTag, "KSZ8851SNL probe: CIDER=0x%02X%02X -> %s",
                     rx[2], rx[3], found ? "found" : "not found");
        }
        break;
#endif // ETHWIFI_KSZ8851SNL
#if ETHWIFI_W5500
    case SpiModule::W5500:
        // Read VERSIONR: address 0x0039, common block (BSB=0), read (RWB=0), VDM → control=0x00
        // Expected value: 0x04
        tx[0] = 0x00; tx[1] = 0x39; tx[2] = 0x00; tx[3] = 0x00;
        t.length = 32;
        spi_device_acquire_bus(dev, portMAX_DELAY);
        err = spi_device_transmit(dev, &t);
        spi_device_release_bus(dev);
        if (err == ESP_OK)
        {
            found = (rx[3] == 0x04);
            ESP_LOGI(m_config.logTag, "W5500 probe: VERSIONR=0x%02X -> %s", rx[3], found ? "found" : "not found");
        }
        break;
#endif // ETHWIFI_W5500
    default:
        ESP_LOGE(m_config.logTag, "Selected SPI module is not compiled in (check ETHWIFI_NO_* build flags)");
        break;
    }

    if (err != ESP_OK)
    {
        ESP_LOGW(m_config.logTag, "SPI probe: transmit failed: %s", esp_err_to_name(err));
    }

    spi_bus_remove_device(dev);
    spi_bus_free(eth.spiHost);
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

    esp_eth_mac_t *mac = nullptr;
    esp_eth_phy_t *phy = nullptr;

#if ETHWIFI_INTERNAL_EMAC
    if (m_config.ethernet.mode == EthernetMode::InternalEmac)
    {
        // ---- Internal RMII EMAC (ESP32 classic) ----
        // eth_mac_config_t holds MDC/MDIO pins and RMII clock config for ESP32 internal EMAC
        eth_mac_config_t macCfg = ETH_MAC_DEFAULT_CONFIG();
        macCfg.smi_mdc_gpio_num   = m_config.ethernet.emacMdcPin;
        macCfg.smi_mdio_gpio_num  = m_config.ethernet.emacMdioPin;
        macCfg.rx_task_stack_size = m_config.ethernet.macRxTaskStackSize;
        macCfg.clock_config.rmii.clock_mode =
            m_config.ethernet.emacRmiiClockExtInput ? EMAC_CLK_EXT_IN : EMAC_CLK_OUT;
        macCfg.clock_config.rmii.clock_gpio =
            (emac_rmii_clock_gpio_t)(int)m_config.ethernet.emacRmiiRefClkPin;
        mac = esp_eth_mac_new_esp32(&macCfg);

        eth_phy_config_t phyCfg = ETH_PHY_DEFAULT_CONFIG();
        phyCfg.phy_addr       = m_config.ethernet.emacPhyAddr;
        phyCfg.reset_gpio_num = m_config.ethernet.emacPhyResetPin;

        switch (m_config.ethernet.emacPhyChip)
        {
        case EmacPhyChip::IP101:   phy = esp_eth_phy_new_ip101(&phyCfg);   break;
        case EmacPhyChip::RTL8201: phy = esp_eth_phy_new_rtl8201(&phyCfg); break;
        case EmacPhyChip::DP83848: phy = esp_eth_phy_new_dp83848(&phyCfg); break;
        case EmacPhyChip::KSZ8041: phy = esp_eth_phy_new_ksz8041(&phyCfg); break;
        case EmacPhyChip::KSZ8081: phy = esp_eth_phy_new_ksz8081(&phyCfg); break;
        case EmacPhyChip::LAN8720:
        default:                   phy = esp_eth_phy_new_lan87xx(&phyCfg); break;
        }

        if (mac == nullptr || phy == nullptr)
        {
            ESP_LOGE(m_config.logTag, "EMAC MAC/PHY init failed");
            return false;
        }
    }
    else
#endif // ETHWIFI_INTERNAL_EMAC
    {
        // ---- SPI Ethernet module ----
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
        m_spiDevCfg.mode = 0;
        m_spiDevCfg.clock_speed_hz = m_config.ethernet.spiClockHz;
        m_spiDevCfg.spics_io_num   = m_config.ethernet.csPin;
        m_spiDevCfg.queue_size     = m_config.ethernet.spiQueueSize;

        // Each chip uses different SPI frame encoding (command/address phase layout)
        switch (m_config.ethernet.spiModule)
        {
#if ETHWIFI_DM9051
        case SpiModule::DM9051:
            m_spiDevCfg.command_bits = 1;
            m_spiDevCfg.address_bits = 7;
            break;
#endif
#if ETHWIFI_KSZ8851SNL
        case SpiModule::KSZ8851SNL:
            m_spiDevCfg.command_bits = 16;
            m_spiDevCfg.address_bits = 0;
            break;
#endif
#if ETHWIFI_W5500
        case SpiModule::W5500:
            m_spiDevCfg.command_bits = 16;
            m_spiDevCfg.address_bits = 8;
            break;
#endif
        default:
            ESP_LOGE(m_config.logTag, "Selected SPI module is not compiled in (check ETHWIFI_NO_* build flags)");
            return false;
        }

        err = spi_bus_add_device(m_config.ethernet.spiHost, &m_spiDevCfg, &m_spiHandle);
        if (err != ESP_OK)
        {
            ESP_LOGE(m_config.logTag, "spi_bus_add_device failed: %s", esp_err_to_name(err));
            return false;
        }

        eth_mac_config_t macCfg = ETH_MAC_DEFAULT_CONFIG();
        macCfg.rx_task_stack_size = m_config.ethernet.macRxTaskStackSize;
        eth_phy_config_t phyCfg = ETH_PHY_DEFAULT_CONFIG();
        phyCfg.reset_gpio_num = GPIO_NUM_NC;

        switch (m_config.ethernet.spiModule)
        {
#if ETHWIFI_DM9051
        case SpiModule::DM9051:
        {
            eth_dm9051_config_t dm9051Cfg = ETH_DM9051_DEFAULT_CONFIG(m_spiHandle);
            dm9051Cfg.int_gpio_num = m_config.ethernet.intPin;
            mac = esp_eth_mac_new_dm9051(&dm9051Cfg, &macCfg);
            phy = esp_eth_phy_new_dm9051(&phyCfg);
            if (mac == nullptr || phy == nullptr)
                ESP_LOGE(m_config.logTag, "DM9051 MAC/PHY init failed");
            break;
        }
#endif // ETHWIFI_DM9051
#if ETHWIFI_KSZ8851SNL
        case SpiModule::KSZ8851SNL:
        {
            eth_ksz8851snl_config_t kszCfg = ETH_KSZ8851SNL_DEFAULT_CONFIG(m_spiHandle);
            kszCfg.int_gpio_num = m_config.ethernet.intPin;
            mac = esp_eth_mac_new_ksz8851snl(&kszCfg, &macCfg);
            phy = esp_eth_phy_new_ksz8851snl(&phyCfg);
            if (mac == nullptr || phy == nullptr)
                ESP_LOGE(m_config.logTag, "KSZ8851SNL MAC/PHY init failed");
            break;
        }
#endif // ETHWIFI_KSZ8851SNL
#if ETHWIFI_W5500
        case SpiModule::W5500:
        {
            eth_w5500_config_t w5500Cfg = ETH_W5500_DEFAULT_CONFIG(m_spiHandle);
            w5500Cfg.int_gpio_num = m_config.ethernet.intPin;
            mac = esp_eth_mac_new_w5500(&w5500Cfg, &macCfg);
            phy = esp_eth_phy_new_w5500(&phyCfg);
            if (mac == nullptr || phy == nullptr)
                ESP_LOGE(m_config.logTag, "W5500 MAC/PHY init failed");
            break;
        }
#endif // ETHWIFI_W5500
        default:
            ESP_LOGE(m_config.logTag, "Selected SPI module is not compiled in (check ETHWIFI_NO_* build flags)");
            break;
        }

        if (mac == nullptr || phy == nullptr)
        {
            return false;
        }
    } // end SPI block

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
                    if (!m_apRouterMode)
                    {
                        stopWiFi();
                    }
                }
            }
        }
        break;

    case ETHERNET_EVENT_DISCONNECTED:
        m_ethLinkUp = false;
        m_ethHasIp = false;
        ESP_LOGW(m_config.logTag, "[ETH] Link DOWN");
        if (!m_apRouterMode)
        {
            ESP_LOGI(m_config.logTag, "[WiFi] Fallback active");
            startWiFi();
        }
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
        if (!m_apRouterMode)
        {
            stopWiFi();
        }
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
