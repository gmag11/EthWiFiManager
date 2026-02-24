#include <Arduino.h>
#include <WiFi.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_eth.h>
#include <esp_eth_mac.h>
#include <esp_eth_phy.h>
#include <esp_wifi.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#if __has_include("secrets.h")
#include "secrets.h"
#endif

// Pin definitions
constexpr spi_host_device_t ETH_SPI_HOST = SPI2_HOST;
constexpr gpio_num_t W5500_SCK = GPIO_NUM_13;
constexpr gpio_num_t W5500_MISO = GPIO_NUM_12;
constexpr gpio_num_t W5500_MOSI = GPIO_NUM_11;
constexpr gpio_num_t W5500_CS = GPIO_NUM_14;
constexpr gpio_num_t W5500_INT = GPIO_NUM_10;

// WiFi credentials (hardcoded)
#ifndef WIFI_SSID
static constexpr char WIFI_SSID[] = "SSID";
#endif
#ifndef WIFI_PASS
static constexpr char WIFI_PASS[] = "PASSWORD";
#endif

static constexpr char TAG[] = "network";

// State
static esp_netif_t *s_eth_netif = nullptr;
static esp_eth_handle_t s_eth_handle = nullptr;
static spi_device_handle_t s_spi_handle = nullptr;
static volatile bool s_eth_has_ip = false;
static volatile bool s_eth_link = false;

// WiFi netif lo crea el framework Arduino; lo recuperamos por ifkey
static esp_netif_t *wifi_netif()
{
  return esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
}

// Structs que el driver esp_eth referencia por puntero: deben vivir en global
static spi_device_interface_config_t s_devcfg = {};
static eth_w5500_config_t s_w5500_cfg = {};

// WiFi helpers
static void wifi_start()
{
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  ESP_LOGI(TAG, "[WiFi] Iniciando conexion a '%s'...", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static void wifi_stop()
{
  ESP_LOGI(TAG, "[WiFi] Desconectando WiFi (Ethernet activo)");
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
}

// Event handlers
static void eth_event_handler(void *, esp_event_base_t, int32_t event_id, void *)
{
  switch (event_id)
  {
  case ETHERNET_EVENT_CONNECTED:
    s_eth_link = true;
    s_eth_has_ip = false;
    ESP_LOGI(TAG, "[ETH] Link UP - lanzando DHCP...");
    esp_netif_dhcpc_stop(s_eth_netif);
    esp_netif_dhcpc_start(s_eth_netif);
    break;
  case ETHERNET_EVENT_DISCONNECTED:
    s_eth_link = false;
    s_eth_has_ip = false;
    ESP_LOGW(TAG, "[ETH] Link DOWN - activando WiFi como fallback");
    wifi_start();
    break;
  case ETHERNET_EVENT_START:
    ESP_LOGI(TAG, "[ETH] Driver iniciado");
    break;
  default:
    break;
  }
}

static void ip_event_handler(void *, esp_event_base_t, int32_t event_id, void *event_data)
{
  if (event_id == IP_EVENT_ETH_GOT_IP)
  {
    auto *ev = static_cast<ip_event_got_ip_t *>(event_data);
    ESP_LOGI(TAG, "[ETH] IP:      " IPSTR, IP2STR(&ev->ip_info.ip));
    ESP_LOGI(TAG, "[ETH] Gateway: " IPSTR, IP2STR(&ev->ip_info.gw));
    ESP_LOGI(TAG, "[ETH] Subnet:  " IPSTR, IP2STR(&ev->ip_info.netmask));
    s_eth_has_ip = true;
    wifi_stop();
  }
  else if (event_id == IP_EVENT_STA_GOT_IP)
  {
    auto *ev = static_cast<ip_event_got_ip_t *>(event_data);
    ESP_LOGI(TAG, "[WiFi] IP:      " IPSTR, IP2STR(&ev->ip_info.ip));
    ESP_LOGI(TAG, "[WiFi] Gateway: " IPSTR, IP2STR(&ev->ip_info.gw));
    ESP_LOGI(TAG, "[WiFi] Subnet:  " IPSTR, IP2STR(&ev->ip_info.netmask));
    if (!s_eth_has_ip)
    {
      ESP_LOGI(TAG, "[WiFi] Sin Ethernet -> usando WiFi");
    }
    else
    {
      ESP_LOGI(TAG, "[WiFi] Ethernet activo -> manteniendo preferencia por Ethernet");
    }
  }
  else if (event_id == IP_EVENT_STA_LOST_IP)
  {
    ESP_LOGW(TAG, "[WiFi] IP perdida");
  }
}

static void wifi_event_handler(void *, esp_event_base_t, int32_t event_id, void *)
{
  if (event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (!s_eth_has_ip)
    {
      ESP_LOGW(TAG, "[WiFi] Desconectado, reintentando...");
      esp_wifi_connect();
    }
    else
    {
      ESP_LOGI(TAG, "[WiFi] Desconectado (Ethernet activo, no se reintenta)");
    }
  }
}

// Status
static void printNetworkInfo()
{
  ESP_LOGI(TAG, "--- Estado de red ---");

  esp_netif_ip_info_t ip_eth = {};
  if (s_eth_netif)
    esp_netif_get_ip_info(s_eth_netif, &ip_eth);
  ESP_LOGI(TAG, "[ETH]  link=%-4s  ip=" IPSTR,
           s_eth_link ? "UP" : "DOWN", IP2STR(&ip_eth.ip));

  esp_netif_ip_info_t ip_wifi = {};
  if (wifi_netif())
    esp_netif_get_ip_info(wifi_netif(), &ip_wifi);
  const wl_status_t wifi_status = WiFi.status();
  const int32_t wifi_rssi = (wifi_status == WL_CONNECTED) ? WiFi.RSSI() : 0;
  ESP_LOGI(TAG, "[WiFi] rssi=%-4d  ip=" IPSTR,
           wifi_rssi, IP2STR(&ip_wifi.ip));

  const char *def_name = s_eth_has_ip ? "Ethernet" : (wifi_status == WL_CONNECTED ? "WiFi" : "ninguna");
  ESP_LOGI(TAG, "Ruta por defecto: %s", def_name);
}

// setup
void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(1500);

  ESP_LOGI(TAG, "ESP32-S3  W5500 + WiFi  failover example");

  esp_err_t err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    ESP_ERROR_CHECK(err);
  ESP_ERROR_CHECK(esp_netif_init());

  // Ethernet netif
  esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
  s_eth_netif = esp_netif_new(&netif_cfg);

  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, ip_event_handler, nullptr));

  // SPI bus
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = W5500_MOSI;
  buscfg.miso_io_num = W5500_MISO;
  buscfg.sclk_io_num = W5500_SCK;
  buscfg.quadwp_io_num = GPIO_NUM_NC;
  buscfg.quadhd_io_num = GPIO_NUM_NC;
  ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  gpio_install_isr_service(0);

  s_devcfg.command_bits = 16;
  s_devcfg.address_bits = 8;
  s_devcfg.mode = 0;
  s_devcfg.clock_speed_hz = 20 * 1000 * 1000;
  s_devcfg.spics_io_num = W5500_CS;
  s_devcfg.queue_size = 20;
  ESP_ERROR_CHECK(spi_bus_add_device(ETH_SPI_HOST, &s_devcfg, &s_spi_handle));

  s_w5500_cfg = ETH_W5500_DEFAULT_CONFIG(s_spi_handle);
  s_w5500_cfg.int_gpio_num = W5500_INT;

  eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
  mac_cfg.rx_task_stack_size = 4096;
  esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&s_w5500_cfg, &mac_cfg);

  eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
  phy_cfg.reset_gpio_num = GPIO_NUM_NC;
  esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);

  esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
  ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &s_eth_handle));

  uint8_t mac_addr[6];
  ESP_ERROR_CHECK(esp_read_mac(mac_addr, ESP_MAC_ETH));
  ESP_LOGI(TAG, "[ETH] MAC: %02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);
  ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));

  esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_eth_handle);
  ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));

  ESP_LOGI(TAG, "[ETH] Iniciando driver...");
  ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

  // WiFi fallback: WiFi.mode() crea el netif STA en el framework Arduino
  WiFi.mode(WIFI_STA);
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event_handler, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, ip_event_handler, nullptr));

  // Arrancar WiFi; si Ethernet llega antes, el handler lo apagara
  wifi_start();
}

// loop
void loop()
{
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 5000)
  {
    lastPrint = millis();
    printNetworkInfo();
  }
}
