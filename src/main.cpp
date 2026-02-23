#include <Arduino.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_eth.h>
#include <esp_eth_mac_spi.h>
#include <esp_eth_phy.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
constexpr spi_host_device_t ETH_SPI_HOST = SPI2_HOST;
constexpr gpio_num_t W5500_SCK = GPIO_NUM_13;
constexpr gpio_num_t W5500_MISO = GPIO_NUM_12;
constexpr gpio_num_t W5500_MOSI = GPIO_NUM_11;
constexpr gpio_num_t W5500_CS = GPIO_NUM_14;
constexpr gpio_num_t W5500_INT = GPIO_NUM_10; // sin pin de interrupción

static constexpr char TAG[] = "ethernet";

// ── Network config ───────────────────────────────────────────────────────────
static uint8_t macAddress[] = {0x02, 0x32, 0x53, 0x55, 0x00, 0x01};

static esp_netif_t        *s_eth_netif  = nullptr;
static esp_eth_handle_t    s_eth_handle = nullptr;
static volatile bool       s_got_ip     = false;
static volatile bool       s_link_up    = false;

// Structs que el driver esp_eth referencia por puntero — deben vivir en global
static spi_device_interface_config_t s_devcfg  = {};
static eth_w5500_config_t            s_w5500_cfg = {};

// ── Event handlers ───────────────────────────────────────────────────────────
static void eth_event_handler(void *, esp_event_base_t, int32_t event_id, void *)
{
  switch (event_id)
  {
  case ETHERNET_EVENT_CONNECTED:
    s_link_up = true;
    ESP_LOGI(TAG, "Link: UP");
    break;
  case ETHERNET_EVENT_DISCONNECTED:
    s_link_up = false;
    s_got_ip = false;
    ESP_LOGI(TAG, "Link: DOWN");
    break;
  case ETHERNET_EVENT_START:
    ESP_LOGI(TAG, "Ethernet started");
    break;
  default:
    break;
  }
}

static void ip_event_handler(void *, esp_event_base_t, int32_t event_id, void *event_data)
{
  if (event_id == IP_EVENT_ETH_GOT_IP)
  {
    auto *event = static_cast<ip_event_got_ip_t *>(event_data);
    ESP_LOGI(TAG, "IP:      " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
    ESP_LOGI(TAG, "Subnet:  " IPSTR, IP2STR(&event->ip_info.netmask));
    s_got_ip = true;
  }
}

// ── Helpers ──────────────────────────────────────────────────────────────────
static void printNetworkInfo()
{
  ESP_LOGI(TAG, "--- Ethernet status ---");
  ESP_LOGI(TAG, "Link: %s", s_link_up ? "UP" : "DOWN");
  if (s_eth_netif)
  {
    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(s_eth_netif, &ip);
    ESP_LOGI(TAG, "IP:      " IPSTR, IP2STR(&ip.ip));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&ip.gw));
    ESP_LOGI(TAG, "Subnet:  " IPSTR, IP2STR(&ip.netmask));
  }
}

// ── Arduino entry points ─────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(1500);

  ESP_LOGI(TAG, "ESP32-S3 + W5500 (esp_eth) example");

  // Crear event loop (ignorar si ya existe)
  esp_err_t err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    ESP_ERROR_CHECK(err);

  // Inicializar stack TCP/IP (LwIP)
  ESP_ERROR_CHECK(esp_netif_init());

  // Crear netif Ethernet
  esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
  s_eth_netif = esp_netif_new(&netif_cfg);

  // Registrar handlers
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, ip_event_handler, nullptr));

  // Bus SPI
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = W5500_MOSI;
  buscfg.miso_io_num = W5500_MISO;
  buscfg.sclk_io_num = W5500_SCK;
  buscfg.quadwp_io_num = GPIO_NUM_NC;
  buscfg.quadhd_io_num = GPIO_NUM_NC;
  ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // Instalar servicio ISR de GPIO (necesario para el pin INT del W5500)
  gpio_install_isr_service(0);

  // Dispositivo SPI del W5500
  s_devcfg.command_bits   = 16;
  s_devcfg.address_bits   = 8;
  s_devcfg.mode           = 0;
  s_devcfg.clock_speed_hz = 20 * 1000 * 1000; // 20 MHz
  s_devcfg.spics_io_num   = W5500_CS;
  s_devcfg.queue_size     = 20;

  // Configuración MAC W5500
  // int_gpio_num y poll_period_ms son mutuamente excluyentes:
  //   int_gpio_num >= 0  →  modo interrupción  (poll_period_ms debe ser 0)
  //   int_gpio_num == -1 →  modo polling       (poll_period_ms > 0)
  s_w5500_cfg.spi_host_id     = ETH_SPI_HOST;
  s_w5500_cfg.spi_devcfg      = &s_devcfg;
  s_w5500_cfg.int_gpio_num    = W5500_INT;
  s_w5500_cfg.poll_period_ms  = (W5500_INT == GPIO_NUM_NC) ? 10 : 0;
  s_w5500_cfg.custom_spi_driver = ETH_DEFAULT_SPI;

  eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
  mac_cfg.rx_task_stack_size = 4096;
  esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&s_w5500_cfg, &mac_cfg);

  // Crear PHY W5500 (integrado)
  eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
  phy_cfg.reset_gpio_num = GPIO_NUM_NC;
  esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);

  // Instalar driver Ethernet
  esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
  ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &s_eth_handle));

  // Fijar dirección MAC
  ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, macAddress));

  // Conectar netif al driver
  esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_eth_handle);
  ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));

  // Arrancar Ethernet
  ESP_LOGI(TAG, "Iniciando Ethernet...");
  ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

  // Esperar IP por DHCP (10 s)
  ESP_LOGI(TAG, "Esperando dirección DHCP...");
  uint32_t t0 = millis();
  while (!s_got_ip && (millis() - t0) < 10000)
    delay(100);

  if (!s_got_ip)
  {
    ESP_LOGW(TAG, "DHCP timeout, aplicando IP estática de fallback");
    esp_netif_dhcpc_stop(s_eth_netif);
    esp_netif_ip_info_t fallback = {};
    IP4_ADDR(&fallback.ip, 192, 168, 1, 180);
    IP4_ADDR(&fallback.netmask, 255, 255, 255, 0);
    IP4_ADDR(&fallback.gw, 192, 168, 1, 1);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &fallback));
  }

  printNetworkInfo();
}

void loop()
{
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 5000)
  {
    lastStatusPrint = millis();
    printNetworkInfo();
  }
}