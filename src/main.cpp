#include <Arduino.h>
#include <EthWiFiManager.h>
#include <ping/ping_sock.h>
#include <lwip/ip4_addr.h>
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
static constexpr char WIFI_SSID[] = "SSID";
#endif
#ifndef WIFI_PASS
static constexpr char WIFI_PASS[] = "PASSWORD";
#endif

static EthWiFiManager network;

static constexpr char PING_HOST[] = "192.168.5.120";
static constexpr uint32_t PING_INTERVAL_MS = 1000;
static volatile bool s_pingInProgress = false;

static void onPingSuccess(esp_ping_handle_t hdl, void *)
{
  uint32_t elapsed_ms = 0;
  esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_ms, sizeof(elapsed_ms));
  ESP_LOGW("ping", "%s: reply time=%lu ms [via %s]", PING_HOST, (unsigned long)elapsed_ms, network.activeInterfaceName());
}

static void onPingTimeout(esp_ping_handle_t hdl, void *)
{
  ESP_LOGW("ping", "%s: timeout [via %s]", PING_HOST, network.activeInterfaceName());
}

static void onPingEnd(esp_ping_handle_t hdl, void *)
{
  esp_ping_delete_session(hdl);
  s_pingInProgress = false;
}

static void startPing()
{
  if (s_pingInProgress)
    return;
  if (network.localIP() == IPAddress(0U))
    return; // no network yet

  ip_addr_t target = {};
  ip4addr_aton(PING_HOST, &target.u_addr.ip4);
  target.type = IPADDR_TYPE_V4;

  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.target_addr = target;
  cfg.count = 1;
  cfg.timeout_ms = 2000;

  esp_ping_callbacks_t cbs = {};
  cbs.on_ping_success = onPingSuccess;
  cbs.on_ping_timeout = onPingTimeout;
  cbs.on_ping_end = onPingEnd;

  esp_ping_handle_t ping;
  if (esp_ping_new_session(&cfg, &cbs, &ping) == ESP_OK)
  {
    s_pingInProgress = true;
    esp_ping_start(ping);
  }
  else
  {
    ESP_LOGE("ping", "Failed to create ping session");
  }
}

// setup
void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(1500);

  EthWiFiManager::Config config;
  config.logTag = "network";
  config.wifi.ssid = WIFI_SSID;
  config.wifi.password = WIFI_PASS;
  config.wifi.autoReconnect = true;

  config.ethernet.enabled = true;
  config.ethernet.spiHost = SPI2_HOST;
  config.ethernet.sckPin = GPIO_NUM_13;
  config.ethernet.misoPin = GPIO_NUM_12;
  config.ethernet.mosiPin = GPIO_NUM_11;
  config.ethernet.csPin = GPIO_NUM_14;
  config.ethernet.intPin = GPIO_NUM_10;

  config.ethernet.config(
      IPAddress(192, 168, 5, 138),
      IPAddress(192, 168, 5, 1),
      IPAddress(255, 255, 255, 0),
      IPAddress(1, 1, 1, 1),
      IPAddress(8, 8, 8, 8));

  config.wifi.config(
      IPAddress(192, 168, 5, 225),
      IPAddress(192, 168, 5, 1),
      IPAddress(255, 255, 255, 0),
      IPAddress(1, 1, 1, 1),
      IPAddress(8, 8, 8, 8));

  network.begin(config);
}

// loop
void loop()
{
  static uint32_t lastPingMs = 0;
  uint32_t now = millis();
  if (now - lastPingMs >= PING_INTERVAL_MS)
  {
    lastPingMs = now;
    startPing();
  }
  delay(100);
}
