#include <Arduino.h>
#include <EthWiFiManager.h>
#include <ping/ping_sock.h>
#include <lwip/ip4_addr.h>
#include <lwip/dns.h>
#include <netdb.h>
#if __has_include("wificonfig.h")
#include "wificonfig.h"
#endif

#ifndef WIFI_SSID
static constexpr char WIFI_SSID[] = "SSID";
#endif
#ifndef WIFI_PASS
static constexpr char WIFI_PASS[] = "PASSWORD";
#endif

static EthWiFiManager network;

static constexpr char PING_HOST[] = "espressif.com";
static constexpr uint32_t PING_INTERVAL_MS = 1000;
static volatile bool s_pingInProgress = false;

static void onPingSuccess(esp_ping_handle_t hdl, void *)
{
  uint32_t elapsed_ms = 0;
  esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_ms, sizeof(elapsed_ms));
  Serial.printf("%s: reply time=%lu ms [via %s]\n", PING_HOST, (unsigned long)elapsed_ms, network.activeInterfaceName());
}

static void onPingTimeout(esp_ping_handle_t hdl, void *)
{
  Serial.printf("%s: timeout [via %s]\n", PING_HOST, network.activeInterfaceName());
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

  // Resolve hostname via DNS
  struct addrinfo hints = {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_RAW;
  struct addrinfo *res = nullptr;
  int err = getaddrinfo(PING_HOST, nullptr, &hints, &res);
  if (err != 0 || res == nullptr)
  {
    Serial.printf("DNS lookup failed for %s (err=%d)\n", PING_HOST, err);
    return;
  }

  ip_addr_t target = {};
  struct sockaddr_in *sa = reinterpret_cast<struct sockaddr_in *>(res->ai_addr);
  target.u_addr.ip4.addr = sa->sin_addr.s_addr;
  target.type = IPADDR_TYPE_V4;
  freeaddrinfo(res);

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
    Serial.printf("Failed to create ping session\n");
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
