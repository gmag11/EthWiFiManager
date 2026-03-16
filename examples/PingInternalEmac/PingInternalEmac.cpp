/**
 * PingInternalEmac — Ping example using the ESP32 internal RMII EMAC
 *
 * Periodically resolves a hostname via DNS and sends a single ICMP ping,
 * reporting the round-trip time and the active interface (ETH or WiFi).
 *
 * Default pin assignment matches the WT32-ETH01 / WT32-ETH02 boards:
 *   MDC      → GPIO23
 *   MDIO     → GPIO18
 *   REF_CLK  → GPIO0  (50 MHz input from on-board oscillator via LAN8720A)
 *   PHY rst  → GPIO16
 *   PHY addr → 1
 *
 * Compatible targets: ESP32 classic ONLY (CONFIG_ETH_USE_ESP32_EMAC).
 */

#include <sdkconfig.h>
#if CONFIG_ETH_USE_ESP32_EMAC

#include <Arduino.h>
#include <EthWiFiManager.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ping/ping_sock.h>
#include <lwip/ip4_addr.h>
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

// Run DNS resolution and ping in a background task so loop() is never blocked
// by getaddrinfo() (which can take several seconds on a slow/changing network).
static void pingTask(void *)
{
    struct addrinfo hints = {};
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_RAW;
    struct addrinfo *res = nullptr;
    int err = getaddrinfo(PING_HOST, nullptr, &hints, &res);
    if (err != 0 || res == nullptr)
    {
        Serial.printf("DNS lookup failed for %s (err=%d)\n", PING_HOST, err);
        s_pingInProgress = false;
        vTaskDelete(nullptr);
        return;
    }

    ip_addr_t target = {};
    struct sockaddr_in *sa = reinterpret_cast<struct sockaddr_in *>(res->ai_addr);
    target.u_addr.ip4.addr = sa->sin_addr.s_addr;
    target.type = IPADDR_TYPE_V4;
    freeaddrinfo(res);

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr = target;
    cfg.count       = 1;
    cfg.timeout_ms  = 2000;

    esp_ping_callbacks_t cbs = {};
    cbs.on_ping_success = onPingSuccess;
    cbs.on_ping_timeout = onPingTimeout;
    cbs.on_ping_end     = onPingEnd;

    esp_ping_handle_t ping;
    if (esp_ping_new_session(&cfg, &cbs, &ping) == ESP_OK)
    {
        esp_ping_start(ping);
        // s_pingInProgress cleared by onPingEnd callback
    }
    else
    {
        Serial.printf("Failed to create ping session\n");
        s_pingInProgress = false;
    }
    vTaskDelete(nullptr);
}

static void startPing()
{
    if (s_pingInProgress)
        return;
    if (network.localIP() == IPAddress(0, 0, 0, 0))
        return; // no network yet

    s_pingInProgress = true;
    if (xTaskCreate(pingTask, "ping", 4096, nullptr, 1, nullptr) != pdPASS)
    {
        Serial.printf("Failed to create ping task\n");
        s_pingInProgress = false;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    EthWiFiManager::Config config;
    config.logTag = "PingEmac";

    config.wifi.ssid          = WIFI_SSID;
    config.wifi.password      = WIFI_PASS;
    config.wifi.autoReconnect = true;

    config.ethernet.enabled               = true;
    config.ethernet.mode                  = EthWiFiManager::EthernetMode::InternalEmac;
    config.ethernet.emacPhyChip           = EthWiFiManager::EmacPhyChip::LAN8720;
    config.ethernet.emacPhyAddr           = 1;            // SMI address — 1 on WT32-ETH01/ETH02
    config.ethernet.emacMdcPin            = GPIO_NUM_23;  // MDC
    config.ethernet.emacMdioPin           = GPIO_NUM_18;  // MDIO
    config.ethernet.emacPhyResetPin       = GPIO_NUM_16;  // Power/reset — GPIO16 on WT32-ETH01/ETH02
    config.ethernet.emacRmiiRefClkPin     = GPIO_NUM_0;   // REF_CLK input from on-board oscillator
    config.ethernet.emacRmiiClockExtInput = true;          // External 50 MHz oscillator drives REF_CLK
    config.ethernet.emacAutoNegoTimeoutMs = 1500;          // ms (IDF default: 4000)
    config.ethernet.emacLinkCheckPeriodMs = 500;           // ms (IDF default: 2000)

    network.begin(config);
}

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

#endif // CONFIG_ETH_USE_ESP32_EMAC
