// main.c — ESP32-C6 + SHT45 via esp-idf-lib/sht4x
// ESPNOW Unified 25B: svc="TMP ", f1=temp[°C], f2=rh[%], f3=0, f4=0

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_log.h"
#include "esp_check.h"

// --- Sensor-Treiber (aus ESP-IDF Component Registry)
#include "i2cdev.h"
#include "sht4x.h"

// ===================== Logging =====================
static const char *TAG = "APP";

// ===================== Unified 25B ESPNOW Frame =====================
#pragma pack(push,1)
typedef struct {
    uint32_t dev_id;   // FNV-1a32 der STA-MAC
    char     svc[4];   // "FAN ", "GPS ", "TMP " ...
    int8_t   dbm;      // 0 wenn unbekannt
    float    f1;       // Bedeutungen service-spezifisch
    float    f2;
    float    f3;
    float    f4;
} espnow_frame_t;
#pragma pack(pop)
_Static_assert(sizeof(espnow_frame_t) == 25, "frame must be 25 bytes");

// Service-Code
#define SVC_TMP  { 'T','M','P',' ' }

// ===================== Device Identity =====================
static uint32_t g_dev_id32 = 0;
static uint8_t  g_mac[6]   = {0};

// ===================== Timing / ESPNOW =====================
#define BASE_PERIOD_MS 1000U
#define JITTER_MAX_MS   500U

static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static esp_now_peer_info_t peerInfo;

// ===================== I2C / SHT45 =====================
// SparkFun Qwiic auf IO6/IO7
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

static sht4x_t sht;   // Geräte-Descriptor (vom Treiber)

// Sensor init (über esp-idf-lib)
static esp_err_t sht45_init(void)
{
    // i2cdev-Helper initialisieren (setzt I²C-Pins beim Desc-Init)
    ESP_RETURN_ON_ERROR(i2cdev_init(), TAG, "i2cdev_init");

    // Descriptor auf Port 0, Pins 6/7
    ESP_RETURN_ON_ERROR(sht4x_init_desc(&sht, I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN),
                        TAG, "sht4x_init_desc");
    // Sensor initialisieren
    ESP_RETURN_ON_ERROR(sht4x_init(&sht), TAG, "sht4x_init");

    // Defaults: High Repeatability, Heater aus (kann man setzen, falls gewünscht)
    sht.repeatability = SHT4X_HIGH;        // präzise Messung
    sht.heater        = SHT4X_HEATER_OFF;  // kein Heater

    ESP_LOGI(TAG, "SHT45 ready on I2C0 SDA=%d SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
    return ESP_OK;
}

// eine Messung T/RH holen
static esp_err_t sht45_read(float *t_c, float *rh)
{
    return sht4x_measure(&sht, t_c, rh);
}

// ===================== Utils =====================
static uint32_t fnv1a32(const uint8_t *d, size_t n) {
    uint32_t h = 0x811C9DC5u;
    for (size_t i = 0; i < n; i++) { h ^= d[i]; h *= 16777619u; }
    return h;
}
static uint32_t stable_offset_ms_from_mac(void) {
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return fnv1a32(mac, 6) % (JITTER_MAX_MS + 1);
}

// ===================== WiFi / ESPNOW Init =====================
static void init_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.ampdu_tx_enable = 0;   // deterministischere Latenz
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Deutschland, explizit 1..13
    wifi_country_t country = { .cc = "DE", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));
    ESP_ERROR_CHECK(esp_wifi_start());

    // fester Kanal 1
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // 11b/g/n, 20 MHz, kein Powersave
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    uint8_t pri=0; wifi_second_chan_t sec=WIFI_SECOND_CHAN_NONE;
    ESP_ERROR_CHECK(esp_wifi_get_channel(&pri, &sec));
    ESP_LOGI(TAG, "WiFi fixed on channel %u (second=%d)", pri, (int)sec);
}

static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "ESP-NOW send status: %d", (int)status);
    }
}

static void init_espnow(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(OnDataSent));

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 1;              // 0 = current; hier fix auf 1
    peerInfo.ifidx   = WIFI_IF_STA;
    peerInfo.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));

    ESP_LOGI(TAG, "ESPNOW init done.");
}

// ===================== Task: Messung + Sendeloop =====================
static void mainloop(void *arg)
{
    (void)arg;

    // stabiler Start-Offset (Jitter aus MAC)
    uint32_t first_offset_ms = stable_offset_ms_from_mac();
    vTaskDelay(pdMS_TO_TICKS(first_offset_ms));

    for (;;) {
        float temp_c = 0.0f, rh = 0.0f;

        esp_err_t err_sht = sht45_read(&temp_c, &rh);
        if (err_sht != ESP_OK) {
            ESP_LOGW(TAG, "SHT45 read failed: %s (%d)", esp_err_to_name(err_sht), (int)err_sht);
            temp_c = 0.0f;
            rh     = 0.0f;
        }

        ESP_LOGI(TAG, "T = %.2f °C, RH = %.2f %%", temp_c, rh);

        // Unified 25B-Frame (f1=temp, f2=rh, f3=f4=0)
        espnow_frame_t pkt = {
            .dev_id = g_dev_id32,
            .svc    = SVC_TMP,   // "TMP "
            .dbm    = 0,         // Sender kennt RSSI nicht
            .f1     = temp_c,
            .f2     = rh,
            .f3     = 0.0f,
            .f4     = 0.0f,
        };

        // ESPNOW-Broadcast
        esp_err_t err = esp_now_send(broadcastAddress, (uint8_t*)&pkt, sizeof(pkt));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_send failed: %s (%d)", esp_err_to_name(err), err);
        }

        // Grundintervall + zufälliger Jitter
        uint32_t jitter_ms = esp_random() % (JITTER_MAX_MS + 1);
        vTaskDelay(pdMS_TO_TICKS(BASE_PERIOD_MS + jitter_ms));
    }
}

// ===================== app_main =====================
void app_main(void)
{
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);

    esp_log_level_set("wifi",      ESP_LOG_ERROR); // oder ESP_LOG_WARN
    esp_log_level_set("phy_init",  ESP_LOG_ERROR);
    esp_log_level_set("coexist",   ESP_LOG_ERROR);
 
    // NVS initialisieren
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Sensor/I2C
    ESP_ERROR_CHECK(sht45_init());

    // WiFi/ESPNOW

    init_wifi();
    init_espnow();

    // Device-ID (32-bit) aus MAC
    ESP_ERROR_CHECK(esp_read_mac(g_mac, ESP_MAC_WIFI_STA));
    g_dev_id32 = fnv1a32(g_mac, 6);
    ESP_LOGI(TAG, "Device ID (FNV32): 0x%08X  MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned)g_dev_id32,
             g_mac[0], g_mac[1], g_mac[2], g_mac[3], g_mac[4], g_mac[5]);

    // Task starten
    xTaskCreatePinnedToCore(mainloop, "mainloop", 4096, NULL, 4, NULL, 0);

vTaskSuspend(NULL);
}
