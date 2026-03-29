#include "espnow_rx.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "ESPNOW_RX";

static imu_packet_t s_latest_pkt;
static int64_t      s_last_rx_time = 0;
static bool         s_has_new = false;
static SemaphoreHandle_t s_rx_mutex = NULL;

/* 校验和: 前 17 字节 XOR */
static uint8_t calc_checksum(const uint8_t *data, int len)
{
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

/* ESP-NOW 接收回调 (在 WiFi 任务上下文中调用) */
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int data_len)
{
    ESP_LOGD(TAG, "RX: %d bytes", data_len);
    
    if (data_len != sizeof(imu_packet_t)) {
        ESP_LOGW(TAG, "Bad len: %d (exp %d)", data_len, (int)sizeof(imu_packet_t));
        return;
    }

    const imu_packet_t *pkt = (const imu_packet_t *)data;

    /* 校验 magic */
    if (pkt->magic != IMU_PKT_MAGIC) {
        ESP_LOGW(TAG, "Bad magic: 0x%02X", pkt->magic);
        return;
    }

    /* 校验 checksum */
    uint8_t cs = calc_checksum(data, sizeof(imu_packet_t) - 1);
    if (cs != pkt->checksum) {
        ESP_LOGW(TAG, "Bad CS: 0x%02X (exp 0x%02X)", cs, pkt->checksum);
        return;
    }

    ESP_LOGD(TAG, "OK: roll=%.3f pitch=%.3f yaw=%.3f trig=%d",
             (double)pkt->roll, (double)pkt->pitch, (double)pkt->yaw, pkt->trigger);

    /* 写入共享缓冲区 */
    xSemaphoreTake(s_rx_mutex, portMAX_DELAY);
    memcpy(&s_latest_pkt, pkt, sizeof(imu_packet_t));
    s_last_rx_time = esp_timer_get_time();
    s_has_new = true;
    xSemaphoreGive(s_rx_mutex);
}

void espnow_rx_init(void)
{
    s_rx_mutex = xSemaphoreCreateMutex();
    configASSERT(s_rx_mutex);

    /* 打印当前 WiFi 信道供控制器端参考 */
    uint8_t primary;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    ESP_LOGI(TAG, "WiFi channel: %d (controller must use same channel)", primary);

    /* 初始化 ESP-NOW */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "ESP-NOW receiver initialized");
}

bool espnow_rx_get_latest(imu_packet_t *pkt, int64_t *timestamp)
{
    xSemaphoreTake(s_rx_mutex, portMAX_DELAY);
    bool has = s_has_new;
    if (has) {
        memcpy(pkt, &s_latest_pkt, sizeof(imu_packet_t));
        if (timestamp) *timestamp = s_last_rx_time;
        s_has_new = false;
    }
    xSemaphoreGive(s_rx_mutex);
    return has;
}

bool espnow_rx_is_connected(void)
{
    xSemaphoreTake(s_rx_mutex, portMAX_DELAY);
    int64_t last = s_last_rx_time;
    xSemaphoreGive(s_rx_mutex);

    if (last == 0) return false;

    int64_t now = esp_timer_get_time();
    return (now - last) < 500000;  /* 500ms = 500000 us */
}
