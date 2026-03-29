#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wifi.h"
#include "servo.h"
#include "robot.h"
#include "web_server.h"
#include "espnow_rx.h"
#include "kinematics.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=== Robot Arm Teaching System ===");

    /* 1. 连接 WiFi（阻塞直到获取 IP） */
    ESP_LOGI(TAG, "Connecting WiFi...");
    wifi_init_sta();

    /* 2. 初始化 ESP-NOW 接收 (需在 WiFi 之后) */
    ESP_LOGI(TAG, "Initializing ESP-NOW receiver...");
    espnow_rx_init();

    /* 3. 初始化舵机 UART */
    ESP_LOGI(TAG, "Initializing servos...");
    servo_init();

    /* 等待舵机上电稳定 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 4. 初始化运动学模块 */
    kin_init();

    /* 5. 初始化机械臂控制层（开启扭矩、移到Home、分配缓冲区） */
    ESP_LOGI(TAG, "Initializing robot controller...");
    robot_init();

    /* 6. 启动 Web 服务器（注册 WebSocket 推送回调） */
    ESP_LOGI(TAG, "Starting web server...");
    web_server_start();

    /* 7. 启动 50Hz servo_task */
    robot_start_task();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "System ready! Open browser to control.");
    ESP_LOGI(TAG, "========================================");

    /* 持续打印 IP 地址，方便用户查看 */
    while (1) {
        ESP_LOGI(TAG, ">>> Open http://%s in browser <<<", wifi_get_ip_str());
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
