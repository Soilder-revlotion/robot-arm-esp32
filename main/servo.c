#include "servo.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "SERVO";

#define SERVO_UART_NUM   UART_NUM_1
#define SERVO_TX_PIN     17          /* ESP32 TX → 调试板 RXD */
#define SERVO_RX_PIN     18          /* ESP32 RX ← 调试板 TXD */
#define BUF_SIZE         256

/* 协议常量 */
#define HEADER           0xFF
#define INST_READ        0x02
#define INST_WRITE       0x03
#define INST_PING        0x01

/* STS3215 寄存器地址 */
#define REG_TORQUE_LIMIT      34
#define REG_TORQUE_ENABLE     40
#define REG_GOAL_POSITION     42
#define REG_GOAL_SPEED        46
#define REG_PRESENT_POSITION  56

/* 通信参数 */
#define READ_RETRY_COUNT      2     /* 读取失败重试次数 */
#define INTER_CMD_DELAY_US    150   /* 命令间隔(微秒), 半双工总线切换 */
#define READ_TIMEOUT_MS       20    /* 读取超时(毫秒) */

/* 当前工作波特率 */
static int s_baud_rate = 1000000;

/* 诊断模式: 前几次读取打印原始数据 */
static int s_diag_count = 30;

/* 上次命令发送时间 */
static int64_t s_last_cmd_time_us = 0;

/* 计算校验和: ~(sum of bytes[2..len-1]) */
static uint8_t calc_checksum(const uint8_t *buf, int len)
{
    uint8_t sum = 0;
    for (int i = 2; i < len; i++) {
        sum += buf[i];
    }
    return ~sum;
}

/* 打印十六进制原始数据 */
static void print_hex(const char *prefix, const uint8_t *buf, int len)
{
    char hex[128];
    int pos = 0;
    for (int i = 0; i < len && pos < 120; i++) {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[i]);
    }
    if (len == 0) {
        snprintf(hex, sizeof(hex), "(empty)");
    }
    ESP_LOGI(TAG, "%s [%d bytes]: %s", prefix, len, hex);
}

/* 确保命令间隔, 让半双工总线有时间切换方向 */
static void bus_delay(void)
{
    int64_t now = esp_timer_get_time();
    int64_t elapsed = now - s_last_cmd_time_us;
    if (elapsed < INTER_CMD_DELAY_US) {
        esp_rom_delay_us((uint32_t)(INTER_CMD_DELAY_US - elapsed));
    }
}

/* 发送后记录时间, 并排空残余数据 */
static void post_write_drain(void)
{
    s_last_cmd_time_us = esp_timer_get_time();
    /* 排空写命令可能产生的舵机应答(写指令默认也会回复) */
    uint8_t drain[16];
    uart_read_bytes(SERVO_UART_NUM, drain, sizeof(drain), pdMS_TO_TICKS(2));
}

/* 尝试用指定波特率 ping 舵机ID 1 */
static bool try_ping_at_baud(int baud)
{
    /* 重新配置波特率 */
    uart_set_baudrate(SERVO_UART_NUM, baud);
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_flush_input(SERVO_UART_NUM);

    /* 发送 PING 包 */
    uint8_t pkt[6];
    pkt[0] = HEADER;
    pkt[1] = HEADER;
    pkt[2] = 1;       /* ID 1 */
    pkt[3] = 2;       /* len: inst + checksum */
    pkt[4] = INST_PING;
    pkt[5] = calc_checksum(pkt, 5);

    uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, 6);
    uart_wait_tx_done(SERVO_UART_NUM, pdMS_TO_TICKS(10));

    /* 读取回复（可能有回显） */
    uint8_t resp[32];
    int len = uart_read_bytes(SERVO_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "  Ping ID=1 @ %d bps -> got %d bytes", baud, len);
    if (len > 0) {
        print_hex("  RAW", resp, len);
    }

    /* 在回复中搜索有效的舵机应答: FF FF 01 02 00 xx */
    for (int i = 0; i + 5 < len; i++) {
        if (resp[i] == 0xFF && resp[i+1] == 0xFF && resp[i+2] == 0x01) {
            ESP_LOGI(TAG, "  => Found servo response at offset %d!", i);
            return true;
        }
    }
    /* 也检查是否收到了任何非零字节（至少有通信） */
    if (len > 0) {
        ESP_LOGW(TAG, "  => Got bytes but no valid servo header at %d bps", baud);
    }

    return false;
}

void servo_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(SERVO_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(SERVO_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART%d init OK (TX=%d, RX=%d)", SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN);

    /* ====== 自动检测波特率 ====== */
    static const int bauds[] = { 1000000, 500000, 115200 };
    static const int num_bauds = sizeof(bauds) / sizeof(bauds[0]);

    ESP_LOGI(TAG, "=== Auto-detecting servo baud rate ===");
    bool found = false;
    for (int b = 0; b < num_bauds; b++) {
        ESP_LOGI(TAG, "Trying %d bps ...", bauds[b]);
        if (try_ping_at_baud(bauds[b])) {
            s_baud_rate = bauds[b];
            ESP_LOGI(TAG, "*** Servo found at %d bps! ***", s_baud_rate);
            found = true;
            break;
        }
    }

    if (!found) {
        s_baud_rate = 1000000;
        uart_set_baudrate(SERVO_UART_NUM, s_baud_rate);
        ESP_LOGW(TAG, "No servo detected at any baud rate. Using default %d bps.", s_baud_rate);
        ESP_LOGW(TAG, "Check wiring: ESP32 GPIO17(TX)->Board RXD, GPIO18(RX)<-Board TXD, GND<->GND");
        ESP_LOGW(TAG, "If still failing, try swapping TX/RX wires.");
    }

    ESP_LOGI(TAG, "Servo UART ready at %d bps", s_baud_rate);
}

int16_t servo_read_pos(uint8_t id)
{
    uint8_t pkt[8];
    pkt[0] = HEADER;
    pkt[1] = HEADER;
    pkt[2] = id;
    pkt[3] = 4;            /* len: inst + addr + count + checksum */
    pkt[4] = INST_READ;
    pkt[5] = REG_PRESENT_POSITION;
    pkt[6] = 2;            /* 读 2 字节 */
    pkt[7] = calc_checksum(pkt, 7);

    for (int attempt = 0; attempt <= READ_RETRY_COUNT; attempt++) {
        bus_delay();
        uart_flush_input(SERVO_UART_NUM);

        uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, 8);
        uart_wait_tx_done(SERVO_UART_NUM, pdMS_TO_TICKS(10));

        /* 读取所有回复（可能含回显+应答） */
        uint8_t resp[32];
        int len = uart_read_bytes(SERVO_UART_NUM, resp, sizeof(resp),
                                  pdMS_TO_TICKS(READ_TIMEOUT_MS));
        s_last_cmd_time_us = esp_timer_get_time();

        /* 诊断: 打印前几次的原始数据 */
        if (s_diag_count > 0) {
            s_diag_count--;
            char label[32];
            snprintf(label, sizeof(label), "RX ID=%d try=%d", id, attempt);
            print_hex(label, resp, len);
        }

        /* 在回复中搜索有效的舵机应答帧 */
        for (int i = 0; i + 7 <= len; i++) {
            if (resp[i] == HEADER && resp[i+1] == HEADER && resp[i+2] == id) {
                uint8_t resp_len = resp[i+3];
                if (resp_len >= 4 && i + 3 + resp_len <= len) {
                    int16_t pos = resp[i+5] | (resp[i+6] << 8);
                    return pos;
                }
            }
        }

        /* 重试前短暂等待 */
        if (attempt < READ_RETRY_COUNT) {
            esp_rom_delay_us(200);
        }
    }

    return -1;
}

void servo_write_pos(uint8_t id, int16_t position, uint16_t speed)
{
    bus_delay();

    uint8_t pkt[13];
    pkt[0]  = HEADER;
    pkt[1]  = HEADER;
    pkt[2]  = id;
    pkt[3]  = 9;           /* len: inst(1)+addr(1)+data(6)+chk(1) = 9 */
    pkt[4]  = INST_WRITE;
    pkt[5]  = REG_GOAL_POSITION;
    pkt[6]  = position & 0xFF;
    pkt[7]  = (position >> 8) & 0xFF;
    pkt[8]  = 0;           /* 时间 low */
    pkt[9]  = 0;           /* 时间 high */
    pkt[10] = speed & 0xFF;
    pkt[11] = (speed >> 8) & 0xFF;
    pkt[12] = calc_checksum(pkt, 12);

    uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, 13);
    uart_wait_tx_done(SERVO_UART_NUM, pdMS_TO_TICKS(10));
    post_write_drain();
}

void servo_torque_off(uint8_t id)
{
    bus_delay();

    uint8_t pkt[8];
    pkt[0] = HEADER;
    pkt[1] = HEADER;
    pkt[2] = id;
    pkt[3] = 4;
    pkt[4] = INST_WRITE;
    pkt[5] = REG_TORQUE_ENABLE;
    pkt[6] = 0;
    pkt[7] = calc_checksum(pkt, 7);

    uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, 8);
    uart_wait_tx_done(SERVO_UART_NUM, pdMS_TO_TICKS(10));
    post_write_drain();
}

void servo_torque_on(uint8_t id)
{
    bus_delay();

    uint8_t pkt[8];
    pkt[0] = HEADER;
    pkt[1] = HEADER;
    pkt[2] = id;
    pkt[3] = 4;
    pkt[4] = INST_WRITE;
    pkt[5] = REG_TORQUE_ENABLE;
    pkt[6] = 1;
    pkt[7] = calc_checksum(pkt, 7);

    uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, 8);
    uart_wait_tx_done(SERVO_UART_NUM, pdMS_TO_TICKS(10));
    post_write_drain();
}

void servo_set_torque_limit(uint8_t id, uint16_t limit)
{
    bus_delay();

    uint8_t pkt[9];
    pkt[0] = HEADER;
    pkt[1] = HEADER;
    pkt[2] = id;
    pkt[3] = 5;            /* len: inst(1)+addr(1)+data(2)+chk(1) */
    pkt[4] = INST_WRITE;
    pkt[5] = REG_TORQUE_LIMIT;
    pkt[6] = limit & 0xFF;
    pkt[7] = (limit >> 8) & 0xFF;
    pkt[8] = calc_checksum(pkt, 8);

    uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, 9);
    uart_wait_tx_done(SERVO_UART_NUM, pdMS_TO_TICKS(10));
    post_write_drain();
}
