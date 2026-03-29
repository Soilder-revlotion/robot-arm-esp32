#ifndef ESPNOW_RX_H
#define ESPNOW_RX_H

#include <stdint.h>
#include <stdbool.h>

/* ESP-NOW 数据包格式 (18 字节) */
typedef struct __attribute__((packed)) {
    uint8_t  magic;       /* 0xA5 */
    uint8_t  seq;         /* 递增序列号 */
    float    roll;        /* 横滚角 (弧度, 绝对值) */
    float    pitch;       /* 俯仰角 (弧度, 绝对值) */
    float    yaw;         /* 偏航角 (弧度, 绝对值) */
    uint8_t  trigger;     /* 死人开关: 1=按下 0=松开 */
    uint8_t  gripper;     /* 夹爪 0-255 */
    uint8_t  aux_axis;    /* 辅助轴 0-255 */
    uint8_t  checksum;    /* 前17字节 XOR */
} imu_packet_t;

#define IMU_PKT_MAGIC  0xA5

/* 初始化 ESP-NOW 接收 (需在 WiFi 初始化之后调用) */
void espnow_rx_init(void);

/* 获取最新的 IMU 数据包 (线程安全)
 * 返回: true=有新数据, false=无新数据或超时
 * timestamp: 收到包的时间 (微秒, esp_timer) */
bool espnow_rx_get_latest(imu_packet_t *pkt, int64_t *timestamp);

/* 控制器是否在线 (最后收包距今 < 500ms) */
bool espnow_rx_is_connected(void);

#endif
