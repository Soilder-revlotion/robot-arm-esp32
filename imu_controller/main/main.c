/**
 * IMU 控制器 - ESP-NOW 发送端
 * 硬件：ESP32-S3 + MPU6050 + 摇杆 + 按钮
 *
 * 接线:
 *   MPU6050: SCL→GPIO9, SDA→GPIO8, VCC→3.3V, GND→GND
 *   摇杆:   VRX→GPIO1(ADC), VRY→GPIO2(ADC)
 *   按钮:   SW→GPIO4
 *
 * 发送绝对姿态角 (互补滤波), 接收端直接映射到关节位置
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

static const char *TAG = "IMU_TX";

/* ============ 硬件引脚配置 ============ */
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SCL_IO   GPIO_NUM_9
#define I2C_MASTER_SDA_IO   GPIO_NUM_8
#define I2C_MASTER_FREQ_HZ  400000

#define BUTTON_GPIO         GPIO_NUM_4
#define VRX_ADC_CHANNEL     ADC_CHANNEL_0   /* GPIO1 → ADC1_CH0 */
#define VRY_ADC_CHANNEL     ADC_CHANNEL_1   /* GPIO2 → ADC1_CH1 */

/* MPU6050 寄存器 */
#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I     0x75

/* ============ 数据包格式 (与接收端一致) ============ */
typedef struct __attribute__((packed)) {
    uint8_t  magic;       /* 0xA5 */
    uint8_t  seq;         /* 序列号 */
    float    roll;        /* 横滚角 (弧度, 绝对值) */
    float    pitch;       /* 俯仰角 (弧度, 绝对值) */
    float    yaw;         /* 偏航角 (弧度, 绝对值, 仅陀螺积分) */
    uint8_t  trigger;     /* 按钮: 1=按下 0=松开 */
    uint8_t  gripper;     /* 夹爪 0-255 (摇杆 VRX) */
    uint8_t  aux_axis;    /* 辅助轴 0-255 (摇杆 VRY) */
    uint8_t  checksum;    /* 前 17 字节 XOR */
} imu_packet_t;

#define IMU_PKT_MAGIC  0xA5

/* WiFi 信道: 必须与接收端路由器的信道一致!
 * 接收端串口日志会打印: "WiFi channel: X"
 * 修改这里匹配那个值 */
#define ESPNOW_CHANNEL  1

/* ============ 全局状态 ============ */
static uint8_t s_seq = 0;
static uint8_t s_peer_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

/* ============ I2C 读写 ============ */

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_MASTER_NUM, addr, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

/* ============ MPU6050 ============ */

static int mpu6050_init(void)
{
    uint8_t chip_id = 0;
    if (i2c_read_regs(MPU6050_ADDR, MPU6050_WHO_AM_I, &chip_id, 1) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not found! Check wiring (SDA=GPIO%d, SCL=GPIO%d)",
                 I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        return -1;
    }
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02X", chip_id);

    /* 唤醒 */
    i2c_write_reg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 加速度计: ±2g */
    i2c_write_reg(MPU6050_ADDR, 0x1C, 0x00);
    /* 陀螺仪: ±250°/s */
    i2c_write_reg(MPU6050_ADDR, 0x1B, 0x00);
    /* 低通滤波: ~44Hz */
    i2c_write_reg(MPU6050_ADDR, 0x1A, 0x03);

    ESP_LOGI(TAG, "MPU6050 initialized (±2g, ±250°/s)");
    return 0;
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3])
{
    uint8_t buf[14];
    if (i2c_read_regs(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14) == ESP_OK) {
        for (int i = 0; i < 3; i++) {
            accel[i] = (int16_t)((buf[i * 2] << 8) | buf[i * 2 + 1]);
        }
        /* buf[6..7] = temperature, skip */
        for (int i = 0; i < 3; i++) {
            gyro[i] = (int16_t)((buf[8 + i * 2] << 8) | buf[8 + i * 2 + 1]);
        }
    } else {
        memset(accel, 0, 6);
        memset(gyro, 0, 6);
    }
}

/* ============ ADC (摇杆) ============ */

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, VRX_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, VRY_ADC_CHANNEL, &chan_cfg));

    ESP_LOGI(TAG, "ADC initialized (VRX=CH%d, VRY=CH%d)", VRX_ADC_CHANNEL, VRY_ADC_CHANNEL);
}

static uint8_t adc_read_8bit(adc_channel_t ch)
{
    int raw = 0;
    adc_oneshot_read(s_adc_handle, ch, &raw);
    /* 12bit (0-4095) → 8bit (0-255) */
    return (uint8_t)(raw >> 4);
}

/* ============ 按钮 ============ */

static void button_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Button on GPIO%d (active LOW)", BUTTON_GPIO);
}

static bool button_read(void)
{
    return gpio_get_level(BUTTON_GPIO) == 0;
}

/* ============ 校验和 ============ */

static uint8_t calc_checksum(const uint8_t *data, int len)
{
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

/* ============ ESP-NOW ============ */

static void espnow_send_cb(const uint8_t *mac, esp_now_send_status_t status)
{
    (void)mac;
    (void)status;
}

static void wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "WiFi STA started, channel=%d", ESPNOW_CHANNEL);
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, s_peer_addr, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW TX ready (broadcast)");
}

/* ============ 主任务 (50Hz) ============ */

/*
 * 控制方式说明:
 * - MPU6050 倾斜 → 发送绝对姿态角 (互补滤波)
 *     pitch (前后倾) → J2 肩部
 *     roll  (上下翻) → J3 肘部
 *     yaw   (左右转) → J4 腕部
 * - 摇杆 VRY → 腕部旋转 J5: 前推=减小, 后推=增大
 * - 按钮 SW  → 夹爪 J6: 按住=夹紧(max), 松开=松开(min)
 */

/* 互补滤波系数: 0.98偏信陀螺仪, 0.02修正加速度计 */
#define COMP_ALPHA  0.98f

static void imu_task(void *arg)
{
    int16_t accel[3], gyro[3];
    int64_t last_us = esp_timer_get_time();
    uint32_t count = 0;

    /* 绝对姿态角 (弧度) */
    float abs_roll  = 0.0f;
    float abs_pitch = 0.0f;
    float abs_yaw   = 0.0f;

    /* 初始化: 用加速度计获取初始姿态 */
    mpu6050_read_raw(accel, gyro);
    float ax = accel[0] / 16384.0f;
    float ay = accel[1] / 16384.0f;
    float az = accel[2] / 16384.0f;
    abs_roll  = atan2f(ay, az);
    abs_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    abs_yaw   = 0.0f;  /* 无磁力计, yaw 从 0 开始 */

    ESP_LOGI(TAG, "Initial pose: roll=%.1f pitch=%.1f deg",
             (double)(abs_roll * 180.0f / M_PI),
             (double)(abs_pitch * 180.0f / M_PI));

    while (1) {
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) / 1000000.0f;
        if (dt > 0.05f) dt = 0.02f;  /* 防止异常大步 */
        if (dt < 0.001f) dt = 0.001f;
        last_us = now_us;

        /* 读 MPU6050 */
        mpu6050_read_raw(accel, gyro);

        /* 加速度 → g */
        ax = accel[0] / 16384.0f;
        ay = accel[1] / 16384.0f;
        az = accel[2] / 16384.0f;

        /* 加速度计计算的姿态角 */
        float acc_roll  = atan2f(ay, az);
        float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        /* 陀螺仪角速度 (°/s → rad/s) */
        float gyro_scale = (M_PI / 180.0f) / 131.0f;
        float gx = gyro[0] * gyro_scale;
        float gy = gyro[1] * gyro_scale;
        float gz = gyro[2] * gyro_scale;

        /* 互补滤波: roll & pitch */
        abs_roll  = COMP_ALPHA * (abs_roll  + gx * dt) + (1.0f - COMP_ALPHA) * acc_roll;
        abs_pitch = COMP_ALPHA * (abs_pitch + gy * dt) + (1.0f - COMP_ALPHA) * acc_pitch;

        /* yaw: 仅陀螺仪积分 (无磁力计参考, 会缓慢漂移) */
        abs_yaw += gz * dt;

        /* 读摇杆 + 按钮 */
        uint8_t vrx = adc_read_8bit(VRX_ADC_CHANNEL);
        uint8_t vry = adc_read_8bit(VRY_ADC_CHANNEL);
        bool btn = button_read();

        /* 按钮控制 J6: 按下=夹紧(255), 松开=松开(0) */
        uint8_t gripper_val = btn ? 255 : 0;

        /* 构建数据包: 发送绝对姿态角 */
        imu_packet_t pkt;
        pkt.magic    = IMU_PKT_MAGIC;
        pkt.seq      = s_seq++;
        pkt.roll     = abs_roll;      /* 绝对横滚角 (弧度) */
        pkt.pitch    = abs_pitch;     /* 绝对俯仰角 (弧度) */
        pkt.yaw      = abs_yaw;       /* 绝对偏航角 (弧度) */
        pkt.trigger  = btn ? 1 : 0;   /* 按钮状态 */
        pkt.gripper  = gripper_val;   /* 按钮控制夹爪: 按下=255, 松开=0 */
        pkt.aux_axis = vry;           /* 摇杆 Y → 腕部旋转 J5 */
        pkt.checksum = calc_checksum((uint8_t *)&pkt, sizeof(imu_packet_t) - 1);

        /* 发送 */
        esp_now_send(s_peer_addr, (uint8_t *)&pkt, sizeof(imu_packet_t));

        /* 每 50 次 (1秒) 打印一次状态 */
        if (++count % 50 == 0) {
            ESP_LOGI(TAG, "R=%.1f P=%.1f Y=%.1f btn=%d grip=%d vry=%d",
                     (double)(abs_roll * 180.0f / M_PI),
                     (double)(abs_pitch * 180.0f / M_PI),
                     (double)(abs_yaw * 180.0f / M_PI),
                     btn, gripper_val, vry);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); /* 50Hz */
    }
}

/* ============ app_main ============ */

void app_main(void)
{
    ESP_LOGI(TAG, "=== IMU Controller (ESP-NOW TX) ===");

    /* I2C */
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C: SDA=GPIO%d, SCL=GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    /* MPU6050 */
    if (mpu6050_init() != 0) {
        ESP_LOGE(TAG, "MPU6050 init failed! Halting.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* ADC (摇杆) */
    adc_init();

    /* 按钮 */
    button_init();

    /* WiFi + ESP-NOW */
    wifi_init();
    espnow_init();

    /* 启动 IMU 任务 */
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "=============================");
    ESP_LOGI(TAG, "Sending at 50Hz on channel %d", ESPNOW_CHANNEL);
    ESP_LOGI(TAG, "=============================");
}
