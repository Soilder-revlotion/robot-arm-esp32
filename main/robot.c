#include "robot.h"
#include "servo.h"
#include "espnow_rx.h"
#include "kinematics.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "ROBOT";

/* ============ 关节限位配置 ============ */
typedef struct {
    uint8_t  id;
    int16_t  min_pos;
    int16_t  max_pos;
    int16_t  home_pos;
} joint_config_t;

static const joint_config_t JOINTS[SERVO_NUM_JOINTS] = {
    { .id = 1, .min_pos = 1900, .max_pos = 4000, .home_pos = 3000 },
    { .id = 2, .min_pos = 1000, .max_pos = 2500, .home_pos = 2180 },
    { .id = 3, .min_pos = 1550, .max_pos = 3000, .home_pos = 1550 },
    { .id = 4, .min_pos = 400,  .max_pos = 4000, .home_pos = 1970 },
    { .id = 5, .min_pos = 2200, .max_pos = 4000, .home_pos = 3000 },
    { .id = 6, .min_pos = 2000, .max_pos = 4000, .home_pos = 2048 },
};

/* ============ 录制缓冲区 ============ */
#define RECORD_MAX_FRAMES  6000  /* 5分钟 @20Hz */

typedef struct {
    int16_t positions[SERVO_NUM_JOINTS];
} record_frame_t;

/* ============ 全局状态 ============ */
static robot_mode_t   s_mode = ROBOT_MODE_IDLE;
static bool           s_loop_playback = false;
static uint32_t       s_play_index = 0;
static uint16_t       s_speed = 500;  /* 默认中等速度, 减少抖动 */
static volatile bool  s_torque_releasing = false; /* 泄力过程中暂停servo_task */

static int16_t        s_target_pos[SERVO_NUM_JOINTS];
static int16_t        s_current_pos[SERVO_NUM_JOINTS];

static record_frame_t *s_frames = NULL;
static uint32_t       s_frame_count = 0;

static SemaphoreHandle_t s_mutex = NULL;

static ws_push_cb_t   s_ws_cb = NULL;
static void          *s_ws_cb_arg = NULL;

/* ============ IMU 控制状态 ============ */
static float          s_imu_sensitivity = 2.0f;  /* mm/度 */
static kin_pos_t      s_ee_target = {0};          /* 末端目标位置 */
static float          s_psi_target = 0;           /* 腕部俯仰角 */
static kin_pos_t      s_ee_current = {0};         /* 末端当前位置 (供显示) */

/* 进入 IMU 模式时记录 MPU6050 初始角度作为零点 */
static float          s_imu_ref_roll  = 0;
static float          s_imu_ref_pitch = 0;
static float          s_imu_ref_yaw   = 0;
static bool           s_imu_needs_ref = true;

#define IMU_MAX_DELTA_MM   10.0f  /* 每帧最大位移 (mm) */
#define IMU_TIMEOUT_US     500000 /* 通信超时 (500ms) */

/* ============ 辅助函数 ============ */

static int16_t clamp(int16_t val, int16_t lo, int16_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static void read_all_positions(void)
{
    for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
        int16_t pos = servo_read_pos(JOINTS[i].id);
        if (pos >= 0) {
            s_current_pos[i] = pos;
        }
    }
}

static void write_all_positions(const int16_t pos[SERVO_NUM_JOINTS])
{
    for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
        int16_t clamped = clamp(pos[i], JOINTS[i].min_pos, JOINTS[i].max_pos);
        servo_write_pos(JOINTS[i].id, clamped, s_speed);
    }
}

static void all_torque_on(void)
{
    for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
        servo_set_torque_limit(JOINTS[i].id, 1000);
        servo_torque_on(JOINTS[i].id);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void all_torque_off(void)
{
    /*
     * 逐步降低扭矩限制, 让机械臂缓慢卸力而不是瞬间掉落.
     * 从 950 每步减 50, 共 20 步, 每步间隔 150ms, 总计约 3 秒.
     */
    ESP_LOGI(TAG, "Gradual torque release starting...");
    s_torque_releasing = true;

    /* 等一个 servo_task 周期确保它已暂停 */
    vTaskDelay(pdMS_TO_TICKS(40));

    for (int limit = 950; limit >= 0; limit -= 50) {
        for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
            servo_set_torque_limit(JOINTS[i].id, (uint16_t)limit);
        }
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    /* 扭矩已降到0, 正式关闭扭矩 */
    for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
        servo_torque_off(JOINTS[i].id);
    }
    vTaskDelay(pdMS_TO_TICKS(30));

    s_torque_releasing = false;
    ESP_LOGI(TAG, "Torque released gradually over ~3s");
}

static void move_to_home(void)
{
    int16_t home[SERVO_NUM_JOINTS];
    for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
        home[i] = JOINTS[i].home_pos;
    }
    write_all_positions(home);
}

/* ============ servo_task (50Hz) ============ */

static void servo_task(void *arg)
{
    uint32_t tick = 0;

    while (1) {
        /* 泄力过程中暂停 UART 操作, 避免总线冲突 */
        if (s_torque_releasing) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        robot_mode_t mode = s_mode;
        xSemaphoreGive(s_mutex);

        switch (mode) {
        case ROBOT_MODE_IDLE:
            read_all_positions();
            break;

        case ROBOT_MODE_WEB_CONTROL: {
            int16_t target[SERVO_NUM_JOINTS];
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            memcpy(target, s_target_pos, sizeof(target));
            xSemaphoreGive(s_mutex);

            write_all_positions(target);
            read_all_positions();
            break;
        }

        case ROBOT_MODE_TEACHING:
            read_all_positions();
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            if (s_frame_count < RECORD_MAX_FRAMES) {
                memcpy(s_frames[s_frame_count].positions,
                       s_current_pos, sizeof(s_current_pos));
                s_frame_count++;
            }
            xSemaphoreGive(s_mutex);
            break;

        case ROBOT_MODE_PLAYBACK: {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            bool done = false;
            if (s_frame_count == 0) {
                s_mode = ROBOT_MODE_IDLE;
                done = true;
            } else {
                write_all_positions(s_frames[s_play_index].positions);
                s_play_index++;
                if (s_play_index >= s_frame_count) {
                    if (s_loop_playback) {
                        s_play_index = 0;
                    } else {
                        s_mode = ROBOT_MODE_IDLE;
                        done = true;
                    }
                }
            }
            xSemaphoreGive(s_mutex);

            if (!done) {
                read_all_positions();
            }
            break;
        }

        case ROBOT_MODE_IMU_CONTROL: {
            /* 获取最新 IMU 数据包 */
            imu_packet_t pkt;
            int64_t rx_time;
            bool has_data = espnow_rx_get_latest(&pkt, &rx_time);

            /* 调试：打印连接状态 */
            static uint32_t s_last_print = 0;
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            bool connected = espnow_rx_is_connected();
            if (!connected && (now_ms - s_last_print > 2000)) {
                ESP_LOGW(TAG, "IMU not connected! Check controller WiFi channel");
                s_last_print = now_ms;
            }

            /* 超时检查 */
            if (!connected) {
                read_all_positions();
                break;
            }

            if (!has_data) {
                read_all_positions();
                break;
            }

            /* 首帧: 捕获 IMU 当前角度作为零点参考
             * 这样无论 MPU6050 当前是什么姿态, 进入时手臂都从 home 开始 */
            if (s_imu_needs_ref) {
                s_imu_ref_roll  = pkt.roll;
                s_imu_ref_pitch = pkt.pitch;
                s_imu_ref_yaw   = pkt.yaw;
                s_imu_needs_ref = false;
                ESP_LOGI(TAG, "IMU ref captured: R=%.1f P=%.1f Y=%.1f deg",
                         (double)(pkt.roll * 180.0f / M_PI),
                         (double)(pkt.pitch * 180.0f / M_PI),
                         (double)(pkt.yaw * 180.0f / M_PI));
                read_all_positions();
                break;  /* 跳过本帧, 仅校准 */
            }

            /* 读取当前关节位置 */
            read_all_positions();

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            float sensitivity = s_imu_sensitivity;
            xSemaphoreGive(s_mutex);

            /* === 相对角度 → 关节位置映射 ===
             *
             * 用 (当前角度 - 进入时的参考角度) 作为偏移量:
             *   dpitch → J2 肩部: 前倾=J2减小, 后仰=J2增大
             *   droll  → J3 肘部: 下翻=J3减小, 上翻=J3增大
             *   dyaw   → J4 腕部: 左转=J4增大, 右转=J4减小
             *
             * 摇杆 VRY → J5 腕部翻转: 前推=减小, 后推=增大
             * 按钮 SW  → J6 夹爪: 按住=夹紧(max), 松开=松开(min)
             *
             * 公式: target = home_pos + (angle - ref_angle) * scale
             */

            /* 计算相对偏移 */
            float droll  = pkt.roll  - s_imu_ref_roll;
            float dpitch = pkt.pitch - s_imu_ref_pitch;
            float dyaw   = pkt.yaw   - s_imu_ref_yaw;

            /* 角度→舵机步数的缩放: 弧度 × scale = 舵机步数偏移
             * sensitivity=2.0, scale≈1200, 即 ±90°(1.57rad) 对应 ±1885 步 */
            float scale = sensitivity * 600.0f;

            int16_t target_pos[SERVO_NUM_JOINTS];
            /* 先复制当前位置，未被控制的轴保持 */
            memcpy(target_pos, s_current_pos, sizeof(target_pos));

            /* J2 肩部: pitch 前倾(dpitch>0)→J2减小, 后仰(dpitch<0)→J2增大 */
            target_pos[1] = clamp(
                JOINTS[1].home_pos + (int32_t)(-dpitch * scale),
                JOINTS[1].min_pos, JOINTS[1].max_pos);

            /* J3 肘部: roll 下翻(droll<0)→J3减小, 上翻(droll>0)→J3增大 */
            target_pos[2] = clamp(
                JOINTS[2].home_pos + (int32_t)(droll * scale),
                JOINTS[2].min_pos, JOINTS[2].max_pos);

            /* J4 腕部: yaw 左转(dyaw>0)→J4增大, 右转(dyaw<0)→J4减小 */
            target_pos[3] = clamp(
                JOINTS[3].home_pos + (int32_t)(dyaw * scale),
                JOINTS[3].min_pos, JOINTS[3].max_pos);

            /* J5 腕部翻转: 摇杆 VRY */
            {
                int joy_y = (int)pkt.aux_axis - 128;  /* -128~127 */
                if (abs(joy_y) > 10) {  /* 摇杆死区 */
                    int delta5 = -joy_y / 4;
                    target_pos[4] = clamp(s_current_pos[4] + delta5,
                                          JOINTS[4].min_pos, JOINTS[4].max_pos);
                }
            }

            /* J6 夹爪: 按钮控制
             * trigger=1 (按钮按下) → J6 最大 (夹紧)
             * trigger=0 (按钮松开) → J6 最小 (松开)
             */
            if (pkt.trigger) {
                target_pos[5] = JOINTS[5].max_pos;  /* 夹紧 */
            } else {
                target_pos[5] = JOINTS[5].min_pos;  /* 松开 */
            }

            /* 写入舵机 */
            write_all_positions(target_pos);

            break;
        }
        }

        /* 每 5 个周期 (10Hz) 推送 WebSocket 数据 */
        tick++;
        if (tick % 5 == 0 && s_ws_cb != NULL) {
            s_ws_cb(s_ws_cb_arg);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============ 公共 API ============ */

void robot_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    s_frames = (record_frame_t *)malloc(RECORD_MAX_FRAMES * sizeof(record_frame_t));
    configASSERT(s_frames);
    s_frame_count = 0;

    /* 初始化目标位置为 Home */
    for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
        s_target_pos[i] = JOINTS[i].home_pos;
    }

    /* 开启扭矩并移到 Home */
    all_torque_on();
    move_to_home();

    /* 等待舵机到位后读取位置 */
    vTaskDelay(pdMS_TO_TICKS(500));
    read_all_positions();

    ESP_LOGI(TAG, "Robot init OK. Positions: [%d,%d,%d,%d,%d,%d]",
             s_current_pos[0], s_current_pos[1], s_current_pos[2],
             s_current_pos[3], s_current_pos[4], s_current_pos[5]);
}

void robot_start_task(void)
{
    xTaskCreate(servo_task, "servo_task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "servo_task started (50Hz)");
}

robot_mode_t robot_get_mode(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    robot_mode_t m = s_mode;
    xSemaphoreGive(s_mutex);
    return m;
}

void robot_set_mode(robot_mode_t mode)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    robot_mode_t old = s_mode;

    if (old == mode) {
        xSemaphoreGive(s_mutex);
        return;
    }

    /* 离开旧模式的清理 */
    if (old == ROBOT_MODE_TEACHING) {
        ESP_LOGI(TAG, "Teaching stopped. Recorded %lu frames", (unsigned long)s_frame_count);
    }

    /* 进入新模式的初始化 */
    switch (mode) {
    case ROBOT_MODE_IDLE:
        s_mode = ROBOT_MODE_IDLE;
        xSemaphoreGive(s_mutex);
        all_torque_on();
        return;

    case ROBOT_MODE_WEB_CONTROL:
        /* 读当前位置作为初始目标，防止突跳 */
        memcpy(s_target_pos, s_current_pos, sizeof(s_target_pos));
        s_mode = ROBOT_MODE_WEB_CONTROL;
        xSemaphoreGive(s_mutex);
        all_torque_on();
        return;

    case ROBOT_MODE_TEACHING:
        s_frame_count = 0;
        s_mode = ROBOT_MODE_TEACHING;
        xSemaphoreGive(s_mutex);
        all_torque_off();
        return;

    case ROBOT_MODE_PLAYBACK:
        if (s_frame_count == 0) {
            ESP_LOGW(TAG, "No recording to play");
            xSemaphoreGive(s_mutex);
            return;
        }
        s_play_index = 0;
        s_mode = ROBOT_MODE_PLAYBACK;
        xSemaphoreGive(s_mutex);
        all_torque_on();
        return;

    case ROBOT_MODE_IMU_CONTROL: {
        /* 先回到 Home 位置, 再进入 IMU 控制 */
        for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
            s_target_pos[i] = JOINTS[i].home_pos;
        }
        s_imu_needs_ref = true;  /* 下一帧捕获 IMU 零点 */

        s_mode = ROBOT_MODE_IMU_CONTROL;
        xSemaphoreGive(s_mutex);
        all_torque_on();
        move_to_home();
        /* 等待舵机到达 home 位置 */
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "IMU control: arm at home, waiting for IMU reference...");
        return;
    }
    }

    xSemaphoreGive(s_mutex);
}

void robot_set_target(const int16_t pos[SERVO_NUM_JOINTS])
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
        s_target_pos[i] = clamp(pos[i], JOINTS[i].min_pos, JOINTS[i].max_pos);
    }
    xSemaphoreGive(s_mutex);
}

void robot_get_current_pos(int16_t pos[SERVO_NUM_JOINTS])
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(pos, s_current_pos, sizeof(s_current_pos));
    xSemaphoreGive(s_mutex);
}

uint32_t robot_get_record_count(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t n = s_frame_count;
    xSemaphoreGive(s_mutex);
    return n;
}

void robot_clear_recording(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_frame_count = 0;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Recording cleared");
}

void robot_set_playback_loop(bool loop)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_loop_playback = loop;
    xSemaphoreGive(s_mutex);
}

void robot_set_speed(uint16_t speed)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_speed = speed;
    xSemaphoreGive(s_mutex);
}

void robot_set_ws_push_cb(ws_push_cb_t cb, void *arg)
{
    s_ws_cb = cb;
    s_ws_cb_arg = arg;
}

void robot_go_home(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    robot_mode_t old = s_mode;

    /* 切到 IDLE 而不是 WEB_CONTROL, 这样 JS 端的 vals 会持续
     * 从推送中同步到真实位置, 等手臂到达 home 后 vals 自然正确.
     * 用户再点"手动控制"时不会出现跳回现象. */
    s_mode = ROBOT_MODE_IDLE;
    for (int i = 0; i < SERVO_NUM_JOINTS; i++) {
        s_target_pos[i] = JOINTS[i].home_pos;
    }
    xSemaphoreGive(s_mutex);

    /* 确保扭矩开启 */
    if (old == ROBOT_MODE_TEACHING || old == ROBOT_MODE_IMU_CONTROL) {
        all_torque_on();
    }

    /* 发送回原点命令 (STS3215 舵机收到位置指令后内部 PID 自行到位) */
    move_to_home();
    ESP_LOGI(TAG, "Moving to home position");
}

void robot_set_imu_sensitivity(float sensitivity)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_imu_sensitivity = sensitivity;
    xSemaphoreGive(s_mutex);
}

bool robot_get_imu_connected(void)
{
    return espnow_rx_is_connected();
}

void robot_get_endeffector_pos(float *x, float *y, float *z)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *x = s_ee_current.x;
    *y = s_ee_current.y;
    *z = s_ee_current.z;
    xSemaphoreGive(s_mutex);
}
