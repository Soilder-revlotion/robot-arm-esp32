#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include <stdbool.h>
#include "servo.h"

/* 运行模式 */
typedef enum {
    ROBOT_MODE_IDLE = 0,
    ROBOT_MODE_WEB_CONTROL,
    ROBOT_MODE_TEACHING,
    ROBOT_MODE_PLAYBACK,
    ROBOT_MODE_IMU_CONTROL,
} robot_mode_t;

/* WebSocket 推送回调类型 */
typedef void (*ws_push_cb_t)(void *arg);

/* 初始化机械臂控制层（创建互斥锁、分配缓冲区、开启扭矩、读初始位置） */
void robot_init(void);

/* 启动 50Hz servo_task */
void robot_start_task(void);

/* 获取/设置运行模式（线程安全） */
robot_mode_t robot_get_mode(void);
void robot_set_mode(robot_mode_t mode);

/* 设置 Web 控制的目标位置（已做限位 clamp） */
void robot_set_target(const int16_t pos[SERVO_NUM_JOINTS]);

/* 获取当前实际位置 */
void robot_get_current_pos(int16_t pos[SERVO_NUM_JOINTS]);

/* 录制信息 */
uint32_t robot_get_record_count(void);
void robot_clear_recording(void);

/* 回放控制 */
void robot_set_playback_loop(bool loop);

/* 舵机运动速度 (0=最大) */
void robot_set_speed(uint16_t speed);

/* 注册 WebSocket 推送回调（在 servo_task 中以 10Hz 调用） */
void robot_set_ws_push_cb(ws_push_cb_t cb, void *arg);

/* 回原点: 开启扭矩并移动到 Home 位置 */
void robot_go_home(void);

/* IMU 控制灵敏度 (mm/度, 默认 2.0) */
void robot_set_imu_sensitivity(float sensitivity);

/* 查询 IMU 控制器是否在线 */
bool robot_get_imu_connected(void);

/* 获取末端执行器位置 (mm) */
void robot_get_endeffector_pos(float *x, float *y, float *z);

#endif
