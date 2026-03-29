#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_NUM_JOINTS  6

/* 初始化舵机 UART */
void servo_init(void);

/* 读取舵机当前位置 (返回 0-4095, 失败返回 -1) */
int16_t servo_read_pos(uint8_t id);

/* 写入舵机目标位置和速度 (speed=0 表示最大速度) */
void servo_write_pos(uint8_t id, int16_t position, uint16_t speed);

/* 关闭舵机扭矩（可手动拖动） */
void servo_torque_off(uint8_t id);

/* 开启舵机扭矩 */
void servo_torque_on(uint8_t id);

/* 设置扭矩限制 (0-1000, 1000=满扭矩) */
void servo_set_torque_limit(uint8_t id, uint16_t limit);

#endif
