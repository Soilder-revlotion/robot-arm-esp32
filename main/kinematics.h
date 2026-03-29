#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

/* DH 参数 (毫米) — 根据实测调整 */
#define KIN_D1   65.0f   /* 底座到肩部高度 */
#define KIN_A2  105.0f   /* 肩部到肘部连杆 */
#define KIN_A3  105.0f   /* 肘部到腕部连杆 */
#define KIN_D5   80.0f   /* 腕部到末端长度 */

/* 笛卡尔坐标 (mm) */
typedef struct {
    float x, y, z;
} kin_pos_t;

/* 关节角 (弧度), J1-J5 (J6夹爪不参与位置运动学) */
typedef struct {
    float j[5];
} kin_joints_t;

/* IK 求解结果 */
typedef enum {
    KIN_OK = 0,
    KIN_UNREACHABLE,
    KIN_SINGULAR,
} kin_result_t;

/* 初始化运动学模块 */
void kin_init(void);

/* 舵机位置 (0-4095) 与弧度互转, idx=0..4 对应 J1..J5 */
float kin_pos_to_rad(int idx, int16_t pos);
int16_t kin_rad_to_pos(int idx, float rad);

/* 正运动学: 关节角 → 末端位置 + 腕部总俯仰角 psi */
void kin_forward(const kin_joints_t *joints, kin_pos_t *out_pos, float *out_psi);

/* 逆运动学: 目标位置 + 腕部俯仰角 → 关节角 */
kin_result_t kin_inverse(const kin_pos_t *target, float psi, kin_joints_t *out_joints);

/* 工作空间边界检查 */
int kin_check_workspace(const kin_pos_t *pos);

#endif
