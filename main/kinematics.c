#include "kinematics.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "KIN";

/* Home 位置 (舵机值) — 与 robot.c 中 JOINTS[] 一致 */
static const int16_t HOME_POS[5] = { 3000, 2180, 1550, 1970, 3000 };

/* STS3215: 0-4095 对应 0-360°, 即 2π 弧度 */
#define POS_TO_RAD  (2.0f * M_PI / 4095.0f)
#define RAD_TO_POS  (4095.0f / (2.0f * M_PI))

/* 工作空间限制 (mm) */
#define WS_MAX_REACH  (KIN_A2 + KIN_A3 + KIN_D5)  /* ~290mm */
#define WS_MIN_REACH  20.0f
#define WS_Z_MIN     -50.0f
#define WS_Z_MAX      (KIN_D1 + WS_MAX_REACH)

void kin_init(void)
{
    ESP_LOGI(TAG, "Kinematics init: d1=%.0f a2=%.0f a3=%.0f d5=%.0f mm",
             KIN_D1, KIN_A2, KIN_A3, KIN_D5);
    ESP_LOGI(TAG, "Max reach: %.0f mm", WS_MAX_REACH);
}

float kin_pos_to_rad(int idx, int16_t pos)
{
    if (idx < 0 || idx > 4) return 0.0f;
    /* 角度 = (pos - home) 对应的弧度偏移 */
    return (float)(pos - HOME_POS[idx]) * POS_TO_RAD;
}

int16_t kin_rad_to_pos(int idx, float rad)
{
    if (idx < 0 || idx > 4) return HOME_POS[0];
    int32_t pos = HOME_POS[idx] + (int32_t)roundf(rad * RAD_TO_POS);
    /* 钳位到 0-4095 */
    if (pos < 0) pos = 0;
    if (pos > 4095) pos = 4095;
    return (int16_t)pos;
}

void kin_forward(const kin_joints_t *joints, kin_pos_t *out_pos, float *out_psi)
{
    float t1 = joints->j[0];  /* J1: 底座旋转 */
    float t2 = joints->j[1];  /* J2: 肩部 */
    float t3 = joints->j[2];  /* J3: 肘部 */
    float t4 = joints->j[3];  /* J4: 腕部俯仰 */
    /* J5: 腕部翻转, 不影响位置 */

    float c1 = cosf(t1), s1 = sinf(t1);

    /* 平面内的投影距离和高度 */
    float t23  = t2 + t3;
    float t234 = t23 + t4;

    float r = KIN_A2 * cosf(t2) + KIN_A3 * cosf(t23) + KIN_D5 * cosf(t234);
    float z = KIN_D1 + KIN_A2 * sinf(t2) + KIN_A3 * sinf(t23) + KIN_D5 * sinf(t234);

    out_pos->x = c1 * r;
    out_pos->y = s1 * r;
    out_pos->z = z;

    if (out_psi) {
        *out_psi = t234;  /* 腕部总俯仰角 */
    }
}

kin_result_t kin_inverse(const kin_pos_t *target, float psi, kin_joints_t *out_joints)
{
    float x = target->x;
    float y = target->y;
    float z = target->z;

    /* === J1: 底座旋转 === */
    float r_xy = sqrtf(x * x + y * y);
    float t1;
    if (r_xy < 1.0f) {
        /* 太靠近 Z 轴, J1 不确定 → 保持当前值 */
        t1 = out_joints->j[0];
    } else {
        t1 = atan2f(y, x);
    }

    /* === 去除 d5 的贡献, 求腕心 === */
    float r  = r_xy - KIN_D5 * cosf(psi);
    float wz = z - KIN_D1 - KIN_D5 * sinf(psi);

    /* === J2, J3: 二连杆 IK (余弦定理) === */
    float D_sq = r * r + wz * wz;
    float D = sqrtf(D_sq);

    /* 可达性检查 */
    if (D > (KIN_A2 + KIN_A3 - 0.1f)) {
        return KIN_UNREACHABLE;
    }
    if (D < fabsf(KIN_A2 - KIN_A3) + 0.1f) {
        return KIN_UNREACHABLE;
    }

    float cos_t3 = (D_sq - KIN_A2 * KIN_A2 - KIN_A3 * KIN_A3) / (2.0f * KIN_A2 * KIN_A3);

    /* 数值钳位防止 acos 越界 */
    if (cos_t3 > 1.0f) cos_t3 = 1.0f;
    if (cos_t3 < -1.0f) cos_t3 = -1.0f;

    /* 取肘上解 (elbow-up) */
    float t3 = atan2f(sqrtf(1.0f - cos_t3 * cos_t3), cos_t3);

    float alpha = atan2f(wz, r);
    float beta  = atan2f(KIN_A3 * sinf(t3), KIN_A2 + KIN_A3 * cos_t3);
    float t2 = alpha - beta;

    /* === J4: 腕部俯仰补偿 === */
    float t4 = psi - t2 - t3;

    /* === 奇异性检查 === */
    if (fabsf(cos_t3) > 0.98f) {
        return KIN_SINGULAR;
    }

    out_joints->j[0] = t1;
    out_joints->j[1] = t2;
    out_joints->j[2] = t3;
    out_joints->j[3] = t4;
    /* j[4] (J5) 不由位置 IK 控制 */

    return KIN_OK;
}

int kin_check_workspace(const kin_pos_t *pos)
{
    float r_xy = sqrtf(pos->x * pos->x + pos->y * pos->y);
    float r_total = sqrtf(r_xy * r_xy + (pos->z - KIN_D1) * (pos->z - KIN_D1));

    if (r_total > WS_MAX_REACH * 0.99f) return 0;
    if (r_total < WS_MIN_REACH) return 0;
    if (pos->z < WS_Z_MIN) return 0;
    if (pos->z > WS_Z_MAX) return 0;

    return 1;
}
