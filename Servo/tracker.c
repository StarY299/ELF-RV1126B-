/** 平滑追踪控制 — 输入插值 + 速度预测 + 输出滤波 */
#include <math.h>
#include "tracker.h"

/* ====== PAN (左右) 参数 — 加强跟踪 ====== */
#define KP_PAN       0.20f
#define DB_PAN       35
#define MAX_STEP_PAN 55

/* ====== TILT (上下) 参数 — 降低灵敏度 ====== */
#define KP_TILT       0.10f
#define DB_TILT       65
#define MAX_STEP_TILT 25

/* ====== 通用参数 ====== */
#define ALPHA_IN     0.45f   /* 输入插值系数                           */
#define ALPHA_OUT    0.45f   /* 输出 PWM 平滑系数                      */
#define VEL_DECAY    0.90f   /* 丢帧速度衰减                           */
#define COAST_CYCLES 30      /* 最大滑行周期                           */
#define LOST_TIMEOUT 100     /* 超时归中                               */
#define CENTER_US    1500

/* ====== 物理限位 (180°舵机) ====== */
#define PAN_MIN_US   600
#define PAN_MAX_US   2400
#define TILT_MIN_US  830   /* 1500 - 60°(666us) */
#define TILT_MAX_US  2170  /* 1500 + 60°(666us) */

/* ====== 内部状态 ====== */
static float pos[2]          = {CENTER_US, CENTER_US};
static float output[2]       = {CENTER_US, CENTER_US};
static float lerp_err[2]     = {0, 0};
static float velocity[2]     = {0, 0};
static int   coast_count[2]  = {0, 0};
static int   lost_count[2]   = {0, 0};

void tracker_init(void) {
    pos[0] = pos[1] = CENTER_US;
    output[0] = output[1] = CENTER_US;
    lerp_err[0] = lerp_err[1] = 0;
    velocity[0] = velocity[1] = 0;
    coast_count[0] = coast_count[1] = 0;
    lost_count[0] = lost_count[1] = 0;
}

int tracker_update(const ai_msg_t *m, int ch) {
    /* 1. 有目标：更新速度和插值目标 */
    float db       = (ch == 0) ? DB_PAN       : DB_TILT;
    if (m->has_target) {
        lost_count[ch]  = 0;
        coast_count[ch] = COAST_CYCLES;

        int center = (ch == 0) ? m->frame_w / 2 : m->frame_h / 2;
        int coord  = (ch == 0) ? m->cx : m->cy;
        float raw_err = (float)(coord - center);

        /* 更新速度 (用上一次插值误差计算) */
        float new_vel = raw_err - lerp_err[ch];
        velocity[ch] = 0.5f * new_vel + 0.5f * velocity[ch];  /* 速度 EMA */

        /* 死区检查：用原始误差 */
        if (fabsf(raw_err) < db) {
            lerp_err[ch] *= 0.8f;
            velocity[ch] *= 0.5f;
            goto move;
        }

        /* 输入插值：不直接跳到 raw_err，而是缓慢逼近 */
        lerp_err[ch] = ALPHA_IN * raw_err + (1.0f - ALPHA_IN) * lerp_err[ch];

    } else {
        /* 2. 无目标 */
        lost_count[ch]++;
        if (coast_count[ch] > 0) {
            /* 滑行：误差向零衰减，速度也衰减 (避免继续冲向极限) */
            coast_count[ch]--;
            lerp_err[ch]  *= 0.92f;              /* 向零衰减 */
            velocity[ch]  *= VEL_DECAY * 0.8f;   /* 更快衰减速度 */
            if (fabsf(lerp_err[ch]) < 1.0f) {
                /* 滑行结束 */
                coast_count[ch] = 0;
                lerp_err[ch] = 0;
                velocity[ch] = 0;
                return (int)output[ch];
            }
            goto move;
        }
        /* 滑行耗尽，超时归中 */
        lerp_err[ch] *= 0.9f;
        velocity[ch] = 0;
        if (lost_count[ch] > LOST_TIMEOUT) {
            float ms = (ch == 0) ? MAX_STEP_PAN : MAX_STEP_TILT;
            if (fabsf(pos[ch] - CENTER_US) < ms / 2.0f + 1.0f) {
                /* 一步之内: 直接归中 */
                pos[ch] = CENTER_US;
                output[ch] = CENTER_US;
                return -1;  /* 信号: 已归中, 可断开舵机 */
            } else {
                float sign = (pos[ch] > CENTER_US) ? -1.0f : 1.0f;
                pos[ch] += sign * (ms / 2.0f);
            }
            output[ch] = pos[ch];
            return (int)output[ch];
        }
        return (int)output[ch];
    }

move:
    /* 3. 计算步进 */
    {
        float kp       = (ch == 0) ? KP_PAN       : KP_TILT;
        float max_step = (ch == 0) ? MAX_STEP_PAN : MAX_STEP_TILT;
        float dir      = (ch == 0) ? -1.0f : 1.0f;  /* PAN反, TILT正 */
        float delta    = dir * lerp_err[ch] * kp;

        /* 自适应步进限幅 */
        float abs_err = fabsf(lerp_err[ch]);
        if (abs_err > 250)       max_step *= 1.2f;
        else if (abs_err > 80)   /* use base max_step */;
        else                     max_step *= 0.5f;

        if (delta >  max_step) delta =  max_step;
        if (delta < -max_step) delta = -max_step;

        /* 死区微调：小误差用更小步进 */
        if (abs_err < db * 2.0f && abs_err > db) {
            delta *= 0.5f;
        }

        pos[ch] += delta;
    }

    /* 4. 输出 PWM EMA 平滑 */
    output[ch] = ALPHA_OUT * pos[ch] + (1.0f - ALPHA_OUT) * output[ch];

    /* 5. 物理限位 */
    {
        int limit_lo = (ch == 0) ? PAN_MIN_US  : TILT_MIN_US;
        int limit_hi = (ch == 0) ? PAN_MAX_US  : TILT_MAX_US;
        if (output[ch] < limit_lo) { output[ch] = limit_lo; pos[ch] = limit_lo; }
        if (output[ch] > limit_hi) { output[ch] = limit_hi; pos[ch] = limit_hi; }
    }

    return (int)output[ch];
}
