/* pid.c —— 位置/航向环：相差距离 → 三道加工 → 本拍命令，20ms 一拍，单位 cm/s、°/s。 */
/* 出发边：1 = 右基准（车头沿任务X）0 = 左基准（车头沿任务Y）。判定"哪个轴是横移轴"要用它 */
extern int follow_side;
#include "pid.h"
#include "chassis_cfg.h"
#include <math.h>

static Pid_t pid_x, pid_y, pid_yaw;       /* 三个环的增益 */

static float vx_cmd, vy_cmd, w_cmd;       /* 上一拍真正发出去的命令（斜坡基准） */
static float ff_vx, ff_vy;                /* 跟踪目标速度前馈，世界系 cm/s */
static uint8_t crawl = 0;                 /* 0 = 常规 / 1 = 蠕动，只影响平移限速与刹车包线 */

/* 把矢量 (vx,vy) 等比缩到模长 ≤ limit —— 方向不变、只压模长（三处限幅共用） */
static void shrink_to(float *vx, float *vy, float limit)
{
    float m = sqrtf(*vx * *vx + *vy * *vy);

    if (m > limit && m > 0.0f) {
        float k = limit / m;
        *vx *= k;
        *vy *= k;
    }
}

void PID_Init(void)
{
    pid_x.kp   = GO_XY_KP;
    pid_y.kp   = GO_XY_KP;
    pid_yaw.kp = GO_YAW_KP;

    PID_Clear();            /* 斜坡基准也归零 */
}

void PID_Clear(void)
{
    vx_cmd = vy_cmd = w_cmd = 0.0f;
    ff_vx = ff_vy = 0.0f;
}

void PID_SetTrackFF(float vx, float vy)
{
    ff_vx = vx;
    ff_vy = vy;
}

void PID_Crawl(uint8_t on)
{
    crawl = on ? 1 : 0;
}

/* 速度上限覆盖：0 = 用默认档（常规 / 蠕动）。只压上限，刹车包线不受影响。 */
static float v_max_ovr = 0.0f;

void PID_SetVmax(float v)
{
    v_max_ovr = (v > 0.0f) ? v : 0.0f;
}

static void pid_xy(float ex, float ey, float *out_vx, float *out_vy)
{
    float v_max   = (v_max_ovr > 0.0f) ? v_max_ovr : (crawl ? CHASSIS_V_CRAWL_CMPS : CHASSIS_V_MAX_CMPS);
    float a_acc   = CHASSIS_A_ACC_CMPS2;              /* 纯直行/混合：150 cm/s² */
    float a_brake = crawl ? CHASSIS_A_CRAWL_CMPS2  : CHASSIS_A_BRAKE_CMPS2;
    float ax = fabsf(ex);
    float ay = fabsf(ey);
    float vx = pid_x.kp * ex;
    float vy = pid_y.kp * ey;
    float dvx, dvy, em;
    float e_side, e_fwd;   /* 横移轴 / 直行轴 的误差绝对值（世界系）*/

    /* 纯横移判定：**横移轴**误差占绝对主导。
     * ⚠ 横移轴随出发边变：右基准车头沿任务 X ⇒ 横移是 Y；左基准车头沿任务 Y ⇒ 横移是 X。
     *   原来固定按 Y 判，左基准会把"直行"当成横移（参数一样时看不出来，一调就不对了）。*/
    e_side = follow_side ? ay : ax;
    e_fwd  = follow_side ? ax : ay;
    if (!crawl && e_side > 2.0f && e_side > 2.0f * e_fwd) {
        if (v_max_ovr <= 0.0f) v_max = CHASSIS_V_STRAFE_MAX_CMPS;   /* 有覆盖值时连横移也压到同一个上限 */
        a_acc = CHASSIS_A_STRAFE_CMPS2;
    }

    shrink_to(&vx, &vy, v_max);            /* ① 限速 |v| ≤ v_max */

    /* ② 刹车包线：|v| ≤ √(2·a_brake·|e|) */
    em = sqrtf(ex * ex + ey * ey);
    shrink_to(&vx, &vy, sqrtf(2.0f * a_brake * em));

    /* ②.5 跟踪目标速度前馈：直接叠加在位置环输出上，再由 v_max 和加速斜坡限制。 放在刹车包线之后，否则误差到 0 时包线会把前馈一起压成 0，运动目标永远追不上。 */
    vx += ff_vx;
    vy += ff_vy;
    shrink_to(&vx, &vy, v_max);

    /* ③ 斜坡只限加速，减速立即执行 —— 减速也被限则冲过刹车包线（kp=3 实测滑 26cm） */
    dvx = vx - vx_cmd;
    dvy = vy - vy_cmd;
#if (CHASSIS_ACC_RAMP_EN != 0)
    if (vx * vx + vy * vy > vx_cmd * vx_cmd + vy_cmd * vy_cmd) {          /* 在加速 */
        shrink_to(&dvx, &dvy, a_acc * CTRL_DT_S);                         /* 每拍加速上限 */
    }
#endif
    vx_cmd += dvx;
    vy_cmd += dvy;

    *out_vx = vx_cmd;
    *out_vy = vy_cmd;
}

/* 航向：w = kp × 相差角度 → 限幅 → 对称斜坡（这里限的就是角加速度本身） */
static float pid_yaw_step(float eyaw)
{
    float w = pid_yaw.kp * eyaw;
    float dw, lim;

    if (w >  CHASSIS_W_MAX_DPS) w =  CHASSIS_W_MAX_DPS;
    if (w < -CHASSIS_W_MAX_DPS) w = -CHASSIS_W_MAX_DPS;

    lim = GO_YAW_STEP_RPM / ROT_RPM_PER_DPS;      /* 轮 rpm → °/s（每拍允许的变化量） */
    dw  = w - w_cmd;
    if (dw >  lim) dw =  lim;
    if (dw < -lim) dw = -lim;
    w_cmd += dw;

    return w_cmd;
}

PidOut_t PID_Calc(float ex, float ey, float eyaw)
{
    PidOut_t out;

    pid_xy(ex, ey, &out.vx, &out.vy);
    out.w = pid_yaw_step(eyaw);

    return out;
}
