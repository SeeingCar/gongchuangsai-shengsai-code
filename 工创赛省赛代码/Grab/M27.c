#include "M27.h"
#include "grab_cfg.h"   /* FB_EXT_MAX_CM：速度模式的硬保护 */
#include "6020.h"        /* MoveCm 里设 6020 目标 */
#include "zdt_motor.h"
#include "usart.h"
#include "cmsis_os.h"       /* osDelay：同总线两帧间隔 5ms */
#include <math.h>
#include <stdio.h>

/* 地址：前后 = 电机2、上下 = 电机7（总线C）。 每轴一个**带符号**系数：0x36 角度每 1cm 变化多少度（cm 正方向 = 伸出/上升）。
 * 命令 = cm×系数；反馈 cm = 角度/系数 —— 两处共用同一个符号， 所以"某轴方向反了"只改它自己那一行系数的符号，命令和到位判定会一起正确。 */
#define FB_DEG_PER_CM       (-41.6f)       /* 前后(电机2)：1cm = 41.6°（实测 41.5° ⇒ 8.7cm/圈）*/
#define UD_DEG_PER_CM       (-180.0f)      /* 上下(电机7)：1cm = 180°（实测 359.9°/2cm ⇒ 2cm/圈）*/
#define M27_TOL_CM          0.12f          /* 到位容差 cm（0.05→0.12：UD 差 0.1cm 就算到位，别磨蹭）*/
#define M27_FRESH_MS        150           /* 角度多久没更新算掉线（20ms 读一次 ⇒ 连丢 7 次） */
#define FB_CM_MAX           FB_EXT_MAX_CM  /* 位置目标上限 = 软限位 19（原来 20 会开过软限位）*/
#define UD_CM_MAX           14.0f          /* 上下行程上限，待实测 */

static float   s_tar_fb = 0.0f, s_tar_ud = 0.0f;   /* 目标 cm */
float          cm_2     = 0.0f, cm_7     = 0.0f;   /* 实际 cm（2 = FB 伸缩 / 7 = UD 升降）*/
static int s_a = 0;                        /* 标志位：1 = 有新目标待下发 */
static int s_home_req = 0;                 /* 1 = 请求 M2/M7 回零，由 MotorC 发 */
volatile int m2_mode = 0;                  /* 0=M2位置模式，1=M2速度模式；仅MotorC发帧 */
static float   s_tar_speed = 0.0f;             /* M2 目标速度 cm/s（速度模式） */
static int s_acc_speed = 100;              /* M2 速度帧 acc */
static uint32_t s_last_speed = 0;              /* 上次 M2 速度帧时刻 */
static uint32_t s_last_send = 0;               /* 上次位置帧时刻，未到位 50ms 重发 */
static uint16_t s_rpm_fb = 900, s_rpm_ud = 900;   /* 两轴各自的速度档 */
static int  s_acc_fb = 100,       s_acc_ud = 100;         /* 两轴各自的加减速档 */

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float fb_cm_of(float d) { return d / FB_DEG_PER_CM; }
static inline float ud_cm_of(float d) { return d / UD_DEG_PER_CM; }

void M27_Init(void)
{
    s_tar_fb = s_tar_ud = 0.0f;
    cm_2  = cm_7  = 0.0f;
    s_a = 0;
    s_home_req = 0;
    m2_mode = 0;
    s_tar_speed = 0.0f;
    s_acc_speed = 100;
    s_last_speed = 0;
}

/* 只置请求，实际 0x9A 由 MotorC 的 M27_Send 发（UART5 单写者） */
void M27_Home(void)
{
    s_home_req = 1;
    s_a = 0;
    m2_mode = 0;
}

/* 手臂一次设三轴目标：6020 角度 + 前后(cm,rpm,acc) + 上下(cm,rpm,acc)。 **只改目标，不发帧**：6020 由 Motor6020 的 5ms 双环发，M2/M7 由 MotorC 的 M27_Send() 发。
 * ⚠ 三轴都必给：不想动的轴用 Grab6020_GetTarget() / cm_2 / cm_7 回填； 不想改速度就传 900 / 100（位置帧 rpm 0~3000、加减速档 0~255）。 */
void MoveCm(float deg, float ang_step,
            float fb_cm, uint16_t fb_rpm, uint8_t fb_acc,
            float ud_cm, uint16_t ud_rpm, uint8_t ud_acc)
{
    Grab6020_SetRateMax(0.0f);        /* 位置指令一律用默认最高速 */
    Grab6020_SetAngle(deg, ang_step);

    if (fb_cm < 0.0f) {
        s_tar_fb = cm_2;                     /* FB_HOLD_CM：原地保持（不钳位，免得被拉到 0.8）*/
    } else {
        s_tar_fb = clampf(fb_cm, FB_MIN_CM, FB_CM_MAX);
    }
    s_tar_ud = clampf(ud_cm, 0.0f, UD_CM_MAX);
    s_rpm_fb = fb_rpm; s_acc_fb = fb_acc;
    s_rpm_ud = ud_rpm; s_acc_ud = ud_acc;
    m2_mode  = 0;            /* 位置模式：MotorC 恢复发 M2 位置帧 */
    s_a      = 1;
}

/* 只看上下轴是否到位：抓取子步 0 只等 UD，FB 被 UD 机械推着走时别一起等 */
bool M27_UdArrived(void)
{
    return (fabsf(s_tar_ud - cm_7) <= M27_TOL_CM);
}

bool M27_Arrived(void)
{
    float e5, e6;

    if (!ZDT_AngleFresh(2, M27_FRESH_MS)) return false;
    if (!ZDT_AngleFresh(7, M27_FRESH_MS)) return false;

    e5 = s_tar_fb - cm_2;
    e6 = s_tar_ud - cm_7;
    return (e5 > -M27_TOL_CM && e5 < M27_TOL_CM) &&
           (e6 > -M27_TOL_CM && e6 < M27_TOL_CM);
}

/* 扫描只用 FB：UD 不参与，避免 UD 反馈暂时不新鲜时 M2 卡在 12/3 不回 */
bool M27_FbArrived(void)
{
    float e5;

    if (!ZDT_AngleFresh(2, M27_FRESH_MS)) return false;
    e5 = s_tar_fb - cm_2;
    return (e5 > -M27_TOL_CM && e5 < M27_TOL_CM);
}

void M27_Update(void)
{
    /* 0x36 的位置是驱动器的**绝对**角度（掉电保持）：直接换算成 cm，0cm = 驱动器零点。 所以目标 cm 是绝对量：同一个 cm 反复下发不会重复运动，和 6020 的角度语义一致。 */
    cm_2 = fb_cm_of(zdt_angle[2]);
    cm_7 = ud_cm_of(zdt_angle[7]);
}

void M2Speed(float cm_s, uint8_t acc)
{
    s_tar_speed = cm_s;
    s_acc_speed = acc;
    m2_mode     = 1;
}

/* 有新目标立即发；没到位时每 50ms 重发一次，防丢帧/回零期间目标被吞。 0xFD 是**绝对**位置帧（相对驱动器零点），s_tar 也是绝对 cm ⇒ 直接乘系数即可。 */
void M27_Send(void)
{
    float d2, d7;
    uint32_t now;

    if (s_home_req) {
        s_home_req = 0;
        ZDT_Home(2);
        osDelay(5);
        ZDT_Home(7);
        s_last_send = HAL_GetTick();
        return;
    }

    now = HAL_GetTick();

    if (m2_mode) {                    /* M2 速度模式：MotorC 是唯一发送者 */
        if ((now - s_last_speed) >= 50) {
            float cm_s = s_tar_speed;

            /* 到软限位只许往安全方向转 */
            if (cm_s > 0.0f && cm_2 >= FB_EXT_MAX_CM) cm_s = 0.0f;
            if (cm_s < 0.0f && cm_2 <= FB_MIN_CM)     cm_s = 0.0f;
            if (cm_s >  17.0f) cm_s =  17.0f;
            if (cm_s < -17.0f) cm_s = -17.0f;

            {
                float   deg_s = FB_DEG_PER_CM * cm_s;
                int dir   = (deg_s < 0.0f) ? 0x01 : 0x00;
                float   rpm   = fabsf(deg_s) / 6.0f;

                if (rpm > 120.0f) rpm = 120.0f;
                ZDT_SpeedCtrl(2, dir, (uint16_t)(rpm + 0.5f), s_acc_speed);
            }
            s_last_speed = now;
        }

        /* 速度模式下仍由 MotorC 维护 M7 位置帧 */
        if ((now - s_last_send) >= 50 &&
            fabsf(s_tar_ud - cm_7) > M27_TOL_CM) {
            ZDT_PosCtrl(7, UD_DEG_PER_CM * s_tar_ud, s_rpm_ud, s_acc_ud);
            s_last_send = now;
        }
        return;
    }

    if (!s_a) {
        if ((now - s_last_send) < 50) return;
        if (fabsf(s_tar_fb - cm_2) <= M27_TOL_CM &&
            fabsf(s_tar_ud - cm_7) <= M27_TOL_CM) return;
    }

    d2 = FB_DEG_PER_CM * s_tar_fb;
    d7 = UD_DEG_PER_CM * s_tar_ud;

    ZDT_PosCtrl(2, d2, s_rpm_fb, s_acc_fb);
    osDelay(5);
    ZDT_PosCtrl(7, d7, s_rpm_ud, s_acc_ud);
    s_a = 0;
    s_last_send = HAL_GetTick();
}
