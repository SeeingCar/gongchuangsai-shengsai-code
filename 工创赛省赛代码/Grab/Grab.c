#include "grab.h"
#include "grab_cfg.h"       /* 所有会改的数 */
#include "6020.h"
#include "M27.h"
#include "See.h"            /* 相机帧解析结果 */
#include "pwm.h"
#include "TaskFlow.h"
#include "Go.h"             /* GO_Arrived：判停要底盘也到位 */
#include "usart.h"
#include "cmsis_os.h"       /* osDelay */
#include <math.h>           /* fabsf */

/* 阀门 1（底盘）：0 = 一步不动；StartTask 置 1，不能由 TaskFlow 自己置（死锁）*/
int tf_gate = 0;
/* 阀门 2（手臂）：0 = 一步不动，StartTask 置 1；6020 伺服不受它影响（MotorTx 每 20ms 跑）*/
int grab_gate = 0;
/* 手臂状态机的当前步（case 号） */
int Grab_now = 1;     
/* 本 case 的指令是否已发过 */
static uint8_t s_sent     = 0;    
/* 跨状态机旗标（语义见 Grab.h）：抓取序号 / 抓完 / 判停 / 第二轮 / 环已对中 */
int grab_id = 0;
int tf_tracking = 0;   /* 底盘是否已在 case 2（由 TaskFlow 维护）*/
int grab_id_done = 0;
/* 三块料全抓完的锁存：1 ⇒ MotorRx 停掉周期打印（否则 case4~7 的数据会把抓取阶段的日志冲掉）。 只影响打印，不影响任何控制；Grab_Start() 或复位会清 0。 */
int grab_log_done = 0;
int grab_allow = 0;
int ground_allow = 0;
int grab_round2 = 0;
int grab_round3 = 0;   /* 底盘 → 手臂：1 = 第二遍开始（TF case 9 置 1，手臂 case 9 消费）*/
int ring_ok = 0;
/* 1 = 这是**第二轮**放料：case 7 只做 7-0~7-5（放三次），放完直接进 case 8， 不再做 7-6~7-9 的"回收三块"。由 case 8 → case 5 的交接（第二轮找环）置位。 */
static int s_no_pick = 0;
/* ---- case5 圆环 M2 对准（照 case2 骨架，目标静止）---- */
static uint32_t 	s_r5_fseq = 0;    /* 圆环帧序号（判新帧）*/
static uint32_t 	s_r5_tick = 0;    /* 上一有效帧时刻（算 dt）*/
static uint32_t 	s_r5_t    = 0;    /* 喂狗时刻 */
static uint32_t 	s_r5_mute = 0;    /* 45ms 静默门 */
static uint32_t 	s_r5_stuck= 0;    /* 顶死计时 */
static float    	s_r5_cm   = 0.0f; /* 上一拍 M2 长度（算臂速）*/
static float    	s_r5_v    = 0.0f; /* 上一拍速度指令（斜率限幅用）*/
static float    	s_r5_prev  = 0.0f; /* 上一帧 cy（突发帧判据用）*/
static int  		s_lock_first = 1;    /* 1 = 还没采信过任何一帧 */
/* case 1 伸收行程（FB 绝对 cm；起点 = case 0 的 1cm）*/

/* case 3 抓取 + 放回车上：8 个子步骤，每步"发一次指令 → 等到位"，非阻塞；角度/cm/拍数见 grab_cfg.h */
int            		grab_sub    = 0;    /* case 3 的子步骤号 0..7，每拍 +1 */
static uint16_t    	s_gb_sub_t0 = 0;    /* 子步骤的稳定倒计时（拍） */
static int     		s_gb_rot_wait = 0;  /* 等 6020 转动：到位后还要连着几拍才算停稳 */
static int     		s_lock_retry = 0;/* 1 = 失败重试后先把 6020/UD 摆回抓取姿态 */
static int     		s_gb_c4       = 0;  /* case 4 相位：0=抬UD 1=判色 2=回位 */
static uint32_t    	s_gb_stuck_t  = 0;  /* 顶住软限位且误差大的起始时刻（0 = 未顶死）*/
static uint32_t    	s_gb_ts   = 0;      /* case 3 子步 2：M2/UD 起始时刻 */
static uint32_t    	s_gb_sub_t = 0;     /* 当前子步骤开始时刻（卡住保护）*/
static int     		s_lock_resend    = 0;     /* 子步 0 已重发过一次 */
static int     		s_gb_sub_p = 0xFF;  /* 上一个子步骤号 */
static int     		s_place_i  = 0;     /* case7 放料：当前第几块 0..2 */
static int     		s_pick_i   = 0;     /* case7 回收：当前第几块 0..2 */
static float       	s_place_ang = 0.0f; /* 本轮目标环的 6020 标定角 */
static float       	s_place_m5  = 0.0f; /* 本轮目标环的 M2 标定长度 */
static uint32_t    	s_gb6_tick = 0;     /* K6 上一帧时刻 */
static uint32_t    	s_gb6_fseq = 0;     /* K6 圆环帧序号 */
static int     		s_gb6_tol  = 0;     /* K6 连续居中帧数 */
static uint32_t    	s_gb6_pd_t = 0;     /* K6 上一次 PD 调整时刻 */
static float       	s_gb6_ex_p = 0.0f;  /* K6 上一拍 cx 误差（PD 的 D） */
static float       	s_gb6_ey_p = 0.0f;  /* K6 上一拍 cy 误差（PD 的 D） */
static uint32_t    	s_gb6_dead_t = 0;   /* K6 每个 PD 子步的起始时刻（3s 兜底用） */

/* 抓取成功判定时实际读到的 6 帧颜色：每块料一组，未判前全 0。 */
static uint8_t s_chk_col[GRAB_ROUNDS][6] = {{0}};

/* ---- case 5（第二轮找圆环，FB 绝对 cm）---- */
static const float s_scan5_cm[2] = { FB_SCAN5_HI_CM, FB_SCAN5_LO_CM };
static int     		s_scan5_i  = 0;

/* ---- case 7（放料用的实测位置：6020 角度 + FB cm）---- */
float grab_k6_th   = 0.0f;   /* case6 现算：atan(15/10) 的角度值 ° */
float grab_k6_m2   = 0.0f;   /* case6 现算：√(M2₈² + 15²)，两个环的 M2 长度 */
static float       s_cal_ang[3] = { 0.0f, 0.0f, 0.0f };
static float       s_cal_m5[3]  = { 0.0f, 0.0f, 0.0f };

/* ---- case 8（放料：第几块料 / 槽位）---- */

/* 车上转盘（舵机 TIM4_CH2）：3 块料各一个角度 */
static const float s_carousel_ang[3] = { CAROUSEL_ANG0, CAROUSEL_ANG1, CAROUSEL_ANG2 };

/* ---- case 8（按放置顺序逐块抓料）---- */

/* ---- case 2 跟踪状态（速度模式）---- */
static uint32_t 	s_gb_fseq    = 0;     /* 处理到第几帧相机帧 */
static uint32_t 	s_gb_frame_t = 0;     /* 最近一次目标色新帧时刻（看门狗）*/
static uint32_t 	s_gb_tick    = 0;     /* 上一帧处理时刻（算 dt）*/
static uint32_t 	s_gb_mute    = 0;     /* 指令静默截止时刻 */

static float    	s_gb_prev_err = 0.0f; /* 上一帧 cam_cy */
static float    	cm_last  = 0.0f;      /* 上一帧实测 cm（算臂速）*/
static float    	s_gb_vobj     = 0.0f; /* 物料速度估计 cm/s（前馈）*/
static float    	s_gb_vcmd     = 0.0f; /* 上一帧速度指令（斜率限制）*/
static float    	s_m5_tar      = 3.0f; /* 当前 FB 位置 cm（抓取阶段用）*/
static int  		s_gb_first    = 0;    /* 1 = 本段第一帧 */
static int  		s_gb_tol_n    = 0;    /* A1 判停的连续帧计数 */
static uint32_t 	s_gb_notrack_t = 0;   /* case2：底盘不在跟踪状态的起始时刻（0 = 在跟踪）*/
static int8_t   	s_scan_dir   = 1;     /* 扫描方向 +1 = 往外（case 1 兜底用）*/
float 				grab_last_grab_cm = 0.0f; /* 上一次成功抓取那一刻的 M2 长度 cm */

/* ---- case 2 跟踪参数（初值 = grab_cfg.h；串口只剩 2/4 号方向开关，其余固定）---- */
/* case 2 的 M2 追踪方向（串口 2-1 / 2-0）：-1 = 默认（实测方向），+1 = 反过来。 只作用于 case 2 的速度律，不影响 case 5 的圆环对准，也不动相机极性。 */
static float 		s_t2_sgn     = -1.0f;
/* case 5（两轮共用）的 M2 对准方向（串口 4-1 / 4-0）：-1 = 默认（实测方向）。*/
static float 		s_r5_sy_sgn  = -1.0f;

float Grab_SetSign(uint8_t i, float v)
{
    float sg = (v >= 0.5f) ? 1.0f : -1.0f;

    if (i == 1) 	s_t2_sgn    = sg;      /* case2 的 M2 */
    else         	s_r5_sy_sgn = sg;      /* case5 的 M2 */

    return (sg > 0.0f) ? 1.0f : 0.0f;
}

/* ---- 环号 / 槽位映射（圆环对中 & 放料用）---- */

/* 在距离 d(cm) 内能停住的最大速度：解 v·τ + v²/(2a) = d v = a·(−τ + √(τ² + 2d/a)) a=s_t2_adec, τ=s_t2_tau 延迟 只算 v²/(2a) 会漏掉 v·τ 那一段（10cm/s 时就有 1.8cm），必然过冲。 */
static float gb_stop_speed(float d)
{
    float a = GB_TRACK_DECEL;
    float t = GB_TRACK_TAU;

    if (d <= 0.0f) return 0.0f;
    if (a < 10.0f) a = 10.0f;
    return a * (-t + sqrtf(t * t + 2.0f * d / a));
}

/* ④ 限位包线：方向相关的线性压速 —— 到上/下限还剩 d cm，就只许 d×K_LIM cm/s。 正速度朝 FB_EXT_MAX_CM、负速度朝 FB_MIN_CM；断帧保持也用它（不许盲推撞限位）。 */
static float gb_limit_envelope(float v, float cm)
{
    if (v > 0.0f) {
        float m = GB_K_LIM * (FB_EXT_MAX_CM - cm);
        if (m < 0.0f) m = 0.0f;
        if (v > m) v = m;
    } else if (v < 0.0f) {
        float m = GB_K_LIM * (cm - FB_MIN_CM);
        if (m < 0.0f) m = 0.0f;
        if (-v > m) v = -m;
    }
    return v;
}
/* 速度模式：立刻停臂（丢目标 / 退模式 / 进跟踪前都用） */
static float gb_tol(void)
{
    float cm = cm_2;

    /* 基准还没记下时用宽死区：先粗对准把基准记下来，之后回下面的常规死区 */
    if (!TaskFlow_BaseValid()) return (float)TF_FIRST_TOL_PX;

    /* 两端放宽：贴底时相机最近、比例尺最大、抖动折成像素最多；伸出到 17cm 以上时行程余量只剩 2cm，宁可不动也别来回蹭 */
    if (cm < GB_TOL_NEAR_M2CM) return GB_TOL_NEAR_CM;
    if (cm >= GB_TOL_FAR_M2CM)  return GB_TOL_FAR_CM;
    return (float)GB_TRACK_TOL_PX;
}

static void gb_m2_stop(void)
{
    float cm = cm_2;

    s_gb_vobj     = 0.0f;
    s_gb_vcmd     = 0.0f;
    M2Speed(0.0f, (uint8_t)GB_TRACK_ACC);

    /* 断帧停车时也要把臂拉出软限位：限位逻辑在“有新帧”分支里， 没帧就整段不跑 ⇒ 实测臂卡在 0.56cm（低于 1cm）却给 0，白白干等。 */
    if (tf_tracking && (cm < FB_MIN_CM)) {
        M2Speed(3.0f, (uint8_t)GB_TRACK_ACC);
    } else if (tf_tracking && (cm > FB_EXT_MAX_CM)) {
        M2Speed(-3.0f, (uint8_t)GB_TRACK_ACC);
    }
}
/* case 6 二维 PD：底盘已停，只调 6020/M2 两轴；cx、cy 连续 K6_ALIGN_FRAMES 帧都在 RING_K6_TOL_PX 内才算对准。返回 1 = 已写出 ang/m5。 */
/* 按环号取图像目标点：7/8/9 各一套，其它值按 8 号兜底 */
static float gb_ring_cx_tgt(int ring)
{
    if (ring == 7) return RING_CX_TGT_PX_7;
    if (ring == 9) return RING_CX_TGT_PX_9;
    return RING_CX_TGT_PX_8;
}

static float gb_ring_cy_tgt(int ring)
{
    if (ring == 7) return RING_CY_TGT_PX_7;
    if (ring == 9) return RING_CY_TGT_PX_9;
    return RING_CY_TGT_PX_8;
}

static uint8_t gb_k6_pd_align(uint8_t ring_no, float *out_ang, float *out_m5)
{
    float ex, ey, dex = 0.0f, dey = 0.0f, d_ang, d_cm, ang, m5;
    uint32_t now, dt;

    if (!See_IsNewRingFrame(&s_gb6_fseq) || cam_ring_no != ring_no) return 0;

    now = HAL_GetTick();
    /* 目标点用固定偏置 RING_*_TGT_PX（跟爪子中心/0 会随姿态漂）*/
    ex = (float)cam_ring_cx - gb_ring_cx_tgt(ring_no);
    ey = (float)cam_ring_cy - gb_ring_cy_tgt(ring_no);

    if (s_gb6_pd_t == 0) {
        s_gb6_ex_p = ex;
        s_gb6_ey_p = ey;
        s_gb6_pd_t = now;
        return 0;
    }
    dt = now - s_gb6_pd_t;

    if (fabsf(ex) <= (float)RING_K6_TOL_PX && fabsf(ey) <= (float)RING_K6_TOL_PX) {
        s_gb6_tol++;
        s_gb6_ex_p = ex;
        s_gb6_ey_p = ey;
        if (s_gb6_tol >= K6_ALIGN_FRAMES) {
            *out_ang = Grab6020_GetAngle();
            *out_m5  = cm_2;
            s_gb6_tol = 0;
            return 1;
        }
        return 0;
    }
    s_gb6_tol = 0;

    if (dt < K6_PD_PERIOD_MS) return 0;
    s_gb6_pd_t = now;

    if (dt > 5 && dt < 500) {
        dex = (ex - s_gb6_ex_p) * 1000.0f / (float)dt;
        dey = (ey - s_gb6_ey_p) * 1000.0f / (float)dt;
    }
    d_ang =  (K6_PD_ANG_KP * ex + K6_PD_ANG_KD * dex);
    if (d_ang >  K6_PD_ANG_STEP_MAX) d_ang =  K6_PD_ANG_STEP_MAX;
    if (d_ang < -K6_PD_ANG_STEP_MAX) d_ang = -K6_PD_ANG_STEP_MAX;
    d_cm = -(K6_PD_M2_KP * ey + K6_PD_M2_KD * dey);
    if (d_cm >  K6_PD_M2_STEP_MAX) d_cm =  K6_PD_M2_STEP_MAX;
    if (d_cm < -K6_PD_M2_STEP_MAX) d_cm = -K6_PD_M2_STEP_MAX;
    s_gb6_ex_p = ex;
    s_gb6_ey_p = ey;

    ang = Grab6020_GetTarget() + d_ang;
    m5  = cm_2 + d_cm;
    if (m5 < FB_MIN_CM) m5 = FB_MIN_CM;
    if (m5 > FB_EXT_MAX_CM) m5 = FB_EXT_MAX_CM;
    MoveCm(ang, 0.0f, m5, 300, 50, cm_7, 900, 100);
    return 0;
}
/* 抓取判定窗口：按 grab_id 存最近 6 帧颜色；第 2、3 块先剔除前面各块的颜色（第 3 块剔前两块），再往回凑满 6 帧。*/
static void gb_capture_chk(void)
{
    uint8_t tmp[6];
    int i;
    int skip = 0;
    int skip2 = 0;

    if (grab_id >= GRAB_ROUNDS) return;

#if (GB_CHK_SKIP_FIRST_EN != 0)
    if (grab_id >= 1) {                                 /* 第 2、3 块：第 1 块的色不算 */
        uint8_t f = (uint8_t)qr_c0[0];

        if (f >= 1 && f <= 6) skip = f;
    }
    if (grab_id >= 2) {                                 /* 第 3 块：第 2 块的色也不算 */
        uint8_t f = (uint8_t)qr_c0[1];

        if (f >= 1 && f <= 6) skip2 = f;
    }
#endif

    /* 只取"抬升 UD 完成之后"（s_gb_ts）的帧：绝不能拿抬升前地上的旧帧凑数 */
    See_RecentColorsSince(tmp, 6, skip, skip2, s_gb_ts);

    for (i = 0; i < 6; i++) s_chk_col[grab_id][i] = tmp[i];
}
/* case5：进入 M2 对准（子步 2）前把状态清干净。 ⚠ 不清 s_r5_t 的话它还是 0 ⇒ 一进子步 2 就被判"丢环超时"直接弹回巡逻。 */
static void r5_align_begin(void)
{
    s_r5_t     = HAL_GetTick();
    s_r5_tick  = HAL_GetTick();
    s_r5_mute  = 0;
    s_r5_stuck = 0;
    s_r5_cm    = cm_2;
    s_r5_v     = 0.0f;
    s_r5_fseq  = cam_ring_seq;
    s_lock_first = 1;
    s_r5_prev  = 0.0f;
}
/* 第二轮第二次放置（case 12）用的量 */
static int 		s_c12_i    = 0;      /* 第几次放（0→9 号、1→8 号、2→7 号）*/
static int 		s_lock_calc   = 0;      /* 1 = 理论值已算好 */
static uint32_t s_c12_wt  = 0;      /* 等底盘到位的起始时刻（20s 兜底用）*/
static float   	s_c1_ang[3] = {0.0f, 0.0f, 0.0f};  /* 第一轮第二次放置时的 7/8/9 角度（0=7 1=8 2=9）*/
static float   	s_c1_m5[3]  = {0.0f, 0.0f, 0.0f};  /* 同上：M2 长度 */
static float   	s_c12_th   = 0.0f;   /* θ（°）*/

/* 环号 → 转盘槽位（0/1/2）。 规则（用户 2026-10-31）：**颜色**由第 1 轮绑定决定（qr_c0a/qr_c1a）， **槽位**由第 2 轮抓料顺序决定（qr_c2）。
 * 例：156+123+516+231 ⇒ 9 号环的颜色 = 第1轮 9 号那个色 6 ⇒ 6 在 C 段第 3 个 ⇒ 槽 2（ANG2）。 */
static uint8_t gb_ring_slot(uint8_t ring)
{
    int i, j;
    for (i = 0; i < 3; i++) {
        if (qr_c1a[i] == (int)ring) break;
    }
    if (i >= 3) {                                   /* 二维码没配好：按环号本身兜底 */
        return (uint8_t)((ring >= 7 && ring <= 9) ? (ring - 7) : 1);
    }
    for (j = 0; j < 3; j++) {
        if (qr_c2[j] == qr_c0a[i]) return j;
    }
    return i;                                        /* 颜色在 C 段里找不到：用同序兜底 */
}
/* "上一次放进 8 号环的那块料"的颜色：查第 1 轮绑定（qr_c0a/qr_c1a）。*/
uint8_t Grab_Ring8Color(void)
{
    int i;

    for (i = 0; i < 3; i++) {
        if (qr_c1a[i] == 8) return (uint8_t)qr_c0a[i];
    }
    return 0;                                   /* 没配好：0 = 不切源（按环处理）*/
}
/* 7/9 号 M2 长度：按"15cm 等距一条线 + 支点横向偏置 s"的模型算（见 grab_cfg.h）。
 *   ring = 7 / 8 / 9；8 号直接返回实测值。M2₈ 取 s_cal_m5[1]（case 5/11 对准时记的）。*/
float Grab_K6M2Of(uint8_t ring)
{
    float m8 = s_cal_m5[1];
    float c  = K6_M2_OFF_CM;
    float s  = K6_M2_LAT_CM;
    float d8, h2, x;

    if (m8 < 2.0f) m8 = 10.0f;                 /* 反馈没上来：兜底 */
    if (ring == 8) return m8;

    d8 = c + m8;
    h2 = (d8 * d8) - (s * s);
    if (h2 < 1.0f) h2 = 1.0f;

    x = (ring == 7) ? (s + K6_RING_OPP_CM) : (s - K6_RING_OPP_CM);
    {
        float len = sqrtf(h2 + (x * x)) - c;
        if (ring == 7) len += K6_D7_M2_CM;      /* 7 号经验偏置 */
        else            len += K6_D9_M2_CM;      /* 9 号 */
        return len;
    }
}

static void gb_track_prep(void)
{
    uint32_t now = HAL_GetTick();

    s_gb_fseq     = cam_frame_seq;
    s_gb_frame_t  = now;                /* 看门狗从现在起算 */
    s_gb_tick     = now;
    cm_last       = cm_2;
    s_gb_tol_n    = 0;
    s_gb_stuck_t  = 0;
    s_gb_prev_err = (float)cam_cy;
    s_gb_first    = 1;
    s_gb_vcmd     = 0.0f;
    s_gb_vobj     = 0.0f;
}
/* 上电一次（osKernelStart 之前） */
void GRAB_Init(void)
{Grab6020_Init();     /* CAN1：滤波器 + 启动 + 使能 RX0 中断 */
 Servo_Init();        /* 舵机两路 PWM（上电默认张开） */
 M27_Init();          /* 手臂两轴：清状态 */}

/* 环号归一：1~3 号环 +6 成 7~9；已经是 7~9 的原样返回；其它值用 dflt 兜底（dflt = 7/8/9）。 */
static int gb_norm_ring(int r, int dflt)
{
    if (r >= 1 && r <= 3) return r + 6;
    if (r >= 7 && r <= 9) return r;
    return dflt;
}
/* 12 个参数：前 6 个 = 第一遍（3 块料颜色 + 3 个环号），后 6 个 = 第二遍（同样 3 色 + 3 环）。 环号支持写 1/2/3（自动 +6 成 7/8/9）或直接写 7/8/9。 */
void Set(uint8_t m1, uint8_t m2, uint8_t m3,
         int r1, uint8_t r2, uint8_t r3,
         int m4, uint8_t m5, uint8_t m6,
         int r4, uint8_t r5, uint8_t r6)
{
    qr_c0[0] = (int)m1;qr_c0[1] = (int)m2;qr_c0[2] = (int)m3;
    See_UpdateBlueMerge();

    qr_c1[0] = gb_norm_ring(r1, 7);qr_c1[1] = gb_norm_ring(r2, 8);qr_c1[2] = gb_norm_ring(r3, 9);

    qr_c2[0] = (int)m4;qr_c2[1] = (int)m5;qr_c2[2] = (int)m6;
	
    qr_c3[0] = gb_norm_ring(r4, 7);qr_c3[1] = gb_norm_ring(r5, 8);qr_c3[2] = gb_norm_ring(r6, 9);
	
    qr_c0a[0] = qr_c0[0]; qr_c0a[1] = qr_c0[1]; qr_c0a[2] = qr_c0[2];
    qr_c1a[0] = qr_c1[0]; qr_c1a[1] = qr_c1[1]; qr_c1a[2] = qr_c1[2];

    qr_has_data = 1;                 /* 手动值优先，后续扫码不再覆盖 */
    grab_id        = 0;
    grab_id_done   = 0;
    grab_round3    = 0;
    cam_target_id   = (int)qr_c0[0];
}
/* 切到"第二遍"：qr_c2/qr_c3 -> qr_c0/qr_c1，并清第二遍该清的标志。 若 qr_c2 无效（没配/没扫到），**不切换**、沿用第一遍颜色，避免第2遍目标色=0 永远找不到料。 */
void Grab_UseRound2(void)
{
    int i;

    if ((qr_c2[0] >= 1) && (qr_c2[0] <= 6)) {
        /* 覆盖之前先存快照：第二轮第二次放置要用 A/B 段查颜色↔环 */
        for (i = 0; i < 3; i++) {
            qr_c0a[i] = qr_c0[i];
            qr_c1a[i] = qr_c1[i];
        }
        for (i = 0; i < 3; i++) {
            qr_c0[i] = qr_c2[i];
            qr_c1[i] = qr_c3[i];
        }
        cam_target_id = (int)qr_c0[0];
    }
    /* 换到第二遍后必须重算 3/6 是否等价：qr_c0 刚被 qr_c2 覆盖，不重算就沿用第一遍的结果。
     * qr_c2 无效时 qr_c0 没变，重算结果与原来相同，无害。 */
    See_UpdateBlueMerge();
    /* qr_c2 无效时：保留 qr_c0/qr_c1（沿用第一遍），只做下面的标志清零 */

    /* 拍下"第一轮第二次放置时"用的 7/8/9 三组角度与 M2 长度（第二轮 case 6 会覆盖 s_cal_*）*/
    for (i = 0; i < 3; i++) {
        s_c1_ang[i] = s_cal_ang[i];
        s_c1_m5[i]  = s_cal_m5[i];
    }

    grab_id        = 0;
    grab_id_done   = 0;
    s_no_pick      = 0;
    s_place_i      = 0;
    s_pick_i       = 0;
    grab_log_done     = 0;      /* 重新打开周期打印 */
    s_scan5_i      = 0;
    s_gb_tol_n     = 0;
    s_lock_retry = 0;
    s_gb_c4        = 0;
    s_place_ang    = 0.0f;
    s_place_m5     = 0.0f;

    /* 判色帧缓存要清：否则第 2 遍第一块料抓之前，串口 V 行还显示第 1 遍的颜色（会看错）*/
    {
        int m, k;
        for (m = 0; m < GRAB_ROUNDS; m++) {
            for (k = 0; k < 6; k++) s_chk_col[m][k] = 0;
        }
    }

    /* ⚠ 这里**不能**清 grab_sub / s_sent：本函数是在 TF case 9-2 里被调的， 此刻手臂正停在 GB case 9 的子步 2 等 grab_round3；一清子步它会被打回 0， 重新下发一遍收工位命令（多花 1~2 秒，两台机的时序就错开了）。
     * 子步/下发标志由 GB case 9 的子步 2 自己清。 */
}
/* StartTask 的起点岔路走完后调 */
void Grab_Start(void)
{
    /* 全流程：从 case 0 摆位开始，但**要先等阀门**。 TaskFlow 先跑 case -1/0（路线 + 二维码 + 去 15 号格），跑完置 grab_gate=1 → 本机 case 0 摆到 90°/1cm/UD 0 → 摆完置 tf_gate=1 放底盘 → 两边一起进 case
     * 1（底盘找料 / 手臂扫描）。 case 1 扫描 → case 2 跟踪 → A1 判停 → case 3 抓取。 */
    Grab_now   = 0;
    s_sent       = 0;                    /* 清指令记忆：case 0 会重发摆位命令 */
    grab_allow = 0;
    ground_allow   = 0;
    s_lock_retry = 0;
    s_gb_mute    = 0;
    grab_log_done   = 0;                    /* 新一轮开始：重新打开周期打印 */
    s_no_pick    = 0;                    /* 第一轮：case 7 放完还要回收三块 */
    s_m5_tar     = cm_2;

    {
        int i, j;
        for (i = 0; i < GRAB_ROUNDS; i++) {
            for (j = 0; j < 6; j++) s_chk_col[i][j] = 0;
        }
    }
}
/* 整条流程走完、且真的停在收工位（case 4：6020=180°、FB=1cm，见 4-0 的 MoveCm）。 ⚠ 必须同时判 grab_sub == 2：Grab_now=4 是在 case 3 子步 4 置的，那一拍手臂还在 90°。 */
bool Grab_Done(void)   { return (Grab_now == 4 && grab_sub == 2); }
uint8_t Grab_ChkFrame(uint8_t mat, uint8_t i)
{if (mat >= GRAB_ROUNDS || i >= 6) return 0;
    return s_chk_col[mat][i];}

float Grab_CalAng(uint8_t i)
{if (i >= 3) return 0.0f;
    return s_cal_ang[i];}

float Grab_CalM5(uint8_t i)
{if (i >= 3) return 0.0f;
    return s_cal_m5[i];}

void GRAB_Update(void)
{
	//门锁
    if (!grab_gate) return;
    switch (Grab_now) {
	/* 6020 到 90°、FB 到 1cm、UD 到 0：同拍一起发、一起等 */	
    case 0:   
        m2_mode = 0;   	/* M2 位置模式 */              
        if (!s_sent) {
			Gripper_Open();
            /* 一开始将第一个目标颜色带入cam_target_id */
            if (grab_id == 0) cam_target_id = qr_c0[0];
            MoveCm(90.0f, 0.0f, 1.0f, 900, 100, 0.0f, 200, 0);
            s_gb_ts = HAL_GetTick();
            s_sent  = 1;
        } else if (Grab6020_Arrived_Big() ||((HAL_GetTick() - s_gb_ts) >= 3000)) {
            Grab_now = 1;
            s_sent   = 0;
            tf_gate  = 1;
        }
        break;
	/* 速度模式M2扫描 */
    case 1: 
		m2_mode = 1;    /* M2 速度模式 */		
		//触发条件：抓取失败
        if (s_lock_retry) {
            if (!s_sent) {
                MoveCm(90.0f, 0.0f, cm_2, 900, 100, 0.0f, 900, 100);
                s_gb_ts = HAL_GetTick();
                s_sent = 1;
                break;
            }
            /* 到位或超时都往下走（理论 0.9s + 0.5s）*/
            if ((!Grab6020_Arrived_Big() || !M27_UdArrived()) &&((HAL_GetTick() - s_gb_ts) < 2800)) break;
            s_lock_retry = 0;
            s_scan_dir      = 1;
            s_sent          = 1;
        }
		//看见目标进入追踪模式
        if (See_IsTarget(cam_target_id)) {
            gb_track_prep();             /* 复位帧号/前馈；**不停臂**，速度指令连续 */
            Grab_now = 2;
            break;
        }
		//判断初始M2巡逻方向
        if (!s_sent) {
            MoveCm(90, 0.0f, cm_2, 900, 100, 0.0f, 900, 100);
            s_scan_dir = (grab_last_grab_cm < TF_M2_GRAB_CM) ? +1 : -1;
            s_sent     = 1;
        } else {
            float cm = cm_2;
            /* 巡逻期间手臂在 GB_PATROL_M2_NEAR_CM(4) ↔ FAR_CM(14) 之间往返： 到 4cm ⇒ 换向往外 到 14cm ⇒ 换向收回 初始方向在"第一次进来"那里按上一次抓取 M2 定好了。 */
            if (cm <= GB_PATROL_M2_NEAR_CM) s_scan_dir = +1;
            if (cm >= GB_PATROL_M2_FAR_CM)  s_scan_dir = -1;
            M2Speed(GB_SCAN_CMPS * (float)s_scan_dir, 100);
        }
        break;
	/* 跟踪 cam_cy（只动 M2）—— 速度模式，7 个环节见文件头 */
    case 2:   
    {
        m2_mode = 1;                 /* M2 速度模式 */
        int  new_frame;
        int  is_target;
        uint32_t fseq_save;
        /* 底盘已判定“可以抓”才允许进 case 3；A1 只负责把 grab_allow 置 1。 */
        if (grab_allow && ground_allow) {
            ground_allow = 0;
            grab_last_grab_cm = cm_2;   /* 抓取这一刻的长度，供抓完后巡逻用 */
            Grab_now = 3;
            s_sent     = 0;
            grab_sub   = 0;
            gb_m2_stop();
            break;
        }
        /* 底盘不在 case 2 时手臂不能一直守着（否则两边互等、没人抓）：给 500ms 缓冲后回 case 1。*/
        if (!tf_tracking) {
            if (s_gb_notrack_t == 0) {
                s_gb_notrack_t = HAL_GetTick();
            } else if ((HAL_GetTick() - s_gb_notrack_t) >= 500) {
                s_gb_notrack_t = 0;
                s_gb_tol_n = 0;
                s_sent     = 0;
                grab_sub   = 0;
                grab_allow = 0;          /* 握手成对清：否则底盘会拿陈旧 grab_allow 直接判"已对准" */
                gb_m2_stop();
                Grab_now   = 1;          /* 回扫描，等底盘重新进 case 2 */
                break;
            }
        } else {
            s_gb_notrack_t = 0;          /* 底盘在跟踪：计时清零 */
        }
        /* 速度模式看门狗：速度指令是“持续生效”的，一旦不再刷新就会一直转。 必须**先喂狗再判**，否则首帧之前就被判超时 → 永远进不来（就是“卡住不动”）。 */
        fseq_save = s_gb_fseq;                  /* 先存一份：被静默门挡住时把这一帧还回去 */
        new_frame = See_IsNewFrame(&s_gb_fseq);
        is_target = See_IsTarget(cam_target_id);
        if (new_frame && is_target) s_gb_frame_t = HAL_GetTick();
        /* 缺帧 / 异色分两档：≤200ms：保持上次速度，但**必须过 ④ 限位包线**再发（否则盲推撞限位） >200ms：停车 + 清判停计数；>700ms：回 case 1 重新扫描 */
        if (!(new_frame && is_target)) {
            uint32_t age = HAL_GetTick() - s_gb_frame_t;

            if (age <= GB_LOST_HOLD_MS) {
                M2Speed(gb_limit_envelope(s_gb_vcmd, cm_2), (uint8_t)GB_TRACK_ACC);
                break;
            }
            gb_m2_stop();
            s_gb_tol_n = 0;
            s_gb_first = 1;
            if (age > GB_LOST_RETURN_MS) {
                s_sent   = 0;
                grab_allow = 0;      /* 握手成对清：否则底盘 case 3 以为"手臂还在抓"，白等到超时 */
                Grab_now = 1;
            }
            break;
        }
        /* 45ms 静默门（相机 20Hz）：速度模式改的是“目标速度”，不会重启位置曲线。*/
        if ((int32_t)(HAL_GetTick() - s_gb_mute) < 0) {
            s_gb_fseq = fseq_save;   /* 把这一帧还回去：See_IsNewFrame 已经认领过序号，
                                      *   不还就等于被门吞掉 ⇒ 相机帧率一快就丢一半帧 */
            break;
        }
           /* ===== 一个“有效目标帧”的 7 个环节 ===== */
            uint32_t    now    = HAL_GetTick();
            uint32_t    dt_ms  = now - s_gb_tick;
            const float k_px   = cam_px_to_cm_y;
            int         cy     = cam_cy;                      /* y 方向误差 px */
            float       d_raw  = (float)cy - s_gb_prev_err;   /* 与上一帧的像素差 */
            float       cm     = cm_2;               /* M2 实际伸出 cm */
            float       v_arm  = 0.0f;                        /* 臂实测速度 cm/s */
            float       e_cm, tol_cm, v_ff, v, v_all, v_rel;
            int     first  = s_gb_first;
            int     hold0  = 0;                           /* 本帧被强制置 0 */

            /* 不可信帧丢弃：帧间隔很短、定位却跳很大 ⇒ 突发帧/串色，整帧不采信。 计时照走（否则 dt 越积越大），下一拍再来。 */
            if (!first && dt_ms < GB_JUMP_DT_MS && fabsf(d_raw) > GB_JUMP_PX) {
                s_gb_tick = now;
                break;
            }

            s_gb_tick = now;

            if (dt_ms > 8 && dt_ms < 250) v_arm = (cm - cm_last) * 1000.0f / (float)dt_ms;
            cm_last = cm;
            if (dt_ms >= 8) s_gb_prev_err = (float)cy;   /* 突发帧不更新基准，避免下一次差分被污染 */

            if (first) {                 /* 第一帧不算速度 */
                d_raw      = 0.0f;
                s_gb_first = 0;
                s_gb_vcmd  = 0.0f;
            }

            /* 已删机制（实测更差）：末段软着陆、D 项、lead 预测 / 捕获模式 / 半速档 / 软限位保持带。*/
            /* 原来这里算过一个"物料图像速度 w_cm"并拿它当判停门（要求 ≤1cm/s）。 实测该门槛过严：物料以 1.2~3.9cm/s 穿窗时永远不满足 ⇒ 永远不判停。 且 dt 只有 66~80ms，1px 量化噪声就有 1.2cm/s，指标本身不可靠。
             * 已删除；判停改为**连续 N 帧都在死区**（N = STOP_CONFIRM_FRAMES = 3）。 */
            e_cm   = (float)cy * k_px;         /* 正 = 需伸出 */
            tol_cm =  gb_tol() * k_px;

            /* ① 物料速度估计：v_meas = 臂速 − 像素速度×比例尺；变大慢跟、变小/反向快收 */
            if (dt_ms > 8 && dt_ms < 250) {
                float v_meas = v_arm - (d_raw * 1000.0f / (float)dt_ms) * k_px;

                if (fabsf(v_meas) < GB_TRACK_VOBJ_GATE_CMPS) v_meas = 0.0f;

                if (first) {
                    s_gb_vobj = 0.0f;
                } else if (fabsf(v_meas) > 25.0f) {
                    /* 野值：不更新 */
                } else {
                    float dv = v_meas - s_gb_vobj;

                    /* 每帧变化限幅 ±10cm/s（挡单帧跳变，挡不住真运动）*/
                    if (dv >  10.0f) dv =  10.0f;
                    if (dv < -10.0f) dv = -10.0f;

                    {
                        float a_up = 1.0f - expf(-(float)dt_ms / (GB_VOBJ_TAU_UP_S * 1000.0f));
                        float a_dn = 1.0f - expf(-(float)dt_ms / (GB_VOBJ_TAU_DN_S * 1000.0f));

                        if (a_up < 0.02f) a_up = 0.02f;
                        else if (a_up > 0.65f) a_up = 0.65f;
                        if (a_dn < 0.02f) a_dn = 0.02f;
                        else if (a_dn > 0.65f) a_dn = 0.65f;

                        if ((v_meas * s_gb_vobj) < 0.0f || fabsf(v_meas) < fabsf(s_gb_vobj)) {
                            s_gb_vobj += a_dn * dv;
                        } else {
                            s_gb_vobj += a_up * dv;
                        }
                    }
                }            
            if (s_gb_vobj > -0.3f && s_gb_vobj < 0.3f) s_gb_vobj = 0.0f;

            v_ff = GB_TRACK_FF * s_gb_vobj;
            if (v_ff >  GB_FF_MAX_CMPS) v_ff =  GB_FF_MAX_CMPS;
            if (v_ff < -GB_FF_MAX_CMPS) v_ff = -GB_FF_MAX_CMPS;

            /* ② 位置项：唯一反馈项（正号：cy>0 要伸出；实测方向）*/
            v = v_ff + s_t2_sgn * GB_TRACK_KV * e_cm;      /* 方向由 s_t2_sgn 定 */

            /* ③ 刹车包线：只夹“相对物料速度”的修正量 */
            v_all = GB_TRACK_ENV_K * gb_stop_speed(fabsf(e_cm));
            v_rel = v - v_ff;
            if (v_rel >  v_all) v_rel =  v_all;
            if (v_rel < -v_all) v_rel = -v_all;
            v = v_ff + v_rel;

            /* 总指令也要能刹住：刹车距离由整条 v 决定，旧写法让 v_ff 绕过包线（超 2.4 倍 ⇒ 冲过 3cm）。*/
            if (v >  v_all) v =  v_all;
            if (v < -v_all) v = -v_all;

            /* ④ 限位包线：唯一防越界机制（方向相关，线性 K_LIM cm/s 每 cm）*/
            v = gb_limit_envelope(v, cm);

            /* ⑤ 实测超速刹车：分两种情形，判据都是"实测速度已超过剩余距离能刹住的速度"。 (a) 物料速度前馈 |v_ff| 小于判据切换门（1.0cm/s）：物料基本不动 ⇒ 臂速≈相对速度，双方向都判。
              * 作用：抓住"臂正朝目标冲得太快"这个真正导致静态过冲的形态 （仿真：最坏 plant 静止过冲 0.54 → 0.04cm）。 (b) 物料在动 ⇒ 只判"臂正朝让 |e| 变大的方向跑"。绝不能双方向都判：
              * 物料在动时臂必须比"剩余误差能刹住的速度"跑得更快才追得上， 双方向判会把正确追赶误判成超速（仿真：2.3 次/秒误切、判停 4.68s、 5cm/s 迎面过冲 1.18 → 2.06cm）。 阈值用
              * gb_stop_speed 的 k=1（理论边界），不乘 ENV_K：指令侧 ③ 已按 0.7×stop 夹紧，这里必须留出更宽阈值，否则与 ③ 同步触发 ⇒ 每帧"切断→重新加速"抖动。 */
            {
                float dv_lim = gb_stop_speed(fabsf(e_cm));

                if (fabsf(v_ff) < GB_BRK_VFF_CMPS) {
                    if (fabsf(v_arm) > dv_lim) { v = 0.0f; hold0 = 1; }
                } else {
                    /* 符号跟着位置项一起翻：es>0 要伸出，臂却在往回收 ⇒ 误差变大，切断 */
                    {
                        float es = e_cm * s_t2_sgn;

                        if (es > 0.0f && v_arm < -dv_lim) { v = 0.0f; hold0 = 1; }
                        if (es < 0.0f && v_arm >  dv_lim) { v = 0.0f; hold0 = 1; }
                    }
                }
            }

            /* ⑥ 死区：死区内一律 0（不留前馈）*/
            if (fabsf(e_cm) < tol_cm) { v = 0.0f; hold0 = 1; }

            /* ⑦ 总饱和 → 斜率 → 输出 */
            if (v >  GB_VMAX_CMPS) v =  GB_VMAX_CMPS;
            if (v < -GB_VMAX_CMPS) v = -GB_VMAX_CMPS;
            {
                float lim = 0.15f * (float)dt_ms;

                if (lim < 3.0f)  lim = 3.0f;
                if (lim > 10.0f) lim = 10.0f;
                if (v > s_gb_vcmd + lim) v = s_gb_vcmd + lim;
                if (v < s_gb_vcmd - lim) v = s_gb_vcmd - lim;
            }
            /* ⑤/⑥ 给的 0 是硬 0：斜率不许把它拉回去（否则死区里还带着 ~9cm/s 冲过去）*/
            if (hold0) v = 0.0f;
            if (v > -0.3f && v < 0.3f) v = 0.0f;

            s_gb_vcmd = v;                  /* 所有限幅/刹车/限位都处理完了才存 */
            M2Speed(s_gb_vcmd, (uint8_t)GB_TRACK_ACC);
            s_gb_mute = HAL_GetTick() + GB_TRACK_MUTE_MS;

            /* 顶死检测：顶在软限位且误差仍大 ⇒ 物理够不着，连续 1s 判死回 case 1。窗口须 ≥1.0cm，否则 chatter。*/
            if ((cm >= FB_EXT_MAX_CM - 1.0f || cm <= FB_MIN_CM + 1.0f) &&
                fabsf((float)cy) > 41.0f) {   /* 640×480：旧 25px@20px/cm = 1.25cm，新分辨率 1.25×33 = 41px */
                if (s_gb_stuck_t == 0) {
                    s_gb_stuck_t = now;
                } else if ((now - s_gb_stuck_t) >= GB_STUCK_MS) {
                    s_gb_stuck_t = 0;
                    s_sent       = 0;
                    grab_allow   = 0;       /* 握手成对清（本文件规则：退出路径必须清）*/
                    gb_m2_stop();           /* 速度指令是持续生效的，这条分支原来漏了停车 */
                    Grab_now     = 1;       /* 够不着：回扫描重新找机会 */
                }
            } else {
                s_gb_stuck_t = 0;
            }

            /* cy 判据（免判**只服务于"量基准"**，抓取时必须真进 cy 门）：
             *   基准已定                        → 一律判 gb_tol() 死区（**不允许任何免判**）；
             *   基准未定 & M2 < TF_M2_CY_OFF_CM → 车已扑过头、臂顶在软限位，cy 收不回来 ⇒ 免判（只为量基准）；
             *   基准未定 & M2 < 16cm            → 宽死区(30px)；
             *   基准未定 & M2 ≥ 16cm            → 够不着 ⇒ 免判（否则永远量不到基准）。*/
            {
            int cy_ok;

            if (TaskFlow_BaseValid())          cy_ok = (fabsf(e_cm) <= tol_cm);
            else if (cm < TF_M2_CY_OFF_CM)     cy_ok = 1;
            else if (cm < 16.0f)               cy_ok = (fabsf(e_cm) <= tol_cm);
            else                               cy_ok = 1;

            /* A1 判停（只留这一条）：位置进死区 + 横向进 A1 门 + **跟得上物料** + 指令小。 判的是“是否跟得上物料”（|v_arm − v_ff|），不是“是否静止”—— 物料在动时臂本来就不该静止。 */
            if (tf_tracking &&
                cy_ok &&
                fabsf((float)cam_cx - cam_grip_cx_px) <=
                    (TaskFlow_BaseValid() ? (float)A1_CX_TOL_PX : (float)TF_FIRST_TOL_PX) &&
                fabsf(v_arm) <= GB_STOP_V_CMPS &&             /* 手臂停住 */
                fabsf(v)     <= GB_STOP_V_CMPS &&
                /* 判基准阶段（还没记下基准）额外要求**车也几乎停住**： 否则车一边晃一边"凑齐 3 帧"，很容易在追击途中被误判成基准。 记下基准之后不再要求（跟踪时车本来就在动）。 */
                (!TaskFlow_BaseValid() ? (fabsf(TaskFlow_CarV()) <= TF_BASE_V_CMPS) : 1) ) {
                s_gb_tol_n++;
            } else {
                s_gb_tol_n = 0;
            }
            }

            if (s_gb_tol_n >= STOP_CONFIRM_FRAMES) {
                grab_allow = 1;
                s_sent       = 0;
                grab_sub     = 0;
                s_gb_tol_n   = 0;         /* 触发后清零：否则计数停在阈值上，之后任意单帧都能再次触发 */
                gb_m2_stop();
                if (ground_allow) {        /* 底盘已放行（正常会在下一拍走上面的入口）*/
                    ground_allow = 0;
                    Grab_now = 3;
                }
            }

            s_m5_tar = cm;
        }
    }
        break;
	/* 抓取 + 放回车上：5 个子步，每步"发一次指令 → 等到位" */
    case 3:   
        m2_mode = 0;                 /* M2 位置模式 */
        if (grab_sub != s_gb_sub_p) {
            s_gb_sub_p = grab_sub;
            s_gb_sub_t = HAL_GetTick();
        } else if ((HAL_GetTick() - s_gb_sub_t) > 6000) {
            s_sent     = 0;
            s_gb_sub_p = 0xFF;
            if (grab_sub < 4) {
                grab_sub++;
            } else {
                /* ⚠ 最后一步仍超时：走**失败路径**回 case 1 重找这块料。 原来这里无条件下 grab_sub++ ⇒ 变成 5，而下面的 switch 只有 0..4， 于是每 6s 再 +1、**永远退不出 case
                 * 3**（底盘一起被卡在 case 3）。 */
                grab_sub     = 0;
                grab_allow   = 0;
                s_gb_c4      = 0;
                s_lock_retry = 1;
                Grab_now     = 1;
                break;
            }
            break;
        }

        switch (grab_sub) {

        case 0:   /* 3-0 ① 转盘先转到本块料的槽位 ② 降 UD 到物料高度（FB 保持跟踪到的位置） */
            if (!s_sent) {
                s_lock_resend = 0;
                /* 抓取是有顺序的：第 1 块 → ANG0，第 2 块 → ANG1，第 3 块 → ANG2。 舵机转动要 ~0.4s，正好和降 UD 并行，等放料时槽位已经就位。 */
                Servo2_SetAngle(s_carousel_ang[grab_id]);   /* grab_id 在这儿只可能是 0..2 */
                /* 抓取位置 = **现在的实际位置**。用判停那一帧的 s_m5_tar 会把臂往回拉 （判停后臂还会滑一点，旧目标在它后面 ⇒ 臂“前后动”）。 */
                s_m5_tar = cm_2;
                if (s_m5_tar < FB_MIN_CM)    s_m5_tar = FB_MIN_CM;
                if (s_m5_tar > FB_EXT_MAX_CM) s_m5_tar = FB_EXT_MAX_CM;
                MoveCm(Grab6020_GetTarget(), 0.0f, s_m5_tar, 300, 50, (5.15f - (g_high - 8.0f)), 750, 0);
                s_sent = 1;
                s_gb_sub_t0 = GRAB_STOP_SETTLE_TICKS;   /* 停抓只稳 1 拍，别等 200ms */
                break;
            }
            /* 800ms 还一格没动 ⇒ 位置帧可能丢了，重发一次（实测有过 3.2s 不动的）*/
            if (!s_lock_resend && (HAL_GetTick() - s_gb_sub_t) > 1600 && !M27_UdArrived()) {
                s_lock_resend = 1;
                MoveCm(Grab6020_GetTarget(), 0.0f, s_m5_tar, 300, 50, (5.15f - (g_high - 8.0f)), 3000, 250);
                break;
            }
            if (!M27_UdArrived()) break;                 /* 只等 UD：UD 下降会机械推 FB，一起等会卡住 */
            if (s_gb_sub_t0) { s_gb_sub_t0--; break; }   /* 到位后再稳一会儿才合爪 */
            s_sent = 0; grab_sub = 1;
            break;

        case 1:   /* 3-1 合爪：发一次，等 500ms 夹紧（非阻塞）*/
            if (!s_sent) {
                Gripper_Close();
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if ((HAL_GetTick() - s_gb_ts) < 500) break;
            s_sent = 0;
            grab_sub = 2;
            break;

        case 2:   /* 3-2 三轴一步到位：M2→5cm、UD→1.5cm、6020→放料角 */
            if (!s_sent) {
                MoveCm(M6020_PLACE_DEG, 3.15, FB_SAFE_CM, 660, 200, 2.6f, 1800, 250);
                Grab6020_SetRateMax(189);           /* 189°/s ≈ 0.65s 转完 106° */
                s_gb_ts     = HAL_GetTick();
                s_gb_sub_t0 = GRAB_SETTLE_TICKS;
                s_sent      = 1;
                break;
            }
            if ((!M27_Arrived() || !Grab6020_Arrived_Big()) &&
                ((HAL_GetTick() - s_gb_ts) < 3600)) break;
            if (s_gb_sub_t0) { s_gb_sub_t0--; break; }
            s_sent   = 0;
            grab_sub = 3;
            break;

        case 3:   /* 3-3 张爪放料：发一次，下一拍直接进子步 4（不等待）*/
            if (!s_sent) {
                Gripper_Open();
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            s_sent = 0;
            s_gb_c4  = 0;                  /* 抓取成功判断挪到子步 4（抬完 UD 再判）*/
            grab_sub = 4;
            break;

        case 4:   /* 3-4 ① 先把 UD 抬到 0 ② 取最近 6 帧判抓取是否成功 ③ 成功了才回位 */
            switch (s_gb_c4) {

            case 0:   /* 3-4-0 ① 只抬升降轴到 0；FB / 6020 原地不动 */
                if (!s_sent) {
                    MoveCm(Grab6020_GetTarget(), 0.0f, cm_2, 300, 50, 0.5f, 800, 250);
                    s_gb_ts = HAL_GetTick();
                    s_sent  = 1;
                    break;
                }
                if (!M27_UdArrived()) break;
                s_sent  = 0;
                s_gb_c4 = 1;
                s_gb_ts = HAL_GetTick();
                break;

            case 1:   /* 3-4-1 ② 抬升后**等满 500ms**，再数这 500ms 窗口里目标色有几帧：> 2 帧才算抓到 */
                if ((HAL_GetTick() - s_gb_ts) < 500) break;

                {
                    int i, hit = 0;

                    gb_capture_chk();               /* 只取这 500ms 窗口内的帧（见 gb_capture_chk）*/
                    if (grab_id < GRAB_ROUNDS) {
                        for (i = 0; i < 6; i++) {
                            if (See_SameColor(s_chk_col[grab_id][i], cam_target_id)) hit++;
                        }
                    }
                    if (hit > 2) {                 /* 6 帧里目标色超过 2 帧 ⇒ 抓到 */
                        s_gb_c4 = 2;
                        s_sent  = 0;
                        break;
                    }
                }

                /* 失败 = 满了 500ms 且窗口内目标色不到 3 帧：回 case1 重新找这块料 */
                s_gb_c4      = 0;
                grab_sub     = 0;
                s_sent       = 0;
                grab_allow    = 0;
                s_lock_retry = 1;
                Grab_now     = 1;
                break;

            case 2:   /* 3-4-2 ③ 抓到才回位：6020→90° / FB→4cm / UD→0
                       *   但**最后一块不回位**，直接进 case 4 收工位：
                       *     原来会先摆到 90°/4cm，再摆到 180°/1cm，多花 1~2s，
                       *     而且肉眼看就是"头先摆过去又摆回来"，很不自然。*/
                if (!s_sent) {
                    if ((uint8_t)(grab_id + 1) >= GRAB_ROUNDS) {
                        grab_id      = GRAB_ROUNDS;   /* 三块齐了 */
                        grab_id_done = 1;             /* 立刻通知底盘 → 它直接进 case 4 */
                        grab_allow   = 0;
                        grab_sub     = 0;
                        s_sent       = 0;
                        s_gb_c4      = 0;
                        grab_log_done   = 1;             /* 停周期打印，保住抓取阶段的日志 */
                        Grab_now     = 4;             /* 直接收工位（180°/1cm/UD0）*/
                        break;
                    }
                    MoveCm(90.0f, 0.0f, 4.0f, 1800, 250, 0.0f, 800, 200);
                    s_gb_ts       = HAL_GetTick();
                    s_sent        = 1;
                    s_gb_rot_wait = 0;
                    break;
                }
                if ((Grab6020_Arrived_Big() && M27_FbArrived()) ||
                    ((HAL_GetTick() - s_gb_ts) >= 2800)) {
                    if (s_gb_rot_wait < GRAB_SETTLE_TICKS) {
                        s_gb_rot_wait++;
                    } else {
                        grab_id++;                 /* 这块料搞定，换下一块（只会是第 1、2 块）*/
                        cam_target_id = qr_c0[grab_id];
                        grab_id_done = 1;
                        grab_allow   = 0;          /* 本轮结束，清握手，避免下一轮看到旧1 */
                        grab_sub     = 0;
                        s_sent       = 0;
                        s_gb_c4      = 0;
                        Grab_now = 0;              /* 回 case 0 摆姿态，等下一块 */
                    }
                }
                break;

            }
            break;
        }
        break;

    case 4:   /* 收工：6020→180°、FB→1cm、UD→0（以 4-0 的 MoveCm 为准）；摆好后停在这儿等底盘放行第二轮 */
        m2_mode = 0;                 /* M2 位置模式 */
        switch (grab_sub) {
        case 0:   /* 4-0 下发收工位（6020 180°、FB 1cm、UD 0） */
            MoveCm(180.0f, 0.0f, 1.0f, 900, 100, 0.0f, 900, 100);
            s_gb_ts  = HAL_GetTick();
            grab_sub = 1;
            break;
        case 1:   /* 4-1 等三个轴到位（只依赖电机；此时底盘正开去 23 号格）*/
            Servo2_SetAngle(CAROUSEL_ANG_PARK);              /* 转盘转到收工位角 */
            if ((Grab6020_Arrived_Big() && M27_Arrived()) ||
                ((HAL_GetTick() - s_gb_ts) >= 3000)) {       
                Grab6020_Stop();
                grab_sub = 2;
            }
            break;
        case 2:   /* 4-2 收工位已就位 */
            if (!grab_round2) break;                     /* 底盘还没让开第二轮：等着 */
            grab_round2 = 0;                             /* 消费掉：下一个圆环轮次还要再用一次 */
            grab_sub    = 0;
            s_sent      = 0;
            s_scan5_i   = 0;
            Grab_now    = 5;
            break;
        }
        break;
    /* 找目标8号环 */
    case 5:   
        switch (grab_sub) {
        case 0:   /* 5-0 摆扫描姿态：6020→90°、M2→5cm、UD→0 */
            if (!s_sent) {
                MoveCm(90.0f, 0.0f, K6_RETRACT_M2_CM, 900, 200, 0.5f, 900, 100);
                s_gb_ts = HAL_GetTick();
                s_sent = 1;
                break;
            }
            if ((!(Grab6020_Arrived_Big() && M27_Arrived())) &&((HAL_GetTick() - s_gb_ts) < 3000)) break;
            s_sent    = 0;
            s_scan5_i = 0;
            grab_sub  = 1;                 /* 到位了：先看有没有 8 号环 */
            break;
        case 1:   /* 5-1 到位后：**看见 8 → 直接对准**；没看见 → M2 在 3↔8cm 来回巡逻 */
            if (cam_has_ring && cam_ring_no == 8) {
                r5_align_begin();          /* 清状态（含 s_r5_t）*/
                s_sent   = 0;
                grab_sub = 2;
                break;
            }

            if (!s_sent) {
                MoveCm(Grab6020_GetTarget(), 0.0f, s_scan5_cm[s_scan5_i], 900, 100, 0.5f, 900, 100);
                s_gb_ts = HAL_GetTick();
                s_sent = 1;
            } else if (M27_Arrived() ||((HAL_GetTick() - s_gb_ts) >= 1400)) {
                s_scan5_i = (s_scan5_i + 1) & 1;
                s_sent    = 0;
            }
            break;
        case 2:   /* 5-2 已看到 8：M2 速度模式对准图像 */
        {
            int  new_frame;
            uint32_t fseq_save;

            if (ring_ok) {                       /* 两边都对准了 → 先记下 8 号环的位姿，再进 K6 */
                s_cal_ang[1] = Grab6020_GetAngle();   /* 8 号环：对准那一刻的 6020 角度 */
                s_cal_m5[1]  = cm_2;         /* 8 号环：对准那一刻的 M2 长度 */
                ring_ok    = 0;
                gb_m2_stop();
                s_sent     = 0;
                grab_sub   = 0;
                /* case 5 → 6（标定放料）；case 11 → 12（第二轮第二次放置）*/
                Grab_now   = 6;
                break;
            }
            /* 先算新帧、再喂狗：被静默门挡住时要把序号还回去（同 case2）*/
            fseq_save = s_r5_fseq;
            new_frame = See_IsNewRingFrame(&s_r5_fseq);
            if (new_frame) s_r5_t = HAL_GetTick();

            /* 环不在视 / 不是 8 号：按丢失处理 */
            if (!(new_frame && cam_has_ring && cam_ring_no == 8)) {
                uint32_t age = HAL_GetTick() - s_r5_t;

                if (age <= RING_M2_LOST_HOLD_MS) {          /* 短暂丢帧：保持上次速度（仍过限位包线）*/
                    m2_mode = 1;
                    M2Speed(gb_limit_envelope(s_r5_v, cm_2), 100);
                    break;
                }
                gb_m2_stop();
                s_r5_stuck = 0;
                if (age > CH_LOST_RETURN_MS) {              /* 长时间没环：回子步 1 重新扫 */
                    s_sent    = 0;
                    s_scan5_i = 0;
                    grab_sub  = 1;
                }
                break;
            }

            /* 45ms 静默门：同一帧不重复算，被挡住就把序号还回去 */
            if ((int32_t)(HAL_GetTick() - s_r5_mute) < 0) {
                s_r5_fseq = fseq_save;
                break;
            }

            {   /* ===== 一个有效圆环帧的完整处理 ===== */
                uint32_t now    = HAL_GetTick();
                uint32_t dt_ms  = now - s_r5_tick;
                const float k_px = cam_px_to_cm_y;
                int    cy       = cam_ring_cy - (int)RING_CY_TGT_PX_8;   /* 以 RING_CY_TGT_PX_8 为零点 */
                int    acy      = (cy < 0) ? -cy : cy;
                float  cm       = cm_2;
                float  v_arm    = 0.0f;
                float  e_cm, tol_cm, v, vlim;
                int hold0   = 0;

                /* 不可信帧丢弃（同 case2）：帧间隔很短、定位却跳很大 ⇒ 突发帧/串色， 整帧不采信；但计时照走，否则 dt 越积越大。 */
                {
                    float d_raw = (float)cy - s_r5_prev;

                    if (!s_lock_first && dt_ms < GB_JUMP_DT_MS && fabsf(d_raw) > GB_JUMP_PX) {
                        s_r5_tick = now;
                        break;
                    }
                }

                /* 臂实测速度：靠 M2 长度差分 */
                if (dt_ms > 8 && dt_ms < 250) v_arm = (cm - s_r5_cm) * 1000.0f / (float)dt_ms;
                s_r5_cm   = cm;
                s_r5_tick = now;
                s_r5_prev = (float)cy;
                s_lock_first = 0;

                e_cm   = GB_RING_M2_SIGN * (float)cy * k_px;   /* 正 = 需伸出（同 case2 约定）*/
                tol_cm = (float)RING_K6_TOL_PX * k_px;

                /* ② 位置项（静止目标 ⇒ 无速度前馈；正号：cy>0 要伸出）*/
                v = s_r5_sy_sgn * RING_M2_KV * e_cm;      /* 方向由 s_r5_sy_sgn 定 */

                /* ③ 刹车包线：这一拍能停住的速度上限 */
                vlim = RING_M2_ENV_K * gb_stop_speed(fabsf(e_cm));
                if (v >  vlim) v =  vlim;
                if (v < -vlim) v = -vlim;

                /* ④ 限位包线：防撞软限位（方向相关）*/
                v = gb_limit_envelope(v, cm);

                /* ⑤ 实测超速刹车：臂实际速度已经超过"剩余距离能刹住"的值 ⇒ 立刻归零 */
                if (fabsf(v_arm) > gb_stop_speed(fabsf(e_cm))) { v = 0.0f; hold0 = 1; }

                /* ⑥ 死区 */
                if (fabsf(e_cm) < tol_cm) { v = 0.0f; hold0 = 1; }

                /* ⑦ 总饱和 + 斜率限幅（减速不限，加速每拍最多 lim）*/
                if (v >  RING_M2_VMAX) v =  RING_M2_VMAX;
                if (v < -RING_M2_VMAX) v = -RING_M2_VMAX;
                {
                    float lim = 0.15f * (float)dt_ms;

                    if (lim < 3.0f)  lim = 3.0f;
                    if (lim > 10.0f) lim = 10.0f;
                    if (v > s_r5_v + lim) v = s_r5_v + lim;
                    if (v < s_r5_v - lim) v = s_r5_v - lim;
                }
                if (hold0) v = 0.0f;                 /* ⑤/⑥ 给的 0 是硬 0，斜率不许拉回 */
                if (v > -0.3f && v < 0.3f) v = 0.0f;

                s_r5_v = v;
                m2_mode = 1;                         /* M2 速度模式 */
                M2Speed(s_r5_v, 100);
                s_r5_mute = HAL_GetTick() + RING_M2_MUTE_MS;

                /* 顶死检测：M2 已经顶在软限位上、误差还很大 ⇒ 物理够不着，回子步 1 重来 */
                if ((cm >= FB_EXT_MAX_CM - 1.0f || cm <= FB_MIN_CM + 1.0f) &&
                    acy > (int)(2.0f / k_px)) {
                    if (s_r5_stuck == 0) {
                        s_r5_stuck = now;
                    } else if ((now - s_r5_stuck) >= RING_M2_STUCK_MS) {
                        s_r5_stuck = 0;
                        gb_m2_stop();
                        s_sent     = 0;
                        s_scan5_i = 0;
                        grab_sub   = 1;
                    }
                } else {
                    s_r5_stuck = 0;
                }
            }
            break;
        }
        }
        break;

    case 6:   /* 7/9 标定（TaskFlow case 6，两轮共用）：按这一轮的 M2₈ 现算 90°∓θ 和斜边，PD 对准并记录 */
        m2_mode = 0;                 /* M2 位置模式 */

        /* case6_key == 0：跳过 7/9 的 PD 对准，只把理论值写进 s_cal_*，直接去放料。
         *   用的就是 C 行里的那两列「算」：7 号 = 90+θ、9 号 = 90−θ、M2 三处都是"斜边−偏置"。
         *   ⚠ 8 号（s_cal_ang[1]/s_cal_m5[1]）不动，它由 case 5/11 对准时实测得到。*/
        if (g_case6_key == 0) {
            float m2_8 = s_cal_m5[1];
            float adj, th, m2t;

            if (m2_8 < 2.0f) m2_8 = 10.0f;           /* 反馈没上来：兜底，同下面 */
            adj = K6_RING_ARM_OFFSET_CM + m2_8;
            th  = atan2f(K6_RING_OPP_CM, adj) * (180.0f / 3.14159265f);
            m2t = sqrtf((K6_RING_OPP_CM * K6_RING_OPP_CM) + (adj * adj)) - K6_RING_ARM_OFFSET_CM;

            grab_k6_th = th;                            /* 让 C 行与 case 12 的公式保持一致 */
            grab_k6_m2 = m2t;

            s_cal_ang[2] = 90.0f - th + K6_D9_ANG_DEG;  s_cal_m5[2] = Grab_K6M2Of(9);   /* 9 号（理论值+偏置）*/
            s_cal_ang[0] = 90.0f + th + K6_D7_ANG_DEG;  s_cal_m5[0] = Grab_K6M2Of(7);   /* 7 号（同上）*/

            s_place_i = 0;
            grab_sub  = 0;
            s_sent    = 0;
            Grab_now  = 7;                           /* 直接进放料 */
            break;
        }

        switch (grab_sub) {

        case 0:   /* 6-0 用 8 号环记下的 M2 现算两个标定姿态，先摆 9 号（90°−θ），做 PD */
            if (!s_sent) {
                /* 8 号环的 6020/M2 已在 case5 对准完成那一刻记好（s_cal_ang[1] / s_cal_m5[1]）*/
                float m2_8 = s_cal_m5[1];

                /* 直角三角形：θ = atan(15/(K6_RING_ARM_OFFSET_CM+M2₈))，斜边 √(15²+(…+M2₈)²)，M2 目标 = 斜边−偏置。 M2₈≈0（反馈没上来）会算成 90° ⇒ 6020 甩一大截，这里按 10cm 兜底。 */
                if (m2_8 < 2.0f) m2_8 = 10.0f;
                {
                    float adj = K6_RING_ARM_OFFSET_CM + m2_8;      /* 真正的邻边 */

                    grab_k6_th = atan2f(K6_RING_OPP_CM, adj) * (180.0f / 3.14159265f);
                    grab_k6_m2 = sqrtf((K6_RING_OPP_CM * K6_RING_OPP_CM) + (adj * adj))
                              - K6_RING_ARM_OFFSET_CM;             /* 斜边减回偏置 */
                }

                MoveCm(90.0f - grab_k6_th + K6_D9_ANG_DEG, 0.0f, Grab_K6M2Of(9), 300, 100, cm_7, 900, 100);
                s_gb6_tick = HAL_GetTick();
                s_sent = 1;
                break;
            }
            if (!Grab6020_Arrived() || !M27_Arrived()) {
                if ((HAL_GetTick() - s_gb6_tick) < 1500) break;   /* 1.5s 没到位也先做 PD */
            }
            s_sent      = 0;
            s_gb6_fseq = cam_ring_seq;
            s_gb6_tol  = 0;
            s_gb6_pd_t  = 0;         /* 等该姿态第一帧再建 D 基准 */
            s_gb6_dead_t = HAL_GetTick();
            grab_sub   = 1;
            break;

        case 1:   /* 6-1 9 号环位置 PD，连续 K6_ALIGN_FRAMES(7) 帧在 RING_K6_TOL_PX(8) 内记录 */
            if (!gb_k6_pd_align(9, &s_cal_ang[2], &s_cal_m5[2])) {
                /* 6s 兜底：9 号环一直没进画面 / 一直不收敛 ⇒ 用**当前姿态**记录并继续。 原来这里没有任何超时 ⇒ 整台车陪着手臂永久等下去（底盘在 case 6/7 一起卡）。 */
                if ((HAL_GetTick() - s_gb6_dead_t) < 6000) break;
                s_cal_ang[2] = Grab6020_GetAngle();
                s_cal_m5[2]  = cm_2;
                s_gb6_tol    = 0;
            }
            grab_sub = 2;
            s_sent   = 0;
            break;

        case 2:   /* 6-2 摆 7 号姿态（90°+θ，M2 用同一条斜边）*/
            if (!s_sent) {
                MoveCm(90.0f + grab_k6_th + K6_D7_ANG_DEG, 0.0f, Grab_K6M2Of(7), 300, 100, cm_7, 900, 100);
                s_gb6_tick = HAL_GetTick();
                s_sent = 1;
                break;
            }
            if (!Grab6020_Arrived() || !M27_Arrived()) {
                if ((HAL_GetTick() - s_gb6_tick) < 1500) break;   /* 1.5s 没到位也先做 PD */
            }
            s_sent     = 0;
            s_gb6_fseq = cam_ring_seq;
            s_gb6_tol  = 0;
            s_gb6_pd_t  = 0;         /* 等该姿态第一帧再建 D 基准 */
            s_gb6_dead_t = HAL_GetTick();
            grab_sub   = 3;
            break;

        case 3:   /* 6-3 7 号环位置 PD，连续 K6_ALIGN_FRAMES(7) 帧在 RING_K6_TOL_PX(8) 内记录 */
            if (!gb_k6_pd_align(7, &s_cal_ang[0], &s_cal_m5[0])) {
                if ((HAL_GetTick() - s_gb6_dead_t) < 6000) break;   /* 6s 兜底，同 6-1 */
                s_cal_ang[0] = Grab6020_GetAngle();
                s_cal_m5[0]  = cm_2;
                s_gb6_tol    = 0;
            }
            s_place_i   = 0;
            grab_sub    = 0;
            s_sent      = 0;
            Grab_now  = 7;
            break;
        }
        break;

    case 7:   /* 三次放料：取车上物料 → 放到二维码指定环位的标定位置 */
        m2_mode = 0;                 /* M2 位置模式 */
        switch (grab_sub) {
        case 0:   /* 7-0 舵机转到当前物料格 + 6020 放料角 + M2 5cm + M7 3.9cm */
            if (!s_sent) {
                int ring = (uint8_t)qr_c1[s_place_i];
                int slot = (ring >= 7 && ring <= 9) ? (uint8_t)(ring - 7) : 1;  /* 7→0,8→1,9→2；异常值按 8 号兜底 */
                Servo2_SetAngle(s_carousel_ang[s_place_i]);
                Gripper_Open();
                MoveCm(M6020_PLACE_DEG, 0.0f, 4.9f, 900, 250, 4.0f, 900, 250);
                s_place_ang  = s_cal_ang[slot];  /* 本轮目标环的标定位 */
                s_place_m5   = s_cal_m5[slot];
                s_gb_ts = HAL_GetTick();
                s_sent = 1;
                break;
            }
            if ((!Grab6020_Arrived_Big() || !M27_Arrived()) &&
                ((HAL_GetTick() - s_gb_ts) < 2800)) break;
            s_sent   = 0;
            grab_sub = 1;
            break;

        case 1:   /* 7-1 合爪，等 500ms（非阻塞）*/
            if (!s_sent) {
                Gripper_Close();
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if ((HAL_GetTick() - s_gb_ts) < 450) break;
            s_sent   = 0;
            grab_sub = 2;
            break;

        case 2:   /* 7-2 M7 升到 2cm（原来这里没有任何兜底：UD 反馈异常会让整个放料永久卡住）*/
            if (!s_sent) {
                MoveCm(Grab6020_GetTarget(), 0.0f, 4.9, 900, 100, 1.5f, 330, 0);
                s_gb_ts = HAL_GetTick();
                s_sent = 1;
                break;
            }
            if ((!M27_UdArrived()) && ((HAL_GetTick() - s_gb_ts) < 2000)) break;
            s_sent   = 0;
            grab_sub = 3;
            break;

        case 3:   /* 7-3 先 6020 转到标定位，再动 M2/M7（同时动容易缠线）：到位判据用**严格**的 Grab6020_Arrived()（0.5 度）*/
            /* ① 只转 6020：M2/M7 保持当前位置不动 */
            if (s_sent == 0) {
                MoveCm(s_place_ang, 0.0f, cm_2, 1000, 220, cm_7, 300, 200);
                Grab6020_SetRateMax(180.0f);
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if (s_sent == 1) {
                if ((!Grab6020_Arrived()) && ((HAL_GetTick() - s_gb_ts) < 600)) break;
                s_sent = 2;                        /* 6020 到位：接下来动 M2/M7 */
                /* 6020 到位了才让转盘避让（二维码第 4/5 位写 3 ⇒ +6 变 9，且只对第 1/2 块）*/
                if ((s_place_i < 2) && (qr_c1[s_place_i] == 9)) {
                    Servo2_SetAngle(CAROUSEL_ANG_AVOID);
                }
            }

            /* ② 动 M2 到标定长度、M7 降到 13.1cm（6020 目标不变，不会再转）*/
            if (s_sent == 2) {
                MoveCm(s_place_ang, 0.0f, s_place_m5, 1000, 220, 13.1f, 300, 200);
                Grab6020_SetRateMax(180.0f);      /* 标定位慢转 */
                s_gb_ts = HAL_GetTick();
                s_sent = 3;
                break;
            }
            if ((!Grab6020_Arrived() || !M27_Arrived()) &&
                ((HAL_GetTick() - s_gb_ts) < 3000)) break;
            s_sent   = 0;
            grab_sub = 4;
            break;

        case 4:   /* 7-4 开爪，等 300ms（非阻塞）*/
            if (!s_sent) {
                Gripper_Open();
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if ((HAL_GetTick() - s_gb_ts) < 400) break;
            s_sent = 0;
            s_place_i++;
            s_gb_ts  = HAL_GetTick();
            grab_sub = 5;              /* 第 3 块也先做 7-5，再进回收 */
            break;

        case 5:   /* 7-5 下一块之前先把 M7 抬离深位（4cm），避免机械限位卡死 */
            /* 只有"下一步回 7-0 放下一块"时才需要抬（7-0 自己的目标就是 M7=4cm）。 如果三块料已放完、下一步是 **7-6 回收**（M7 马上就要降到 14cm）， 这一次抬升纯属浪费 400ms ⇒
             * 直接跳进 7-6，不执行本步（用户 2026-10-31 要求）。 */
            if ((s_place_i >= 3) && (s_no_pick == 0)) {
                s_place_i = 0;
                s_pick_i  = 0;
                s_sent    = 0;
                grab_sub  = 6;          /* 第一轮：直接进回收第一块 */
                break;
            }

            if (!s_sent) {
                MoveCm(Grab6020_GetTarget(), 0.0f, cm_2, 900, 100, 2.0f, 900, 250);
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if ((HAL_GetTick() - s_gb_ts) < 800) break;
            s_sent = 0;
            if (s_place_i >= 3) {       /* 三块都放完了（只剩第二轮这条路）*/
                s_place_i = 0;
                s_pick_i  = 0;
                grab_sub = 0;
                Grab6020_SetRateMax(0.0f);   /* 恢复默认限速 */
                /* 第二次 case7 不再进 case 8： 第 1 遍 -> GB case 9（收工位等 grab_round3 = 第二遍开始） 第 2 遍 -> GB case 10（停住，整场结束） */
                Grab_now = big_round ? 10 : 9;
            } else {
                grab_sub = 0;            /* 回 7-0 放下一块 */
            }
            break;

        case 6:   /* 7-6 回收第 s_pick_i 块（先抬到 10cm 仅第一块）→ 再把 6020 转到标定角 → 最后动 M2/M7
                   *   ⚠ 转 6020 时 M2 保持不动、M7 只用原来的抬升目标，避免"边横扫边伸"缠线/打掉物料。
                   *   300ms 是**固定延时**，不判 UD 是否到位（用户明确要求不等待）。*/
            if (s_sent == 0) {                     /* ① 第一块先抬到 10cm；第 2/3 块直接转 6020 */
                int ring;
                int slot;

                if (s_pick_i != 0) {
                    ring = (uint8_t)qr_c1[s_pick_i];
                    slot = (ring >= 7 && ring <= 9) ? (uint8_t)(ring - 7) : 1;
                    Servo2_SetAngle(s_carousel_ang[s_pick_i]);
                    MoveCm(s_cal_ang[slot], 0.0f, cm_2, 900, 200, cm_7, 900, 250);   /* 只转 6020 */
                    s_gb_ts = HAL_GetTick();
                    s_sent  = 2;
                    break;
                }

                MoveCm(Grab6020_GetTarget(), 0.0f, cm_2, 900, 100, 10.0f, 900, 250);
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if (s_sent == 1) {                     /* ② 固定等 300ms，然后转 6020（M7 继续抬到 10cm）*/
                int ring;
                int slot;

                if ((HAL_GetTick() - s_gb_ts) < 300) break;

                ring = (uint8_t)qr_c1[s_pick_i];
                slot = (ring >= 7 && ring <= 9) ? (uint8_t)(ring - 7) : 1;

                Servo2_SetAngle(s_carousel_ang[s_pick_i]);
                MoveCm(s_cal_ang[slot], 0.0f, cm_2, 900, 200, 10.0f, 900, 250);   /* 只转 6020 */
                s_gb_ts = HAL_GetTick();
                s_sent  = 2;
                break;
            }
            if (s_sent == 2) {                     /* ③ 6020 到位后，才动 M2/M7 */
                int ring;
                int slot;

                if ((!Grab6020_Arrived_Big()) && ((HAL_GetTick() - s_gb_ts) < 2000)) break;

                ring = (uint8_t)qr_c1[s_pick_i];
                slot = (ring >= 7 && ring <= 9) ? (uint8_t)(ring - 7) : 1;

                MoveCm(s_cal_ang[slot], 0.0f, s_cal_m5[slot], 900, 200, 14.1f, 900, 250);
                s_gb_ts = HAL_GetTick();
                s_sent  = 3;
                break;
            }
            /* ④ 等 M2/M7 到标定位（带超时）*/
            if ((!Grab6020_Arrived_Big() || !M27_Arrived()) &&
                ((HAL_GetTick() - s_gb_ts) < 2400)) break;
            s_sent   = 0;
            grab_sub = 7;
            break;

        case 7:   /* 7-7 合爪，等 500ms（非阻塞）*/
            if (!s_sent) {
                Gripper_Close();
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if ((HAL_GetTick() - s_gb_ts) < 500) break;
            s_sent   = 0;
            grab_sub = 8;
            break;

        case 8:   /* 7-8 M7 先升到 2.5cm 动 500ms，再回车上放料位 */
            if (s_sent == 0) {
                MoveCm(Grab6020_GetTarget(), 0.0f, cm_2, 900, 100, 2.7f, 400, 0);
                Grab6020_SetRateMax(180.0f);      /* 标定位慢转 */
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if (s_sent == 1) {
                if ((HAL_GetTick() - s_gb_ts) < 1000) break;
                MoveCm(M6020_PLACE_DEG, 0.0f, 4.9f, 900, 200, 2.7f, 900, 250);
                Grab6020_SetRateMax(180.0f);      /* 标定位慢转 */
                s_gb_ts = HAL_GetTick();
                s_sent = 2;
                break;
            }
            if ((!Grab6020_Arrived_Big() || !M27_Arrived()) &&
                ((HAL_GetTick() - s_gb_ts) < 3000)) break;
            s_sent   = 0;
            grab_sub = 9;
            break;

        case 9:   /* 7-9 开爪，等 400ms（非阻塞）；三块回收完停 */
            if (!s_sent) {
                Gripper_Open();
                s_gb_ts = HAL_GetTick();
                s_sent  = 1;
                break;
            }
            if ((HAL_GetTick() - s_gb_ts) < 400) break;
            s_sent = 0;
            s_pick_i++;
            grab_sub = 6;
            if (s_pick_i >= 3) {
                s_pick_i   = 0;
                grab_sub   = 0;
                Grab6020_SetRateMax(0.0f);   /* 恢复默认限速：不然 case8 收工位还按 140°/s 慢转 */
                Grab_now = 8;          /* 三块放完 → 进 case 8 收工位（底盘同时进 case 8） */
            }
            break;
        }
        break;

    case 8:   /* 收工位（TaskFlow case 8 用）：6020→180°、FB→3cm、UD→1cm；就位后等 grab_round2 回 case 5 */
        m2_mode = 0;                 /* M2 位置模式 */

        switch (grab_sub) {
        case 0:   /* 8-0 下发收工位 */
            MoveCm(180.0f, 0.0f, 3.0f, 900, 100, 1.0f, 900, 100);
            s_gb_ts  = HAL_GetTick();
            grab_sub = 1;
            break;

        case 1:   /* 8-1 等三个轴到位（只依赖电机）*/
            Servo2_SetAngle(CAROUSEL_ANG_PARK);              /* 转盘转到收工位角 */
            if ((Grab6020_Arrived_Big() && M27_Arrived()) ||
                ((HAL_GetTick() - s_gb_ts) >= 1800)) {
                Grab6020_Stop();     /* 目标 = 当前角：保持力矩，不是断电松劲 */
                grab_sub = 2;
            }
            break;

        case 2:   /* 8-2 收工位就位：等底盘放行再一轮找环（TaskFlow case 8 → 回 case 5）*/
            if (big_round != 0) {                        /* 第 2 遍：收工位摆好就收工停住（底盘不再放行）*/
                Grab_now = 10;
                break;
            }
            if (!grab_round2) break;
            grab_round2 = 0;                             /* 消费掉 */
            grab_sub    = 0;
            s_sent      = 0;
            s_scan5_i   = 0;
            /* ⚠ 判据顺序很重要：**先看是不是第 2 遍**，再决定 11/12。
             *   写反过一次：case11_key=0 时第一遍的 case 8 也会跳到 12，
             *   而底盘照样回 case 5 找环 ⇒ 手臂卡在 12 等 g_c12_ready、6020 一动不动。*/
            if (big_round == 0) {
                s_no_pick = 1;                           /* 第 1 遍第二轮：放完不回收 */
                Grab_now  = 5;                           /* 回"摆姿态 → 找 8 号环 → M2 对准" */
            } 
			//唐人操作
			else if (g_case11_key != 0) {
                Grab_now = 13;                           /* 第 2 遍：对准 8 号环里那块料 */
            } else {
                Grab_now = 13;                           /* 第 2 遍 + 跳过 11：直接放料 */
            }
            break;

        default:
            break;
        }
        break;

    case 9:   /* 第 1 遍结束（TF case 9：11 号格 + 任务5）：摆收工位待命，等 grab_round3 */
        m2_mode = 0;                 /* M2 位置模式 */

        switch (grab_sub) {
        case 0:   /* 9-0 收工位：6020 180°、FB 1cm、UD 2cm */
            MoveCm(180.0f, 0.0f, 1.0f, 900, 100, 2.0f, 900, 100);
            s_gb_ts  = HAL_GetTick();
            grab_sub = 1;
            break;

        case 1:   /* 9-1 **只等 6020 到位**（1.5s 兜底），FB/UD 不等 ——
                   *   原来等的是 6020到位 且 M27_Arrived()：
                   *     FB/UD 只要有一个不干脆（走位没到 / 反馈抖动 / UD 堵转），
                   *     手臂就卡在这个子步里，6020 一直停在 180 度，
                   *     而底盘早就跑完任务 5 在巡逻了 ⇒ 现场就是"巡逻时 6020 还是 180"。
                   *   紧随其后的 case 0 会把三轴重新命令一遍（90/FB 1cm/UD 0），
                   *   所以这里的 FB/UD 等待纯属多余。*/
            if (Grab6020_Arrived_Big() ||
                ((HAL_GetTick() - s_gb_ts) >= 1600)) {
                Grab6020_Stop();     /* 目标=当前角：保持力矩 */
                grab_sub = 2;
            }
            break;

        case 2:   /* 9-2 就位待命：等底盘置 grab_round3（= 第二遍开始）*/
            if (!grab_round3) break;
            grab_round3 = 0;         /* 消费掉 */
            grab_sub    = 0;
            s_sent      = 0;
            s_scan5_i   = 0;
            s_no_pick   = 0;         /* 第二遍第一次 case7 要"放+回收" */
            Grab_now    = 0;         /* 回 case 0 摆抓取姿态 → 之后 case 1 扫描 */
            break;

        default:
            break;
        }
        break;

    case 10:  /* 第 2 遍也放完了：停住（保持力矩，不再发任何动作）*/
        Grab6020_Stop();
        M2Speed(0.0f, 0);
        break;



    /* ⚠ case 11 是 case 5 的完整副本（只差对准源与出口）——改 case 5 时必须同步改这里。*/
    case 11:  /* 第二轮第二次放置前的对准：认"上一次放进 8 号环那块料的颜色"（物料帧），
               *   对准后记下 8 号位的 6020/M2（s_cal_ang[1]/s_cal_m5[1]），去 case 12。
               *   颜色查不到（二维码没配好）时退回 8 号圆环帧，避免卡在巡逻。*/
        See_AlignSetSel((uint8_t)((Grab_Ring8Color() != 0) ? 1 : 0), Grab_Ring8Color());
        /* 这里原来每拍写 m2_mode = 0：5-2 的 M2Speed 已把它置 1，本拍又改回 0， 于是 MotorC 有可能用**位置模式**重发 5-1 的陈旧扫描目标，和 M2 速度对准抢控制。
         * MoveCm/M2Speed 内部各自会设对模式（M27.c），这里不用写。 */
        /* ring_ok **只由 5-2（已对准）消费并记录** s_cal_ang[1]/s_cal_m5[1]。 原来这里有个"看到 ring_ok 就直接进 case 6"的旁路，它**不记录** 8 号环位姿，
         * 于是 case6 拿到 M2₈=0 → θ 被算成 90°（再被兜底成 10cm）。
         * 那条旁路已删除：ring_ok 挂着就先走完 5-0/5-1/5-2，由 5-2 统一记录。 */
        switch (grab_sub) {

        case 0:   /* 5-0 摆扫描姿态：6020→90°、M2→5cm、UD→0 */
            if (!s_sent) {
                MoveCm(90.0f, 0.0f, K6_RETRACT_M2_CM, 900, 200, 0.5f, 900, 100);
                s_gb_ts = HAL_GetTick();
                s_sent = 1;
                break;
            }
            if ((!(Grab6020_Arrived_Big() && M27_Arrived())) &&
                ((HAL_GetTick() - s_gb_ts) < 2600)) break;
            s_sent    = 0;
            s_scan5_i = 0;
            grab_sub  = 1;                 /* 到位了：先看有没有 8 号环 */
            break;

        case 1:   /* 5-1 到位后：**看见 8 → 直接对准**；没看见 → M2 在 3↔8cm 来回巡逻 */
            if (See_AlignSeen()) {
                r5_align_begin();          /* 清状态（含 s_r5_t）*/
                s_sent   = 0;
                grab_sub = 2;
                break;
            }

            if (!s_sent) {
                MoveCm(Grab6020_GetTarget(), 0.0f, s_scan5_cm[s_scan5_i], 900, 100, 0.5f, 900, 100);
                s_gb_ts = HAL_GetTick();
                s_sent = 1;
            } else if (M27_Arrived() ||
                       ((HAL_GetTick() - s_gb_ts) >= 1400)) {
                s_scan5_i = (s_scan5_i + 1) & 1;
                s_sent    = 0;
            }
            break;

        case 2:   /* 5-2 已看到 8：**M2 速度模式对准图像 y**（照 case2 骨架，目标静止）
                   *   底盘 X 由 TaskFlow 前后对图像 x；两轴都到位才 ring_ok → case 6。
                   *   保护一个不少：看门狗三档 / 45ms 静默门 / 刹车包线 / 限位包线 /
                   *                实测超速刹车 / 死区 / 总饱和+斜率 / 顶死检测。*/
        {
            int  new_frame;
            uint32_t fseq_save;

            if (ring_ok) {                       /* 两边都对准了 → 先记下 8 号环的位姿，再进 K6 */
                s_cal_ang[1] = Grab6020_GetAngle();   /* 8 号环：对准那一刻的 6020 角度 */
                s_cal_m5[1]  = cm_2;         /* 8 号环：对准那一刻的 M2 长度 */
                ring_ok    = 0;
                gb_m2_stop();
                s_sent     = 0;
                grab_sub   = 0;
                /* case 5 → 6（标定放料）；case 11 → 12（第二轮第二次放置）*/
                Grab_now   = 12;
                break;
            }

            /* 先算新帧、再喂狗：被静默门挡住时要把序号还回去（同 case2）*/
            fseq_save = s_r5_fseq;
            new_frame = See_AlignNew(&s_r5_fseq);
            if (new_frame) s_r5_t = HAL_GetTick();

            /* 环不在视 / 不是 8 号：按丢失处理 */
            if (!(new_frame && See_AlignSeen())) {
                uint32_t age = HAL_GetTick() - s_r5_t;

                if (age <= RING_M2_LOST_HOLD_MS) {          /* 短暂丢帧：保持上次速度（仍过限位包线）*/
                    m2_mode = 1;
                    M2Speed(gb_limit_envelope(s_r5_v, cm_2), 100);
                    break;
                }
                gb_m2_stop();
                s_r5_stuck = 0;
                if (age > CH_LOST_RETURN_MS) {              /* 长时间没环：回子步 1 重新扫 */
                    s_sent    = 0;
                    s_scan5_i = 0;
                    grab_sub  = 1;
                }
                break;
            }

            /* 45ms 静默门：同一帧不重复算，被挡住就把序号还回去 */
            if ((int32_t)(HAL_GetTick() - s_r5_mute) < 0) {
                s_r5_fseq = fseq_save;
                break;
            }

            {   /* ===== 一个有效圆环帧的完整处理 ===== */
                uint32_t now    = HAL_GetTick();
                uint32_t dt_ms  = now - s_r5_tick;
                const float k_px = cam_px_to_cm_y;
                int    cy       = See_AlignCy() - (int)RING_CY_TGT_PX_8;   /* 以 RING_CY_TGT_PX_8 为零点 */
                int    acy      = (cy < 0) ? -cy : cy;
                float  cm       = cm_2;
                float  v_arm    = 0.0f;
                float  e_cm, tol_cm, v, vlim;
                int hold0   = 0;

                /* 不可信帧丢弃（同 case2）：帧间隔很短、定位却跳很大 ⇒ 突发帧/串色， 整帧不采信；但计时照走，否则 dt 越积越大。 */
                {
                    float d_raw = (float)cy - s_r5_prev;

                    if (!s_lock_first && dt_ms < GB_JUMP_DT_MS && fabsf(d_raw) > GB_JUMP_PX) {
                        s_r5_tick = now;
                        break;
                    }
                }

                /* 臂实测速度：靠 M2 长度差分 */
                if (dt_ms > 8 && dt_ms < 250) v_arm = (cm - s_r5_cm) * 1000.0f / (float)dt_ms;
                s_r5_cm   = cm;
                s_r5_tick = now;
                s_r5_prev = (float)cy;
                s_lock_first = 0;

                e_cm   = GB_RING_M2_SIGN * (float)cy * k_px;   /* 正 = 需伸出（同 case2 约定）*/
                tol_cm = (float)RING_K6_TOL_PX * k_px;

                /* ② 位置项（静止目标 ⇒ 无速度前馈；正号：cy>0 要伸出）*/
                v = s_r5_sy_sgn * RING_M2_KV * e_cm;      /* 方向由 s_r5_sy_sgn 定 */

                /* ③ 刹车包线：这一拍能停住的速度上限 */
                vlim = RING_M2_ENV_K * gb_stop_speed(fabsf(e_cm));
                if (v >  vlim) v =  vlim;
                if (v < -vlim) v = -vlim;

                /* ④ 限位包线：防撞软限位（方向相关）*/
                v = gb_limit_envelope(v, cm);

                /* ⑤ 实测超速刹车：臂实际速度已经超过"剩余距离能刹住"的值 ⇒ 立刻归零 */
                if (fabsf(v_arm) > gb_stop_speed(fabsf(e_cm))) { v = 0.0f; hold0 = 1; }

                /* ⑥ 死区 */
                if (fabsf(e_cm) < tol_cm) { v = 0.0f; hold0 = 1; }

                /* ⑦ 总饱和 + 斜率限幅（减速不限，加速每拍最多 lim）*/
                if (v >  RING_M2_VMAX) v =  RING_M2_VMAX;
                if (v < -RING_M2_VMAX) v = -RING_M2_VMAX;
                {
                    float lim = 0.15f * (float)dt_ms;

                    if (lim < 3.0f)  lim = 3.0f;
                    if (lim > 10.0f) lim = 10.0f;
                    if (v > s_r5_v + lim) v = s_r5_v + lim;
                    if (v < s_r5_v - lim) v = s_r5_v - lim;
                }
                if (hold0) v = 0.0f;                 /* ⑤/⑥ 给的 0 是硬 0，斜率不许拉回 */
                if (v > -0.3f && v < 0.3f) v = 0.0f;

                s_r5_v = v;
                m2_mode = 1;                         /* M2 速度模式 */
                M2Speed(s_r5_v, 100);
                s_r5_mute = HAL_GetTick() + RING_M2_MUTE_MS;

                /* 顶死检测：M2 已经顶在软限位上、误差还很大 ⇒ 物理够不着，回子步 1 重来 */
                if ((cm >= FB_EXT_MAX_CM - 1.0f || cm <= FB_MIN_CM + 1.0f) &&
                    acy > (int)(2.0f / k_px)) {
                    if (s_r5_stuck == 0) {
                        s_r5_stuck = now;
                    } else if ((now - s_r5_stuck) >= RING_M2_STUCK_MS) {
                        s_r5_stuck = 0;
                        gb_m2_stop();
                        s_sent     = 0;
                        s_scan5_i = 0;
                        grab_sub   = 1;
                    }
                } else {
                    s_r5_stuck = 0;
                }
            }
            break;
        }
        }
        break;

    case 12:  /* 第二轮第二次放置 */
        m2_mode = 0;                 /* M2 位置模式 */

        if ((g_case11_key == 0) && (!g_c12_ready)) {
            /* 等底盘开到 8 号环位置；最多等 20s（底盘那侧 15s 就会兜底放行），防止两边对不上时死锁 */
            if (s_c12_wt == 0) s_c12_wt = HAL_GetTick();
            if ((HAL_GetTick() - s_c12_wt) < 20000) break;
        }
        s_c12_wt = 0;

        if (!s_lock_calc) {                             /* 进来第一拍：初始化 */
            if (g_case11_key != 0) {                /* 走 case 11：现算 7/9 的理论值 */
                float m2_8 = s_cal_m5[1];            /* case 11 对准 8 号环时的 M2 实测 */
                float adj;

                if (m2_8 < 2.0f) m2_8 = 10.0f;       /* 反馈没上来：兜底，同 case 6 */
                adj = K6_RING_ARM_OFFSET_CM + m2_8;

                s_c12_th = atan2f(K6_RING_OPP_CM, adj) * (180.0f / 3.14159265f);
            }
            /* 跳过 case 11 时不算理论值：直接用 s_c1_ang/s_c1_m5（第一轮第二次放置的实测）*/
            s_c12_i  = 0;
            s_lock_calc = 1;
            grab_sub = 0;
            s_sent   = 0;
        }

        {
            int ring = (uint8_t)(9 - s_c12_i);      /* 9 → 8 → 7 */
            int slot = gb_ring_slot(ring);
            float   ang = 0.0f;
            float   m2t = 0.0f;

            if (g_case11_key == 0) {
                /* 跳过 case 11：用"第一轮第二次放置时"记下的那三组实测值 */
                int k = (uint8_t)(ring - 7);        /* 0=7号 1=8号 2=9号 */
                ang = s_c1_ang[k];
                m2t = s_c1_m5[k];
            } else {
                if (ring == 9)      ang = s_cal_ang[1] - s_c12_th + K6_D9_ANG_DEG;
                else if (ring == 8) ang = s_cal_ang[1];
                else                 ang = s_cal_ang[1] + s_c12_th + K6_D7_ANG_DEG;
                m2t = Grab_K6M2Of(ring);            /* 7/9 各用各的长度（带横向偏置）*/
            }

            switch (grab_sub) {

            case 0:   /* 12-0 从车上第 slot 槽取料：转盘到位 + 张爪 + 6020 放料角 / M2 5cm / M7 4cm */
                if (!s_sent) {
                    Servo2_SetAngle(s_carousel_ang[slot]);
                    Gripper_Open();
                    MoveCm(M6020_PLACE_DEG, 0.0f, 4.9f, 900, 250, 4.0f, 900, 250);
                    s_gb_ts = HAL_GetTick();
                    s_sent  = 1;
                    break;
                }
                if ((!Grab6020_Arrived_Big() || !M27_Arrived()) &&
                    ((HAL_GetTick() - s_gb_ts) < 2800)) break;
                s_sent   = 0;
                grab_sub = 1;
                break;

            case 1:   /* 12-1 合爪，等 500ms */
                if (!s_sent) {
                    Gripper_Close();
                    s_gb_ts = HAL_GetTick();
                    s_sent  = 1;
                    break;
                }
                if ((HAL_GetTick() - s_gb_ts) < 500) break;
                s_sent   = 0;
                grab_sub = 2;
                break;

            case 2:   /* 12-2 M7 抬到 2cm（防横扫打掉已放好的料）*/
                if (!s_sent) {
                    MoveCm(Grab6020_GetTarget(), 0.0f, cm_2, 900, 100, 2.0f, 300, 0);
                    s_gb_ts = HAL_GetTick();
                    s_sent  = 1;
                    break;
                }
                if ((!M27_UdArrived()) &&
                    ((HAL_GetTick() - s_gb_ts) < 1600)) break;
                s_sent   = 0;
                grab_sub = 3;
                break;

            case 3:   /* 12-3 转到目标环的理论姿态（6020 = ang，M2 = 同一斜边），M7 降到 14.1cm */
                if (!s_sent) {
                    MoveCm(ang, 0.0f, m2t, 900, 200, 6.3f, 900, 250);
                    s_gb_ts = HAL_GetTick();
                    s_sent  = 1;
                    break;
                }
                if ((!Grab6020_Arrived_Big() || !M27_Arrived()) &&
                    ((HAL_GetTick() - s_gb_ts) < 3000)) break;
                s_sent   = 0;
                grab_sub = 4;
                break;

            case 4:   /* 12-4 张爪放下，等 400ms */
                if (!s_sent) {
                    Gripper_Open();
                    s_gb_ts = HAL_GetTick();
                    s_sent  = 1;
                    break;
                }
                if ((HAL_GetTick() - s_gb_ts) < 400) break;
                s_sent   = 0;
                grab_sub = 5;
                break;

            case 5:   /* 12-5 抬起 M7 到 4cm；下一块 / 收工 */
                if (!s_sent) {
                    MoveCm(Grab6020_GetTarget(), 0.0f, cm_2, 900, 100, 4.0f, 900, 250);
                    s_gb_ts = HAL_GetTick();
                    s_sent  = 1;
                    break;
                }
                if ((!M27_UdArrived()) &&
                    ((HAL_GetTick() - s_gb_ts) < 3000)) break;
                s_sent = 0;
                s_c12_i++;
                if (s_c12_i >= 3) {
                    s_c12_i  = 0;
                    s_lock_calc = 0;
                    grab_sub = 0;
                    Grab6020_SetRateMax(0.0f);
                    Grab_now = 10;               /* 三块放完 → 收工停住（TF 同步进 case 10）*/
                } else {
                    grab_sub = 0;                /* 回 12-0 取下一块 */
                }
                break;

            default:
                grab_sub = 0;
                break;
            }
        }
        break;

    }                           /* switch (Grab_now) */
}
