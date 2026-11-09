/* 6020.c —— DJI GM6020 关节（ID=1）双环：角度环 → 角速度环 → 0x1FF 电压给定，5ms 一拍（跑在 Motor6020 任务里）。
 * 0x1FF：电压给定 ±25000；反馈 0x205：[0..1]编码器角 0~8191、[2..3]rpm、[4..5]转矩电流；[6] 不用。 ISR 只搬整数不碰浮点；臂动时底盘不转，不用 IMU。 */
#include "6020.h"
#include "grab_cfg.h"
#include "M27.h"         /* M27_GetCmFb：按手臂伸出量做刚度调度 */
#include "Follow.h"      /* Follow_RxByte：调试口(USART1)也能收路径规划帧 */
#include <math.h>

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;   /* 调参命令走 USART1 */

static volatile uint16_t s_ecd = 0;      /* 单圈机械角 0~8191 */
static volatile int16_t  s_rpm = 0;
static volatile int  s_new = 0;      /* 1 = 有新反馈待处理 */

/* 任务侧状态：只在 5ms 那一拍读写 */
static float    s_angle    = 0.0f;       /* 关节角(°) = 编码器绝对角，0° = 编码器机械零位，被控量 */
static float    s_tar      = 0.0f;       /* 目标关节角(°)，绝对：180 = 读数 180° 那个点 */

static float    s_rate     = 0.0f;       /* 关节角速度(°/s) = rpm×6 低通后 */
static float    s_rate_ref = 0.0f;       /* 角度环输出：期望角速度(°/s) */
static float    s_integ    = 0.0f;       /* 角速度环积分（°/s × 拍）*/
static float    s_sent     = 0.0f;       /* 上一拍发出的给定（斜率限制用）*/
static int  s_hold     = 1;          /* 1 = 还没收到过 MoveCm：目标跟随当前角（上电原地不动） */

/* 按 M2 伸出量在线提升刚性：臂越长，角度 P/D、速度 P 和积分输出上限越大 */
static float    s_ang_kp   = M6020_ANG_KP;
static float    s_ang_kd   = M6020_ANG_KD;
static float    s_spd_kp   = M6020_SPD_KP;
static float    s_spd_ilim = M6020_SPD_ILIM;

/* 串口在线：I 不再分 3cm/18cm，只由 |e| 线性决定：|e|=0 → 14 号，|e|>=15 号 → 0。 */
static float    s_tune_ilim       = M6020_SPD_ILIM;
static float    s_tune_ang_kp     = M6020_ANG_KP;
static float    s_tune_ang_kp_max = M6020_ANG_KP_MAX;
static float    s_tune_spd_kp     = M6020_SPD_KP;
static float    s_tune_spd_kp_max = M6020_SPD_KP_MAX;
static float    s_tune_ang_kd     = M6020_ANG_KD;
static float    s_tune_ang_kd_max = M6020_ANG_KD_MAX;
static float    s_tune_spd_kd     = M6020_SPD_KD;
static float    s_tune_spd_kd_max = M6020_SPD_KD_MAX;
static float    s_tune_spd_ki_hold = M6020_SPD_KI_HOLD;
static float    s_tune_spd_ki_hold_max = M6020_SPD_KI_HOLD_MAX;
static float    s_tune_hold_e_band = M6020_HOLD_E_BAND;
static float    s_tune_hold_e_band_max = M6020_HOLD_E_BAND_MAX;
static float    s_ki_hold          = M6020_SPD_KI_HOLD;
static float    s_e_band           = M6020_HOLD_E_BAND;
static float    s_spd_ki          = 0.0f;
static float    s_spd_kd          = M6020_SPD_KD;
static float    s_rate_step       = M6020_RATE_STEP;   /* 期望速度的加速限幅（每 5ms）*/
static float    s_rate_max        = M6020_RATE_MAX;    /* 期望速度的绝对值上限 °/s（慢转档就压它）*/
static float    s_d_out;           /* 本拍 D 项（电压原始值）*/

void Grab6020_Init(void)
{
    CAN_FilterTypeDef f;

    s_ecd = 0; s_rpm = 0;
    s_new = 0;
    s_angle = 0.0f; s_tar = 0.0f; s_hold = 1;   /* 上电不给人目标：目标跟随当前角，等 MoveCm */
    s_rate = 0.0f; s_rate_ref = 0.0f; s_integ = 0.0f; s_sent = 0.0f;

    s_tune_ilim       = M6020_SPD_ILIM;
    s_tune_ang_kp     = M6020_ANG_KP;
    s_tune_ang_kp_max = M6020_ANG_KP_MAX;
    s_tune_spd_kp     = M6020_SPD_KP;
    s_tune_spd_kp_max = M6020_SPD_KP_MAX;
    s_tune_ang_kd     = M6020_ANG_KD;
    s_tune_ang_kd_max = M6020_ANG_KD_MAX;
    s_tune_spd_kd     = M6020_SPD_KD;
    s_tune_spd_kd_max = M6020_SPD_KD_MAX;
    s_tune_spd_ki_hold = M6020_SPD_KI_HOLD;
    s_tune_spd_ki_hold_max = M6020_SPD_KI_HOLD_MAX;
    s_tune_hold_e_band = M6020_HOLD_E_BAND;
    s_tune_hold_e_band_max = M6020_HOLD_E_BAND_MAX;
    s_ki_hold          = M6020_SPD_KI_HOLD;
    s_e_band           = M6020_HOLD_E_BAND;
    s_spd_ki          = 0.0f;
    s_spd_kd          = M6020_SPD_KD;
    s_rate_step       = M6020_RATE_STEP;

    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;   /* 掩码全 0 = 全通；CAN1 只有这一台，不必挑 ID */
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &f);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* USART1 调参命令：NVIC 由 CubeMX 的 MspInit 开，这里挂 HAL 单字节接收 */
    Grab6020_CmdStart();
}

void Grab6020_CanRx(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef h;
    uint8_t d[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &h, d) != HAL_OK) return;
    if (h.StdId != 0x205) return;           /* ID=1 的反馈固定 0x205（车上扫描实测）*/

    s_ecd  = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
    s_rpm  = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    s_new  = 1;
}

static void send_voltage(float cmd)
{
    CAN_TxHeaderTypeDef th;
    uint32_t mbox;
    uint8_t  tx[8];
    int16_t  i16;

    /* 方向反了就 cmd = -cmd（只翻给定，不翻角度；翻角度会让 90° 变成 270°） 0x1FF：cmd 直接是 ±25000 电压给定；ID1 用槽 1。 */
    i16 = (int16_t)cmd;

    tx[0] = (uint8_t)((uint16_t)i16 >> 8);                /* 槽 1 = ID1 = 本车 6020 */
    tx[1] = (uint8_t)((uint16_t)i16 & 0xFF);
    tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 0; tx[6] = 0; tx[7] = 0;

    th.StdId              = 0x1FF;
    th.ExtId              = 0;
    th.IDE                = CAN_ID_STD;
    th.RTR                = CAN_RTR_DATA;
    th.DLC                = 8;
    th.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &th, tx, &mbox);
}

/* 角度差折到 ±180°：走最短路径（目标 350°、当前 10° ⇒ 差 −20°，不是 +340°） */
static float ang_err(float des, float now)
{
    float e = des - now;

    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

/* M2 伸得越长：ANG_KP 从 12 降到 8（长臂增益低更稳）；ANG_KD、SPD_KP、ILIM 仍增大。 基准值可串口在线改（1~6、8~10），调度系数固定。 */
static void update_stiffness(float fb_cm)
{
    float k = (fb_cm - 3.0f) / 15.0f;   /* 3cm 基准、18cm 满（行程现 19cm，最后 1cm 补偿略欠，影响 <7%）*/

    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    s_ang_kp   = s_tune_ang_kp     + (s_tune_ang_kp_max - s_tune_ang_kp) * k;
    s_ang_kd   = s_tune_ang_kd     + (s_tune_ang_kd_max - s_tune_ang_kd) * k;
    s_spd_kp   = s_tune_spd_kp     + (s_tune_spd_kp_max - s_tune_spd_kp) * k;
    s_spd_kd   = s_tune_spd_kd     + (s_tune_spd_kd_max - s_tune_spd_kd) * k;
    s_ki_hold  = s_tune_spd_ki_hold + (s_tune_spd_ki_hold_max - s_tune_spd_ki_hold) * k;
    s_e_band   = s_tune_hold_e_band + (s_tune_hold_e_band_max - s_tune_hold_e_band) * k;
    s_spd_ilim = s_tune_ilim;
    if (s_spd_ilim > M6020_CMD_MAX) s_spd_ilim = M6020_CMD_MAX;   /* I 项不能超过总限幅 */
}

/* hold/track 权重：只看角度误差；|e|=0 → 1（大 I），|e|>=E_BAND → 0（小 I）。 */
static float hold_weight(float eabs)
{
    float we = (s_e_band - eabs) / s_e_band;

    if (we < 0.0f) we = 0.0f;
    if (we > 1.0f) we = 1.0f;
    return we;
}

/* 5ms 一拍：外环 rate_ref = ANG_KP·(tar−ang) − ANG_KD·rate；内环 cmd = SPD_KP·err + KI·∫ − KD·d(rate)/dt。 D 对**实测 rate** 微分（不对误差微分），避免目标突变时 D 冲击；默认 0。 */
void Grab6020_Update(void)
{
    float err_ang, err_rate, p_out, out, rate_prev;

    /* 6020 的编码器是**绝对**单圈值（掉电不丢）：直接换算成关节角，0° = 编码器机械零位。 ⇒ 目标 180° 永远是"读数 180° 那个点"，与上电时停在哪无关（不做增量累加）。 */
    if (s_new) {
        s_new   = 0;
        s_angle = (float)s_ecd * (360.0f / 8192.0f);   /* 8192 格/圈 = 13 位编码器 */
        if (s_hold) s_tar = s_angle;      /* 还没收到 MoveCm：目标 = 当前角 ⇒ 原地保持，不往 0 跑 */
    }

    rate_prev = s_rate;                 /* D 用：上一拍的滤波速度 */
    s_rate = M6020_RATE_LP * ((float)s_rpm * 6.0f)  /* ×6 = rpm→°/s；整数 rpm ⇒ 量化 6°/s */
           + (1.0f - M6020_RATE_LP) * s_rate;

    update_stiffness(cm_2);

    /* ⚠ 不设死区：死区会把保持力矩一起卸掉，静摩擦就靠它顶住；到位判据在 Arrived() 里 */
    err_ang = ang_err(s_tar, s_angle);
    {
        s_spd_ki = s_ki_hold * hold_weight(fabsf(err_ang));
    }
    {
        float r = s_ang_kp * err_ang - s_ang_kd * s_rate;

#if (M6020_ACC_MAX > 0)
        /* 减速规划：朝目标给的速度 ≤ √(2·A·|误差|) = 以 A 减速刚好停住；反向（刹车）不限 */
        {
            float lim = sqrtf(2.0f * (float)M6020_ACC_MAX
                              * (err_ang < 0.0f ? -err_ang : err_ang));
            if (err_ang > 0.0f) { if (r >  lim) r =  lim; }
            else                { if (r < -lim) r = -lim; }
        }
#endif

        /* ⚠ 斜率限制只限加速侧：减速/反向不限，否则卡住刹车指令会冲过目标（实测 +15°）*/
        if (s_rate_ref > 0.0f) {
            if (r > s_rate_ref + s_rate_step) r = s_rate_ref + s_rate_step;
        } else if (s_rate_ref < 0.0f) {
            if (r < s_rate_ref - s_rate_step) r = s_rate_ref - s_rate_step;
        } else {                                   /* 静止起步：两个方向都算加速 */
            if (r >  s_rate_step) r =  s_rate_step;
            if (r < -s_rate_step) r = -s_rate_step;
        }
        if (r >  s_rate_max) r =  s_rate_max;
        if (r < -s_rate_max) r = -s_rate_max;
        s_rate_ref = r;
    }

    err_rate = s_rate_ref - s_rate;
    p_out    = s_spd_kp * err_rate;
    s_d_out    = -s_spd_kd * (s_rate - rate_prev);
    out      = p_out + s_spd_ki * s_integ + s_d_out;

    /* 抗饱和：输出打满、或被斜率限住且误差同向时，不再累加积分（否则斜率限幅期间积分照涨） 另：|e|>=E_BAND 时 s_spd_ki=0（I 根本不出力），这时也冻结积分 —— 否则在带外白绕， 回到带内 ki 一变大，就把带外绕来的量整个砸出去。 */
    if (s_spd_ki > 1e-6f
        && !((out >= M6020_CMD_MAX && err_rate > 0.0f) || (out <= -M6020_CMD_MAX && err_rate < 0.0f)
          || (out > s_sent + M6020_CMD_SLEW && err_rate > 0.0f)
          || (out < s_sent - M6020_CMD_SLEW && err_rate < 0.0f))) {
        s_integ += err_rate;
    }
    {
        /* 限幅用固定基准 s_ki_hold：瞬时 s_spd_ki 在 |e|>=E_BAND 时会掉到 0，拿它算 i_max 会把积分清成 0， 等于每摆过一次带边就丢掉一次“扛重力”的偏置。ki 只决定增益，不再决定积分状态的上限。 */
        float ki_ref = (s_ki_hold > 1e-6f) ? s_ki_hold : 1.0f;
        float i_max  = s_spd_ilim / ki_ref;   /* 积分项在输出里的上限（与瞬时 ki 无关） */
        if (s_integ >  i_max) s_integ =  i_max;
        if (s_integ < -i_max) s_integ = -i_max;
    }
    out = p_out + s_spd_ki * s_integ + s_d_out;

    if (out >  M6020_CMD_MAX) out =  M6020_CMD_MAX;
    if (out < -M6020_CMD_MAX) out = -M6020_CMD_MAX;
    if (out > s_sent + M6020_CMD_SLEW) out = s_sent + M6020_CMD_SLEW;
    if (out < s_sent - M6020_CMD_SLEW) out = s_sent - M6020_CMD_SLEW;
    s_sent = out;

    send_voltage(out);
}

void Grab6020_SetAngle(float deg, float rate_step)
{
    s_tar       = deg;
    s_rate_step = (rate_step > 0.0f) ? rate_step : M6020_RATE_STEP;   /* 0 = 默认加速度 */
    s_hold      = 0;       /* 有人给目标了：不再跟随当前角 */
}

/* 最高转速限制 °/s：dps<=0 恢复默认 M6020_RATE_MAX。 ⚠ 只改“加速度”看不出慢多少（106° 从 0.38s 只变到 0.65s），要慢必须压这个。 */
void Grab6020_SetRateMax(float dps)
{
    s_rate_max = (dps > 0.0f) ? dps : M6020_RATE_MAX;
}

float Grab6020_GetAngle(void)
{
    return s_angle;
}

float    Grab6020_GetTarget(void) { return s_tar;      }

bool Grab6020_Arrived(void)
{
    float e;

    e = ang_err(s_tar, s_angle);          /* 折 ±180° 再判，否则目标在 180° 附近时永远到不了 */
    if (e < -M6020_TOL_DEG || e > M6020_TOL_DEG) return false;
    if (s_rate_ref > M6020_TOL_DPS || s_rate_ref < -M6020_TOL_DPS) return false;
    return (s_rate > -M6020_TOL_DPS) && (s_rate < M6020_TOL_DPS);
}

/* ★ 宽松到位判据（角度容差 1 度，速度门限与严格版相同）： 给"只要摆个大概姿态"的步骤用（case0/1/3/4/5/7/8/9）。 case 6 的标定贴合仍必须用严格的 Grab6020_Arrived()（0.5 度）。 */
bool Grab6020_Arrived_Big(void)
{
    float e;

    e = ang_err(s_tar, s_angle);
    if (e < -M6020_TOL_DEG_BIG || e > M6020_TOL_DEG_BIG) return false;
    if (s_rate_ref >  M6020_TOL_DPS || s_rate_ref < -M6020_TOL_DPS) return false;
    return (s_rate > -M6020_TOL_DPS) && (s_rate < M6020_TOL_DPS);
}

void Grab6020_Stop(void)
{
    s_tar = s_angle;       /* 目标 = 当前角 → 角度环归零，角速度环刹住并保持 */
}

#define CMD_BUF_LEN 24
static volatile char    s_cmd_buf[CMD_BUF_LEN];
static volatile int s_cmd_len;
static volatile char    s_cmd_line[CMD_BUF_LEN];
static volatile int s_cmd_ready;
static volatile uint8_t s_cmd_byte;
static volatile uint32_t s_cmd_last;      /* 最后一个字节的时刻：50ms 静默也算一行结束 */

void Grab6020_CmdStart(void)
{
    s_cmd_len = 0;
    s_cmd_ready = 0;
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_cmd_byte, 1);
}

static void cmd_finish(void)
{
    int i;

    if (s_cmd_len == 0) return;
    for (i = 0; i < s_cmd_len; i++) s_cmd_line[i] = s_cmd_buf[i];
    s_cmd_line[s_cmd_len] = 0;
    s_cmd_len = 0;
    s_cmd_ready = 1;
}

static void cmd_rx_byte(uint8_t c)
{
    s_cmd_last = HAL_GetTick();
    if (s_cmd_ready) return;                 /* 上一行还没被取走：丢新字节 */
    if (c == '\n' || c == '\r') {
        cmd_finish();
    } else if (s_cmd_len < (uint8_t)(CMD_BUF_LEN - 1)) {
        s_cmd_buf[s_cmd_len++] = (char)c;
    }
}

void Grab6020_CmdRxISR(void)
{
    cmd_rx_byte(s_cmd_byte);
#if (FOLLOW_RX_EN != 0)
    Follow_RxByte(s_cmd_byte);   /* 上位机路径规划帧（FOLLOW_RX_EN = 1 才收）*/
#endif
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_cmd_byte, 1);
}

uint8_t Grab6020_CmdPoll(char *out, uint8_t max)
{
    int n = 0;

    if (out == 0 || max < 2) return 0;
    __disable_irq();
    if (!s_cmd_ready && s_cmd_len > 0 && (HAL_GetTick() - s_cmd_last) > 50) {
        cmd_finish();
    }
    if (s_cmd_ready) {
        while (n < (uint8_t)(max - 1) && s_cmd_line[n] != 0) {
            out[n] = s_cmd_line[n];
            n++;
        }
        out[n] = 0;
        s_cmd_ready = 0;
    }
    __enable_irq();
    return (n > 0) ? 1 : 0;
}
