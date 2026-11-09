/* go_test.c —— 台架测试：世界坐标点到点 + 路径规划联调 */
/* ⚠ 串口字符串一律纯 ASCII：Keil 里中文显示乱码，混着就分不清"串口不通"和"编码不对" */
#include "main.h"
#include "usart.h"
#include "go_test.h"
#include "cmsis_os.h"     /* osDelay：测试函数在任务里跑 */
#include "Go.h"           /* GO / GO_Arrived / GO_Stop / GO_GetXTask */
#include "TaskFlow.h"     /* Motion_Home */
#include "Follow.h"       /* Follow_Set / Follow_Walk / Follow_WalkDone */
#include <string.h>
#include <stdio.h>



static void cal_puts(const char *s)
{
    HAL_UART_Transmit(&huart1, (const uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* ================= 世界坐标测试：GO_To(60,0,+180) → GO_To(0,0,0) ================= 第三个参数是**相对角**（加到当前目标朝向上）：① 走 60cm 并转 +180°；② 走回 (0,0)，不再转。
 * 与 GO() 同一套分离式转弯：第 ① 步先朝 0° 直行，**剩余 ≤ GO_TURN_LATE_CM(30cm) 才开始甩头**。 ⚠ 相对角会累加，所以每步只下发一次（见下面的 step_sent 标志）。 改路线只改 steps[][3]，一行一个点。 */
#define WORLD_DWELL_MS        3000u     /* 每步到位后停留：量尺/看现象；在打印之后，ms 列才是真运动时间 */
#define WORLD_STEP_TIMEOUT_MS 20000u

void Go_Test_WorldRun(void)
{
    static const float steps[][3] = {
        { 0.0f, 80.0f, 0.0f },    /* ① 到 (60,0)，最后 30cm 甩头转 +180° */
        { 0.0f, -80.0f, 0.0f },    /* ② 保持朝向，走回 (0,0) */
    };
    char     b[96];
    int      n;
    uint32_t k, t0;

    /* 归零 + 锁当前车头：任务系原点 = 按 0-1 这一刻的停车点 */
    t0 = HAL_GetTick();
    while (!Motion_Home()) {
        if ((HAL_GetTick() - t0) > 10000u) { cal_puts("W,ERR,no-pose\r\n"); return; }
        osDelay(20);
    }

    cal_puts("W,BEGIN,k,tar_x,tar_y,tar_yaw,x,y,yaw,ms,flag\r\n");

    for (k = 0; k < (uint32_t)(sizeof(steps) / sizeof(steps[0])); k++) {
        uint8_t  ok = 1;
        uint32_t t1 = HAL_GetTick();

        GO_To(steps[k][0], steps[k][1], steps[k][2]);   /* 绝对位置 + **相对**朝向 */

        while (!GO_Arrived()) {
            if ((HAL_GetTick() - t1) > WORLD_STEP_TIMEOUT_MS) { ok = 0; break; }
            osDelay(20);
        }

        n = snprintf(b, sizeof(b), "W,%lu,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%lu,%s\r\n",
                     (unsigned long)(k + 1u),
                     (double)steps[k][0], (double)steps[k][1], (double)steps[k][2],
                     (double)GO_GetXTask(), (double)GO_GetYTask(), (double)GO_GetYawTask(),
                     (unsigned long)(HAL_GetTick() - t1), ok ? "ok" : "TIMEOUT");
        if (n > 0 && n < (int)sizeof(b)) cal_puts(b);

        if (!ok) { GO_Stop(); cal_puts("W,ABORT\r\n"); return; }
        osDelay(WORLD_DWELL_MS);
    }

    GO_Stop();
    n = snprintf(b, sizeof(b), "W,DONE,%.2f,%.2f,%.1f\r\n",
                 (double)GO_GetXTask(), (double)GO_GetYTask(), (double)GO_GetYawTask());
    if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
}

/* ---------- 先直行后横移（非阻塞）---------- */
#define GO_TS_SEG_TIMEOUT_MS  15000u   /* 每段最长等这么久，等不到位也放行（只在测试模块内）*/
#define GO_TS_SETTLE_MS        300u     /* 必须在死区里**连续稳定**这么久才算真到位（防"路过死区就判到"导致冲过头）*/
#define GO_TS_TEST_VMAX_CMPS   20.0f    /* 台架测试时的速度上限 cm/s（车重打滑，50 太快）。0 = 用默认档 */
static uint8_t  s_ss_phase = 0;      /* 0 = 还没开始；1 = 直行段在走；2 = 补完段在走 */
static uint32_t s_ds_ok_t  = 0;    /* "连续稳定到位"计时的起始时刻（0 = 还没稳定），两个测试走位共用 */
static uint32_t s_ss_t0    = 0;      /* 本段开始时刻：超时用 */

void Go_Test_StraightThenSideReset(void)
{
    s_ss_phase = 0;
    s_ds_ok_t = 0;
}

uint8_t Go_Test_StraightThenSidePhase(void)
{
    return s_ss_phase;               /* 诊断：0/1/2 */
}

uint8_t Go_Test_StraightThenSide(uint8_t axis, float tx, float ty, float fork_cm)
{
    float now_x, now_y, d;

    if (s_ss_phase == 0) {                               /* 起点：决定要不要先直行 */
        now_x = GO_GetXTask();
        now_y = GO_GetYTask();
        d = axis ? (tx - now_x) : (ty - now_y);          /* 直行轴上还差多少（带符号）*/

        if ((d > fork_cm) || (d < -fork_cm)) {           /* ① 还远：先只走直行轴，另一轴保持 */
            /* 直行段目标 = "目标的直行轴坐标 + 当前另一轴"，**不提前收油**：
             * 这样到切换点时车速还是满的，换目标才连贯（提前收油会先减速再重加速 ⇒ 看着顿一下）。*/
            if (axis) GO_To(tx, now_y, 0.0f);
            else      GO_To(now_x, ty, 0.0f);
            s_ss_phase = 1;
            s_ss_t0    = HAL_GetTick();
            return 1;
        }
        s_ss_phase = 2;                                  /* 已经够近：直接补完 */
        GO_To(tx, ty, 0.0f);
        s_ss_t0 = HAL_GetTick();
        return 1;
    }

    if (s_ss_phase == 1) {                               /* ② 直行段：剩 fork 就换目标，不等停车 */
        now_x = GO_GetXTask();
        now_y = GO_GetYTask();
        d = axis ? (tx - now_x) : (ty - now_y);

        if (((d > fork_cm) || (d < -fork_cm)) &&         /* 还没到切换点 */
            !GO_Arrived() &&                             /* 也没停住（兜底：到位就切）*/
            ((HAL_GetTick() - s_ss_t0) < GO_TS_SEG_TIMEOUT_MS)) return 1;

        s_ss_phase = 2;
        GO_To(tx, ty, 0.0f);                             /* 直接改目标：车不停，接着横移 */
        s_ss_t0 = HAL_GetTick();
        return 1;
    }

    /* ③ 等补完段到位：连续 GO_TS_SETTLE_MS 稳定在死区内才算到（防冲过头）*/
    if (GO_Arrived()) {
        if (s_ds_ok_t == 0) {
            s_ds_ok_t = HAL_GetTick();
        } else if ((HAL_GetTick() - s_ds_ok_t) >= GO_TS_SETTLE_MS) {
            s_ds_ok_t  = 0;
            s_ss_phase = 0;
            return 0;
        }
        return 1;
    }

    s_ds_ok_t = 0;
    if ((HAL_GetTick() - s_ss_t0) < GO_TS_SEG_TIMEOUT_MS) return 1;
    s_ss_phase = 0;
    return 0;
}
/* ---------- 斜冲 + 直行收尾（非阻塞）----------
 * 为什么这样走：
 *   · 纯横移时横向轮效率低（车重打滑），实测走 15 只能到 9~14；
 *   · 两轴同时动（斜走）所有轮都在滚，抓地好；
 *   · 但直接斜走到底，最后几个 cm 又变成"Y 误差占绝对主导"(pid.c:70) ⇒ 还是纯横移；
 *   · 所以：斜冲时直行轴**多冲 |横移误差|**（45°），到位前切一段**纯直行**退回目标 —— 收尾靠直行，准。
 * 例：(0,0)→(15,15)：斜冲到 (30,15) → 纯直行从 x=30 退回 x=15。*/
#define GO_TS_DIAG_MIN_CM     2.0f     /* 横移误差小于它就不值得斜冲，直接去目标 */
#define GO_TS_DIAG_ARRIVE_CM  1.0f     /* 斜冲段离目标还剩这么多就切"直行收尾"（不等完全停）*/
#define GO_TS_DIAG_K          1.0f     /* 斜冲时直行轴多冲 = |横移误差| × K。1.0 = 45°；调小则角度更大(横移占比高、容易打滑)，调大则收尾段更长 */

static uint8_t  s_ds_phase = 0;    /* 0 = 起点；1 = 斜冲段；2 = 直行收尾段 */
static uint32_t s_ds_t0    = 0;
static float    s_ds_ix    = 0.0f; /* 斜冲段目标（收尾段判"够近"用）*/
static float    s_ds_iy    = 0.0f;

void Go_Test_DiagThenStraightReset(void)
{
    s_ds_phase = 0;
    s_ds_ok_t = 0;
}

uint8_t Go_Test_DiagThenStraightPhase(void)
{
    return s_ds_phase;             /* 诊断：0/1/2 */
}

uint8_t Go_Test_DiagThenStraight(uint8_t axis, float tx, float ty, float over_cm)
{
    float now_x, now_y, m, d;

    if (s_ds_phase == 0) {                                /* 起点：算斜冲目标 */
        now_x = GO_GetXTask();
        now_y = GO_GetYTask();
        m = axis ? (ty - now_y) : (tx - now_x);           /* 横移轴的误差（带符号）*/

        if (over_cm < 0.0f) {                                 /* 负数 = 不斜冲、不收尾，直接去 (tx,ty) */
            s_ds_ix = tx;  s_ds_iy = ty;
        } else {
        if (over_cm == 0.0f) over_cm = m * GO_TS_DIAG_K;      /* 0 = 自动：横移误差 × K */

        if ((over_cm > GO_TS_DIAG_MIN_CM) || (over_cm < -GO_TS_DIAG_MIN_CM)) {
            if (axis) { s_ds_ix = tx + over_cm; s_ds_iy = ty; }  /* 直行轴多冲 over_cm */
            else      { s_ds_ix = tx;           s_ds_iy = ty + over_cm; }
        } else {
            s_ds_ix = tx;  s_ds_iy = ty;                  /* 多冲量太小：直接去 */
        }
        }

        GO_To(s_ds_ix, s_ds_iy, 0.0f);
        s_ds_phase = 1;
        s_ds_t0    = HAL_GetTick();
        return 1;
    }

    if (s_ds_phase == 1) {                                /* ① 斜冲段 */
        now_x = GO_GetXTask();
        now_y = GO_GetYTask();
        d = axis ? (s_ds_ix - now_x) : (s_ds_iy - now_y);

        if (((d > GO_TS_DIAG_ARRIVE_CM) || (d < -GO_TS_DIAG_ARRIVE_CM)) &&
            !GO_Arrived() &&
            ((HAL_GetTick() - s_ds_t0) < GO_TS_SEG_TIMEOUT_MS)) return 1;

        s_ds_phase = 2;
        GO_To(tx, ty, 0.0f);                              /* ② 纯直行收尾（准）*/
        s_ds_t0 = HAL_GetTick();
        return 1;
    }

    /* ③ 等收尾段到位：必须**连续 GO_TS_SETTLE_MS 都稳定在死区里**才算真到位。
     *    否则车以高速路过死区时 arrived 立刻置 1、控制器当拍输出 0 ⇒ 惯性冲过头 0.5~1.5cm。*/
    if (GO_Arrived()) {
        if (s_ds_ok_t == 0) {
            s_ds_ok_t = HAL_GetTick();
        } else if ((HAL_GetTick() - s_ds_ok_t) >= GO_TS_SETTLE_MS) {
            s_ds_ok_t  = 0;
            s_ds_phase = 0;
            return 0;
        }
        return 1;
    }

    s_ds_ok_t = 0;                                     /* 冲出死区：重新计时，让控制器拉回来 */
    if ((HAL_GetTick() - s_ds_t0) < GO_TS_SEG_TIMEOUT_MS) return 1;
    s_ds_phase = 0;
    return 0;
}

/* ---------- 台架：里程计标定（沿车头方向直行 dist cm，停住打 INS 值）----------
 * 用途：量"里程计 1cm = 实物多少 cm"（尺度）。跑 dist=100，实物量到 84cm ⇒ 尺度 0.84。
 * ⚠ 车头方向 = 任务系 +X（Motion_Home 把停车那一刻的车头锁成任务 X 轴）*/
void Go_Test_OdoRun(uint8_t axis, float dist)
{
    char     b[96];
    int      n;
    uint32_t t0 = HAL_GetTick();
    uint32_t tpr;

    cal_puts("O,HOME\r\n");
    while (!Motion_Home()) {
        if ((HAL_GetTick() - t0) > 10000u) { cal_puts("O,ERR,no-imu\r\n"); break; }
        osDelay(20);
    }
    cal_puts("O,BEGIN\r\n");

    if (axis) GO_To(dist, 0.0f, 0.0f);            /* axis=1：沿任务 X（车头方向）直行 */
    else      GO_To(0.0f, dist, 0.0f);            /* axis=0：沿任务 Y 纯横向移动 */
    tpr = HAL_GetTick();
    for (;;) {
        if ((HAL_GetTick() - t0) > 30000u) break;

        if ((HAL_GetTick() - tpr) >= 200u) {
            tpr = HAL_GetTick();
            n = snprintf(b, sizeof(b), "O,%.2f,%.2f,%.1f,%u\r\n",
                         (double)GO_GetXTask(), (double)GO_GetYTask(),
                         (double)GO_GetYawTask(), (unsigned)GO_Arrived());
            if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        }
        if (GO_Arrived()) break;
        osDelay(20);
    }
    cal_puts("O,DONE\r\n");

    for (;;) {                                  /* 停住：每秒打一次 INS 值，去量实物 */
        GO_Stop();
        n = snprintf(b, sizeof(b), "O,STOP,%.2f,%.2f,%.1f\r\n",
                     (double)GO_GetXTask(), (double)GO_GetYTask(), (double)GO_GetYawTask());
        if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        osDelay(1000);
    }
}

/* ---------- 台架：跑一遍「斜冲 + 直行收尾」，跑完停住 ---------- */
void Go_Test_DiagThenStraightRun(uint8_t axis, float tx, float ty)
{
    char     b[96];
    int      n;
    uint32_t t0 = HAL_GetTick();
    uint32_t tpr;
    float    gx, gy;                                /* 实时目标（GO_GetTar）*/

    cal_puts("D,HOME\r\n");
    while (!Motion_Home()) {
        if ((HAL_GetTick() - t0) > 10000u) { cal_puts("D,ERR,no-imu\r\n"); break; }
        osDelay(20);
    }
    cal_puts("D,BEGIN\r\n");

    GO_SetVmax(GO_TS_TEST_VMAX_CMPS);            /* 测试期间限速 */

    Go_Test_DiagThenStraightReset();
    tpr = HAL_GetTick();
    for (;;) {
        uint8_t busy = Go_Test_DiagThenStraight(axis, tx, ty, 0.0f);   /* ⚠ 20ms 一拍 */

        if ((HAL_GetTick() - tpr) >= 200u) {
            tpr = HAL_GetTick();
            GO_GetTar(&gx, &gy);                    /* 每行都取一次实时目标 */
            n = snprintf(b, sizeof(b), "D,%.2f,%.2f,%.1f,%u,%.2f,%.2f\r\n",
                         (double)GO_GetXTask(), (double)GO_GetYTask(),
                         (double)GO_GetYawTask(), (unsigned)Go_Test_DiagThenStraightPhase(),
                         (double)gx, (double)gy);          /* 末尾两个 = 控制器当前目标 */
        }
        if (!busy) break;
        osDelay(20);
    }
    cal_puts("D,DONE\r\n");
    GO_SetVmax(0.0f);                              /* 恢复默认速度上限 */

    for (;;) {                                     /* 每秒打最终位姿，方便量误差 */
        GO_Stop();
        n = snprintf(b, sizeof(b), "D,STOP,%.2f,%.2f,%.1f\r\n",
                     (double)GO_GetXTask(), (double)GO_GetYTask(), (double)GO_GetYawTask());
        if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        osDelay(1000);
    }
}

/* ---------- 台架：初始走到左基准前置落点 (GO_TS_PREGO_L_X, GO_TS_PREGO_L_Y) ----------
 * 走法 = 斜冲 + 直行收尾（over_cm = 0），与 Go_Test_DiagThenStraightRun 完全同一趟：
 *   斜冲到 (X+over, Y) 再纯直行退回 (X, Y)。跑完停住，每秒打 D,STOP,<x>,<y>,<yaw> 方便拿卷尺量。
 * ⚠ 产线任务 1 的前置段（Follow.c 318）用的是 pv = -1（不斜冲不收尾、直接去），两者落点会略有差别，
 *   这里按"更准"的斜冲+收尾走，填 TF_PRE_GO_*_L 之前先确认产线那一趟的实际落点。*/
#define GO_TS_PREGO_L_X   20.0f     /* 前置落点 X（左基准，待实测）*/
#define GO_TS_PREGO_L_Y   20.0f     /* 前置落点 Y（左基准，待实测）*/

void Go_Test_PreGoLeftRun(void)
{
    Go_Test_DiagThenStraightRun(1, 22, 22);
}

void Go_Test_GoToPreRun(void)
{
    char     b[96];
    int      n;
    uint32_t t0;
    uint32_t tpr;

    cal_puts("G,HOME\r\n");
    t0 = HAL_GetTick();
    while (!Motion_Home()) {                      /* 归零 + 锁航向：任务系原点 = 这一刻的停车点 */
        if ((HAL_GetTick() - t0) > 10000u) {
            cal_puts("G,ERR,no-imu\r\n");
            break;
        }
        osDelay(20);
    }
    cal_puts("G,BEGIN\r\n");
    GO_SetVmax(GO_TS_TEST_VMAX_CMPS);
    GO_To(11, 22, 0);
    t0  = HAL_GetTick();
    tpr = HAL_GetTick();
    for (;;) {
        if ((HAL_GetTick() - t0) > GO_TS_SEG_TIMEOUT_MS) {
            cal_puts("G,TIMEOUT\r\n");
            break;
        }

        if ((HAL_GetTick() - tpr) >= 200u) {
            tpr = HAL_GetTick();
            n = snprintf(b, sizeof(b), "G,%.2f,%.2f,%.1f,%u\r\n",
                         (double)GO_GetXTask(), (double)GO_GetYTask(),
                         (double)GO_GetYawTask(), (unsigned)GO_Arrived());
            if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        }
        if (GO_Arrived()) break;
        osDelay(20);
    }
    cal_puts("G,DONE\r\n");

    for (;;) {                                    /* 停住：每秒打一次位姿，拿去量实物 */
        GO_Stop();
        n = snprintf(b, sizeof(b), "G,STOP,%.2f,%.2f,%.1f\r\n",
                     (double)GO_GetXTask(), (double)GO_GetYTask(), (double)GO_GetYawTask());
        if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        osDelay(1000);
    }
}

/* ---------- 台架：走一遍「先直行后横移」，跑完停住（阻塞，直接调用即可看现象） ---------- */
void Go_Test_StraightThenSideRun(uint8_t axis, float tx, float ty, float fork_cm)
{
    char     b[96];
    int      n;
    uint32_t t0 = HAL_GetTick();

    cal_puts("T,HOME\r\n");
    while (!Motion_Home()) {                      /* 归零 + 锁航向：任务系原点 = 这一刻的停车点 */
        if ((HAL_GetTick() - t0) > 10000u) {
            cal_puts("T,ERR,no-imu\r\n");
            break;
        }
        osDelay(20);
    }
    cal_puts("T,BEGIN\r\n");

    GO_SetVmax(GO_TS_TEST_VMAX_CMPS);            /* 测试期间限速 */

    Go_Test_StraightThenSideReset();
    {
        uint32_t tpr = HAL_GetTick();

        for (;;) {
            /* ⚠ 必须 20ms 调一次：这就是控制节拍。隔久了会在直行段终点停住等下一次检查 */
            uint8_t busy = Go_Test_StraightThenSide(axis, tx, ty, fork_cm);

            if ((HAL_GetTick() - tpr) >= 200u) {          /* 打印节拍单独控制，不影响控制 */
                tpr = HAL_GetTick();
                n = snprintf(b, sizeof(b), "T,%.2f,%.2f,%.1f,%u\r\n",
                             (double)GO_GetXTask(), (double)GO_GetYTask(),
                             (double)GO_GetYawTask(), (unsigned)Go_Test_StraightThenSidePhase());
                if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
            }
            if (!busy) break;
            osDelay(20);                                  /* 控制节拍 20ms */
        }
    }
    cal_puts("T,DONE\r\n");
    GO_SetVmax(0.0f);                              /* 恢复默认速度上限 */

    for (;;) {                                    /* 停住：每秒打最终位姿，方便量误差 */
        GO_Stop();
        n = snprintf(b, sizeof(b), "T,STOP,%.2f,%.2f,%.1f\r\n",
                     (double)GO_GetXTask(), (double)GO_GetYTask(), (double)GO_GetYawTask());
        if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        osDelay(1000);
    }
}

/* ================= 路径规划联调：任务 1 → 2 → 3，跑完停 ================= 结构就三件事： ⓪ Motion_Home() 归零 + 锁当前车头（**必须有，见下面的
 * ⚠**） ① Follow_Set() 把三条路线写进 path/ang（不改别的）
  * ② 依次跑任务 1 / 2 / 3，每个任务之间停 200ms ③ GO_Stop() 停住 ⚠ Follow_Walk 是**非阻塞**的：一次调用只推进一步（下发一对 / 查一次到位）， 所以每个任务必须"循环调到
  * Follow_WalkDone 为真"才算走完； 直接写 "Follow_Walk(1); osDelay(200); Follow_Walk(2);" 会把任务 1 半途丢掉。 ⚠⚠ 为什么"调 Follow_Set
  * + Follow_Walk 和直接调 GO_To 不一样"： GO_To 的目标是**任务系绝对坐标**，任务系原点/朝向由最近一次 Motion_Home 决定 （GO_Home 里 INS_ZeroPos +
  * yaw0 = 当前车头）。 Go_Test_WorldRun 在测试开头调了 Motion_Home，所以原点就是"按 0-1 这一刻的停车点"； 而本函数原来只在**开机**时（TaskFlow_Ready
  * 里）锁过一次， 中间只要摆过车 / 推过车 / 车头转过一点，x、y、yaw0 就全偏了 —— 于是同样的 GO_To(20,10,90) 落点完全不同，看起来就像"宏的 x/y 没生效"。 在这里补一次
  * Motion_Home，两者就完全等价。 */
#define FOLLOW_SEQ_GAP_MS   200u      /* 任务之间停 200ms */

/* 台架：跑一个任务，边跑边打进度；每组 30s 兜底（超时就跳过并打 F,HANG）*/
static void seq_run(uint8_t task)
{
    char     b[128];
    int      n;
    uint32_t tk  = HAL_GetTick();
    uint32_t tpr = 0;
    uint32_t t0  = HAL_GetTick();
    float    x, y, tx, ty;

    for (;;) {
        Follow_Walk(task);
        if (Follow_WalkDone(task)) break;

        if ((HAL_GetTick() - tpr) >= 500u) {          /* 每 0.5s 一行进度 */
            tpr = HAL_GetTick();
            x = GO_GetXTask();
            y = GO_GetYTask();
            GO_GetTar(&tx, &ty);
            n = snprintf(b, sizeof(b), "F,%u,%u,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%.1f\r\n",
                         (unsigned)task, (unsigned)Follow_WalkIdx(task),
                         (double)x, (double)y, (double)GO_GetYawTask(),
                         (double)tx, (double)ty, (double)GO_GetYawTar(),
                         (double)GO_Arrived());
            if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        }
        if ((HAL_GetTick() - t0) > 30000u) {          /* 30s 还没走完：报出来并跳过 */
            n = snprintf(b, sizeof(b), "F,HANG,%u,%u\r\n",
                         (unsigned)task, (unsigned)Follow_WalkIdx(task));
            if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
            break;
        }
        tk = HAL_GetTick();
        (void)tk;
        osDelay(20);
    }
}

void Go_Test_FollowSeq(void)
{
    uint8_t  t;
    uint32_t t0;
    char     b[96];
    int      n;

    /* ① 归零 + 锁当前车头：任务系原点 = 这一刻的停车点，yaw0 = 这一刻的车头 */
    t0 = HAL_GetTick();
    while (!Motion_Home()) {
        if ((HAL_GetTick() - t0) > 10000u) return;     /* IMU 一帧都没来过：跑不了 */
        osDelay(20);
    }

    /* ② **不再灌本地路线**：直接用上位机发进来的 path[]/ang[]，
     *   把所有有数据的任务号（1..FOLLOW_TASK_MAX）依次跑完。*/
    cal_puts("F,RUN\r\n");

    for (t = 1u; t <= FOLLOW_TASK_MAX; t++) {
        if (!Follow_HasTask(t)) continue;              /* 这一段没收到：跳过 */

        seq_run(t);
        osDelay(FOLLOW_SEQ_GAP_MS);
    }

    GO_Stop();                                         /* ③ 停住 */
    cal_puts("F,ALLDONE\r\n");

    /* ④ 死循环停住：不退出、不再动，每秒打一行最终位姿，方便现场看跑得怎么样 */
    for (;;) {
        n = snprintf(b, sizeof(b), "F,STOP,%.2f,%.2f,%.1f\r\n",
                     (double)GO_GetXTask(), (double)GO_GetYTask(), (double)GO_GetYawTask());
        if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        osDelay(1000);
    }
}

/* ================= 上位机路线测试（不碰相机、不碰手臂）================= ① 等上位机把路线发进来（任意任务号 path_n != 0；
  * 最多等 30s，超时打 F,ERR,no-route） ② 收到后先把收进来的整张表打一遍，确认解析对不对： R,任务号,对数,格号,角,格号,角,…（最多 6 对） ③
  * 依次跑有数据的任务号：Follow_Walk(N) + Follow_WalkDone(N) 轮询， 跑的过程中**每 1 秒打一行**：F,任务号,当前x,当前y,当前yaw,目标x,目标y ④ 全部跑完
  * GO_Stop() + F,DONE 改行为只改这里的超时/间隔；路线本身完全由上位机决定。 */
#define ROUTE_WAIT_MS   30000u      /* 等上位机发路线的最长时间 */
#define ROUTE_PRINT_MS   1000u      /* 跑的过程中多少毫秒打一行 */

void Go_Test_Route(void)
{
    char     b[160];
    int      n;
    uint32_t tk, t0, tpr;
    float    tx, ty;

    /* ① 归零 + 锁车头 */
    t0 = HAL_GetTick();
    while (!Motion_Home()) {
        if ((HAL_GetTick() - t0) > 10000u) { cal_puts("F,ERR,no-pose\r\n"); return; }
        osDelay(20);
    }

    /* ② 等上位机的路线（任意任务号有数据就算收到）*/
    cal_puts("F,WAIT\r\n");
    t0 = HAL_GetTick();
    for (;;) {
        uint8_t any = 0;

        for (tk = 1; tk <= 10u; tk++) {
            if (path_n[tk] != 0u) { any = 1; break; }
        }
        if (any) break;
        if ((HAL_GetTick() - t0) > ROUTE_WAIT_MS) { cal_puts("F,ERR,no-route\r\n"); return; }
        osDelay(20);
    }

    /* ③ 把收进来的表打一遍（确认格号/角度解析正确）*/
    for (tk = 1; tk <= 10u; tk++) {
        uint8_t j;

        if (path_n[tk] == 0u) continue;

        n = snprintf(b, sizeof(b), "R,%u,%u", (unsigned)tk, (unsigned)path_n[tk]);
        if (n <= 0 || n >= (int)sizeof(b) - 2) continue;
        for (j = 1; j <= path_n[tk] && j <= 6u; j++) {
            int m = snprintf(b + n, sizeof(b) - (uint16_t)n, ",%u,%d",
                             (unsigned)path[tk][j], (int)ang[tk][j]);
            if (m <= 0 || (n + m) >= (int)sizeof(b) - 2) break;
            n += m;
        }
        b[n++] = '\r';
        b[n++] = '\n';
        b[n]   = '\0';
        cal_puts(b);
    }

    /* ④ 依次跑 */
    for (tk = 1; tk <= 10u; tk++) {
        if (path_n[tk] == 0u) continue;

        tpr = HAL_GetTick();
        while (!Follow_WalkDone(tk)) {
            Follow_Walk(tk);                       /* 每 20ms 推进一步 */

            if ((HAL_GetTick() - tpr) >= ROUTE_PRINT_MS) {   /* 每 1 秒一行 */
                tpr = HAL_GetTick();
                GO_GetTar(&tx, &ty);
                n = snprintf(b, sizeof(b), "F,%u,%.2f,%.2f,%.1f,%.2f,%.2f\r\n",
                             (unsigned)tk,
                             (double)GO_GetXTask(), (double)GO_GetYTask(),
                             (double)GO_GetYawTask(), (double)tx, (double)ty);
                if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
            }
            osDelay(20);
        }
        osDelay(200);                              /* 任务之间停一下 */
    }

    GO_Stop();
    cal_puts("F,DONE\r\n");

    /* 死循环停住：不退出、不再动，每秒打一行最终位姿，方便现场看跑得怎么样 */
    for (;;) {
        n = snprintf(b, sizeof(b), "F,STOP,%.2f,%.2f,%.1f\r\n",
                     (double)GO_GetXTask(), (double)GO_GetYTask(), (double)GO_GetYawTask());
        if (n > 0 && n < (int)sizeof(b)) cal_puts(b);
        osDelay(1000);
    }
}
