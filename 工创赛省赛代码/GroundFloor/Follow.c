/* Follow.c —— 路径规划帧解析 + 格点数组（协议、50 个宏都在 Follow.h）。 ISR 里只做整数拼帧/校验/入表：无浮点、无打印，最坏一帧几十个字符，几十微秒量级。 */
#include "Follow.h"
#include "Go.h"          /* GO_To / GO_Arrived */
#include "../Test/Go_Test/go_test.h"   /* Go_Test_DiagThenStraight：斜冲+直行收尾（相对路径，Keil 的 include 路径里没有 Test/Go_Test）*/

/* ===================== 50 个宏 → 两张点表 → x[]/y[] ===================== 宏在 Follow.h 里（那边填数）；这里只负责按 follow_side 选一套拆进数组。
 * ⚠ 表比格号多一项：下标 0 空着（没有 0 号格），这样 x[11] 的下标就是格号。 */
typedef struct { float x, y; } FollowPt_t;

static const FollowPt_t k_pt_left[FOLLOW_CELL_NUM] = {
    { 0.0f, 0.0f },   /* 0 号：没有这个格，占位 */
    CELL_L1,  CELL_L2,  CELL_L3,  CELL_L4,  CELL_L5,
    CELL_L6,  { 0.0f, 0.0f },  CELL_L8,  { 0.0f, 0.0f },  CELL_L10,   /* 7/9 不用 */
    CELL_L11, CELL_L12, CELL_L13, CELL_L14, CELL_L15,
    CELL_L16, { 0.0f, 0.0f },  CELL_L18, { 0.0f, 0.0f },  CELL_L20,   /* 17/19 不用 */
    CELL_L21, CELL_L22, CELL_L23, CELL_L24, CELL_L25,
};

static const FollowPt_t k_pt_right[FOLLOW_CELL_NUM] = {
    { 0.0f, 0.0f },   /* 0 号：占位 */
    CELL_R1,  CELL_R2,  CELL_R3,  CELL_R4,  CELL_R5,
    CELL_R6,  { 0.0f, 0.0f },  CELL_R8,  { 0.0f, 0.0f },  CELL_R10,   /* 7/9 不用 */
    CELL_R11, CELL_R12, CELL_R13, CELL_R14, CELL_R15,
    CELL_R16, { 0.0f, 0.0f },  CELL_R18, { 0.0f, 0.0f },  CELL_R20,   /* 17/19 不用 */
    CELL_R21, CELL_R22, CELL_R23, CELL_R24, CELL_R25,
};

/* 真正的格点数组：格号直接当下标 —— GO_To(x[11], y[11], 0.0f)。 */
float x[FOLLOW_CELL_NUM];
float y[FOLLOW_CELL_NUM];

int follow_side = 1;            /* 0 = 左出发，1 = 右出发（外部可直接赋值） */

static int s_side_applied = 0xFF;   /* 已经拆进数组的方向；0xFF = 还没拆过 */

/* 把当前 follow_side 对应的那套点拆进 x[]/y[]（方向没变就什么都不做） */
static void side_apply(void)
{
    int s = (uint8_t)(follow_side ? 1 : 0);
    const FollowPt_t *p;
    int i;

    if (s == s_side_applied) return;
    s_side_applied = s;
    p = s ? k_pt_right : k_pt_left;
    for (i = 0; i < FOLLOW_CELL_NUM; i++) {
        x[i] = p[i].x;
        y[i] = p[i].y;
    }
}

void Follow_SideInit(void)
{
    s_side_applied = 0xFF;          /* 强制重填 */
    side_apply();
}

void Follow_SetSide(uint8_t right_start)
{
    follow_side = right_start ? 1 : 0;
    Follow_SideInit();
}


/* ============================ 路线存放 ============================ 成对存：path[任务号][第 i 对] = 格号；ang[任务号][第 i 对] = 相对角（°）； path_n[任务号] = 有几对（0 = 还没收到）。
 * 行/列都从 1 开始用，0 号空着；所以数组是 [11][11] 而不是 [10][10] —— 任务号能到 10、对数能到 10，1 起下标必须多留一格，否则正好越界。 */
uint8_t path[FOLLOW_TASK_MAX + 1][FOLLOW_CELL_MAX + 1];
int16_t ang [FOLLOW_TASK_MAX + 1][FOLLOW_CELL_MAX + 1];
uint8_t path_n[FOLLOW_TASK_MAX + 1];

static uint8_t s_walk[FOLLOW_TASK_MAX + 1];   /* s_walk[N] = 任务 N 走到第几对（1 起），0 号空置 */
static uint8_t s_sent[FOLLOW_TASK_MAX + 1];   /* s_sent[N] = 当前这一对的目标已下发（相对角不能重复下发） */
static uint8_t s_rot2[FOLLOW_TASK_MAX + 1];   /* 0=正常；1=位置段已发/在走；2=位置段到位，等发原地补转 */
static uint8_t s_pre15;                      /* 任务1 的扫码位前置：0=没做；1=已下发在走；2=已完成/不需要 */

/* 任务1 的前置落点（右基准用下面 TF_PRE_GO_*_R 那套；左基准已改走 TF_PRE_LEFT_*）：先用「斜冲+直行收尾」走到这儿，第一对的目标点也用它 */
/* 任务1 前置落点（右基准实测值；**左基准已改走 TF_PRE_LEFT_***，下面四个 _L 宏保留但不再被引用）*/
#define TF_PRE_GO_Y_R         11.0f   /* 前置落点的 Y（右）*/
#define TF_PRE_GO_PASS_X_R    22.0f   /* 第一格不是扫码位：只斜走到这儿就停（不收尾）*/
#define TF_PRE_GO_ROT_X_R     20.0f   /* 第一格是扫码位：斜冲到 22 后退到这儿，再原地转 */
#define TF_PRE_GO_ROT_OVER_R   5.0f   /* 上面那一趟的收尾距离 = 22 - 17 */


#define TF_PRE_GO_Y_L         11.0f   /* 左基准已不再引用（改走 TF_PRE_LEFT_*），保留备用 */
#define TF_PRE_GO_PASS_X_L    22.0f   /* 同上，保留备用 */
#define TF_PRE_GO_ROT_X_L     17.0f   /* 同上，保留备用 */
#define TF_PRE_GO_ROT_OVER_L   5.0f   /* 同上，保留备用 */

/* 左基准实测版（2026-11-05 台架实测）：任务1 第一对之前**不管第一格是不是 5**，都先限速直给
 *   GO_To(X, Y_CMD)，到位后把任务系 Y 纠正成 Y_FIX（实测下发 22 只走到 20），再照常走路线第一段。
 *   右基准不受影响，仍走上面 TF_PRE_GO_*_R 那一套。*/
#define TF_PRE_LEFT_X        11.0f   /* 左基准：固定先到的 X（任务系 cm）*/
#define TF_PRE_LEFT_Y_CMD    21.0f   /* 左基准：固定先到的 Y（下发值）*/
#define TF_PRE_LEFT_Y_FIX    19.0f   /* 左基准：到位后把任务系 Y 纠正成这个值（实测）*/
#define TF_PRE_LEFT_VMAX     20.0f   /* 左基准：这一段限速 cm/s */

/* ISR 拼帧状态（只有 RX 中断写，任务只读结果表） */
static int  s_in;          /* 1 = 正在 '(' ... ')' 之间 */
static int  s_ntok;        /* 已提交的 token 数 */
static int  s_has_digit;   /* 当前 token 里已经有数字 */
static int  s_neg;         /* 当前 token 带负号 */
static uint16_t s_val;         /* 当前 token 值（绝对值） */
/* ---- 诊断（只读，遥测用）：看上位机到底发进来什么 ---- */
volatile uint32_t follow_rx  = 0;      /* 收到几个帧头 '@' */
volatile uint32_t follow_ok  = 0;      /* 成功入表几帧 */
volatile uint32_t follow_bad = 0;      /* 解析到了但被判非法（格号/角度/对数/任务号）*/
char              follow_raw[64];      /* 最近一帧的原文（含 '@' … '%'）*/
static int    s_raw_n = 0;

static int16_t  s_tok[(FOLLOW_CELL_MAX * 2) + 1];  /* [0]=任务号，后面成对：格号, 相对角 */

/* 提交当前 token：返回 0 = 空 token / 太多了 */
static uint8_t tok_commit(void)
{
    if (!s_has_digit) return 0;
    if (s_ntok >= (uint8_t)((FOLLOW_CELL_MAX * 2) + 1)) return 0;
    s_tok[s_ntok++] = (int16_t)(s_neg ? -(int16_t)s_val : (int16_t)s_val);
    s_val = 0;
    s_has_digit = 0;
    s_neg = 0;
    return 1;
}


/* 格号是否可用：1..25，且不是 7/9/17/19 —— 这 4 个内圈格子没有定义，上位机不会发； 真收到就整帧判非法，绝不让车跑去 (0,0)。 */
static uint8_t cell_ok(uint8_t c)
{
    if (c < 1 || c >= FOLLOW_CELL_NUM) return 0;
    if (c == 7 || c == 9 || c == 17 || c == 19) return 0;
    return 1;
}

/* 一帧收完：校验 + 入表。格式 = 任务号 + (格号, 相对角) × n */
static uint8_t store_frame(void)
{
    int i, n, task;
    int16_t a;

    if (s_ntok < 3) { return 0; }               /* 至少要 任务号 + 一对 */

    task = (uint8_t)s_tok[0];
    if (task < 1 || task > FOLLOW_TASK_MAX) { return 0; }

    if (((s_ntok - 1) & 1) != 0) { return 0; }/* 任务号后面必须成对 */
    n = (uint8_t)((s_ntok - 1) / 2);
    if (n > FOLLOW_CELL_MAX) { return 0; }

    for (i = 0; i < n; i++) {
        if (!cell_ok((uint8_t)s_tok[1 + 2 * i])) { return 0; }   /* 1..25 且排除 7/9/17/19 */
        a = s_tok[2 + 2 * i];
        if (a < -360 || a > 360) { return 0; }                   /* 相对角 ±360 以外当坏帧 */
    }


    for (i = 0; i < n; i++) {
        path[task][i + 1] = (uint8_t)s_tok[1 + 2 * i];   /* 第 1 对放 [1] */
        ang [task][i + 1] = s_tok[2 + 2 * i];
    }
    path_n[task] = n;                                    /* 最后写对数：读到非 0 时内容已就绪 */
    return 1;
}


/* 帧格式（新）：'@' 任务号 '+' 格号 '(' 角度 ')' … '%' 例：@1+21(90)+11(90)% → 任务 1：走到 21 号格并转 +90° → 走到 11 号格并转 +90° 解析上把 '+' '(' ')' 统统当**分隔符**，token 序列 = 任务号, 格号,
 * 角度, 格号, 角度 … ⚠ ')' 只是角度结束，**不是帧尾**；帧尾只认 '%'。 */
void Follow_RxByte(uint8_t b)
{
    if (b == '@') {                                      /* 帧头：任何时候都能重开 */
        s_in = 1; s_ntok = 0; s_has_digit = 0; s_neg = 0; s_val = 0;
        follow_rx++;                                     /* 诊断：数帧头 */
        s_raw_n = 0;
        follow_raw[s_raw_n++] = '@';
        return;
    }
    if (s_in && s_raw_n < (uint8_t)(sizeof(follow_raw) - 2)) {   /* 抓原文（到 '%' 为止）*/
        follow_raw[s_raw_n++] = (char)b;
        follow_raw[s_raw_n]   = '\0';
    }
    if (!s_in) return;                                   /* 帧外字符全丢 */

    if (b >= '0' && b <= '9') {
        s_has_digit = 1;
        s_val = (uint16_t)(s_val * 10 + (uint16_t)(b - '0'));
        if (s_val > 999) s_val = 999;                  /* 饱和：随后必被判越界 */
        return;
    }
    if (b == '-') {
        /* 负号两种写法都吃："21(-90)" 或 "21-90"（少发一个括号） */
        if (s_has_digit && !tok_commit()) { s_in = 0; return; }
        s_neg = 1;
        return;
    }
    if (b == '+' || b == '(' || b == ')') {              /* 分隔符：把当前数交上去 */
        (void)tok_commit();                              /* ')' 后紧跟 '+' 会产生空 token，忽略即可 */
        return;
    }
    if (b == '%') {                                      /* 帧尾 */
        (void)tok_commit();                              /* 末尾刚好在 ')' 之后时没有待提交的数 */
        if (store_frame()) follow_ok++; else follow_bad++;   /* 诊断：入表 / 被拒 */
        s_in = 0;
        return;
    }
    if (b == '\r' || b == '\n' || b == ' ' || b == '\t') return;   /* 帧内空白忽略 */

    s_in = 0;                                            /* 其它字符 = 坏帧，整帧丢弃 */
}

/* 把一条路线直接写进表里（绕过串口，本地表用）：下面 4 行与 store_frame() 写 path/ang 的 4 行逐字对应，改一处必须同时改另一处 */
static void set_one(uint8_t task, const int16_t *pairs, uint8_t n)
{
    int i;

    for (i = 0; i < n; i++) {
        path[task][i + 1] = (uint8_t)pairs[2 * i];
        ang [task][i + 1] = pairs[2 * i + 1];
    }
    path_n[task] = n;
    s_walk[task] = 0;                        /* 走位指针归零：下次 Follow_Walk 从第 1 对开始 */
    s_sent[task] = 0;
    s_rot2[task] = 0;
    s_pre15 = 0;
}

/* 本地路线表：上位机不发时自己灌（台架/自立跑）。 r1..r5 对应任务 1..5，等价于收到 @任务+格号(角)+格号(角)% */
/* 右出发（follow_side = 1）：与 Follow_Set_L 只差 r1 —— 这里 25→15 各 +90°，左出发是 5→15 转 -90° */
void Follow_Set_R(void)
{
    static const int16_t r1[] = { 25,  90,  15,  90 };
    static const int16_t r2[] = { 13, -90,  23,  90 };
    static const int16_t r3[] = { 23,  90,   3,  90 };
    static const int16_t r4[] = {  1, -90,  11,   0 };   /* case 8 跑的第二阶段路线 */
    static const int16_t r5[] = { 21, -90,  23,   0 };   /* case 9（第二遍转场）*/

    set_one(1, r1, 2);
    set_one(2, r2, 2);
    set_one(3, r3, 2);
    set_one(4, r4, 2);
    set_one(5, r5, 2);
}

/* 左出发（follow_side = 0）那套路线：**只有任务 1 的第一对不同**（5 → 15 转 −90），其余与右出发一致 */
void Follow_Set_L(void)
{
    static const int16_t r1[] = {  5,   0,  15, -90 };
    static const int16_t r2[] = { 13, -90,  23,  90 };
    static const int16_t r3[] = { 23,  90,   3,  90 };
    static const int16_t r4[] = {  1, -90,  11,   0 };
    static const int16_t r5[] = { 21, -90,  23,   0 };

    set_one(1, r1, 2);
    set_one(2, r2, 2);
    set_one(3, r3, 2);
    set_one(4, r4, 2);
    set_one(5, r5, 2);
}

void Follow_Reset(void)
{
    int i;
    for (i = 0; i <= FOLLOW_TASK_MAX; i++) {
        path_n[i] = 0;
    }
    s_in = 0;

    for (i = 0; i <= FOLLOW_TASK_MAX; i++) {   /* 走位指针 + 下发标志 + 补转标志也归零 */
        s_walk[i] = 0;
        s_sent[i] = 0;
        s_rot2[i] = 0;
    }
    s_pre15 = 0;

    s_pre15 = 0;
    Follow_SideInit();
}


uint8_t Follow_HasTask(uint8_t task_id)
{
    if (task_id < 1 || task_id > FOLLOW_TASK_MAX) return 0;
    return (path_n[task_id] != 0) ? 1 : 0;
}


/* ========================= 走一条路线 ========================= Follow_Walk(N) ↔ 任务号 N 的路线：按 (path[N][i],
 * ang[N][i]) 逐对下发 GO_To， 到一对自动进下一对（非阻塞，case 里每 20ms
  * 调一次即可）。 例：@1+1(90)+5(90)% ⇒ path[1][1]=1, ang[1][1]=90, path[1][2]=5, ang[1][2]=90 调 Follow_Walk(1) 就是：先
  * GO_To(x[1],y[1],+90) → 到位后 GO_To(x[5],y[5],+90) → 走完。 ⚠ GO_To 的相对角是**累加**的：所以每一对只在下发的那一拍调一次（s_sent 保证），
  * 之后再调只轮询 GO_Arrived()，绝不重复下发，否则 90→180→270 一路叠上去。 ⚠ 走完一个任务后指针停在末尾，之后再调什么都不做；要重跑调 Follow_WalkReset()。 */
void Follow_Walk(uint8_t task)
{
    int i, c;
    float tx, ty;

    if (task < 1 || task > FOLLOW_TASK_MAX) return;   /* 任务号非法：不动 */

    side_apply();                       /* follow_side 变过就重拆 x[]/y[] */

    if (path_n[task] == 0) return;      /* 这条路线还没收到：不动 */

    if (s_walk[task] == 0) s_walk[task] = 1;   /* 第一次进来从第 1 对开始 */
    i = s_walk[task];
    if (i > path_n[task]) return;              /* 已经走完 */

    /* 左基准（实测版）：任务1 的第一对之前，**不管第一格是不是 5**，都先限速直给 GO_To 到
     *   (TF_PRE_LEFT_X, TF_PRE_LEFT_Y_CMD)；到位后把任务系 Y 强行纠正成 TF_PRE_LEFT_Y_FIX，
     *   再照常走路线第一段（不省略路径，第一格是 5 就接着走去 5 号格 + 转）。*/
    if ((follow_side == 0) && (task == 1) && (i == 1) && (s_pre15 != 2)) {
        if (s_pre15 == 0) {
            GO_SetVmax(TF_PRE_LEFT_VMAX);
            GO_To(TF_PRE_LEFT_X, TF_PRE_LEFT_Y_CMD, 0.0f);
            s_pre15 = 1;
            return;
        }
        if (!GO_Arrived()) return;                 /* 还没到：接着等 */
        GO_SetVmax(0.0f);                          /* 速度上限恢复默认档 */
        GO_SetTaskPos(2, TF_PRE_LEFT_Y_FIX);       /* 任务系 Y = 实测值（另一轴不动）*/
        GO_Stop();                                 /* 位姿改了必须重设目标，否则会往回补那 2cm */
        s_pre15 = 2;
        return;
    }

    /* 任务1 前置（**这一段现在只对右基准生效**；左基准在上面 grab0/TF_PRE_LEFT 那段就 return 了）：
     *   **一律**先限速 20cm/s 走到前置落点（见 TF_PRE_GO_*_R 那几个宏）：
     *   第一格是扫码位 25(右)/5(左) ⇒ 斜冲到 PASS_X 后退到 ROT_X，接着原地旋转；
     *   否则 ⇒ 只斜走到 PASS_X 就停，然后照常走路线第一段。*/
    if (task == 1 && i == 1 && s_pre15 != 2) {
        float px = follow_side ? TF_PRE_GO_PASS_X_R : TF_PRE_GO_PASS_X_L;
        float py = follow_side ? TF_PRE_GO_Y_R      : TF_PRE_GO_Y_L;
        float pv = -1.0f;                            /* 负数 = 不斜冲不收尾，直接去 (px, py) */

        /* 第一格就是扫码位(25/5)：斜冲到 PASS_X 后只退到 ROT_X，接着原地旋转 */
        if (path[1][1] == (follow_side ? 25 : 5)) {
            px = follow_side ? TF_PRE_GO_ROT_X_R    : TF_PRE_GO_ROT_X_L;
            pv = follow_side ? TF_PRE_GO_ROT_OVER_R : TF_PRE_GO_ROT_OVER_L;
        }
        if (s_pre15 == 0) {
            GO_SetVmax(20.0f);                       /* 这一段限速 20cm/s：车重，快了就打滑 */
            Go_Test_DiagThenStraightReset();
            s_pre15 = 1;
        }
        if (s_pre15 == 1) {
            /* 斜冲 + 直行收尾走到 (px, py)：45° 斜走各轮都在滚（不打滑），最后一段纯直行收尾（准）。*/
            /* ⚠ axis 恒为 1：任务系 +X **永远**是车头方向（yaw0 就是按停车车头锁的），左右基准一样；
             *    follow_side 只决定"跟踪时横向修正用哪根轴"，跟车头方向无关。当初按它选轴，左基准会把斜冲冲成横移。*/
            if (Go_Test_DiagThenStraight(1, px, py, pv)) return;
            GO_SetVmax(0.0f);                        /* 速度上限恢复默认档 */
            s_pre15 = 2;
            return;
        }
    }

    /* ---------- 已经下发过：只等到位 ---------- */
    if (s_sent[task]) {
        if (!GO_Arrived()) return;
        s_sent[task] = 0;

        if (s_rot2[task] == 1) {           /* 位置段到了：这一对还没完，转成"补转" */
            s_rot2[task] = 2;
            return;
        }
        s_walk[task]++;                    /* 这一对彻底走完 → 下一对 */
        return;
    }

    /* ---------- 该下发 ---------- */
    c = path[task][i];
    if (!cell_ok(c)) {                     /* 非法格号：跳过这一对 */
        s_walk[task]++;
        s_rot2[task] = 0;
        return;
    }

    /* 任务1 的第一对：只有"是右基准、且第一格就是扫码位 25"时才把目标点改成前置落点（TF_PRE_GO_ROT_X_R/_L）⇒ 这一对只剩原地自转；其它情况仍用表里的格号 x[c]/y[c]，路线第一段照常走。*/
    tx = x[c]; ty = y[c];
    if (task == 1 && i == 1 && follow_side != 0 && path[1][1] == (follow_side ? 25 : 5)) {
        tx = follow_side ? TF_PRE_GO_ROT_X_R : TF_PRE_GO_ROT_X_L;
        ty = follow_side ? TF_PRE_GO_Y_R     : TF_PRE_GO_Y_L;
    }

    /* 需要"先走位置、再原地转"的两种情况（其它情况仍是位置+角度一起下发）： ① 任务 1 的第 1 个动作：起步从原点走出，短腿 + 大角度会走成"边冲边甩"； ② 目标格是 **13 号**且要转：该点位置敏感，先停稳再转才不会带偏。
     * 拆法：GO_To(x,y,0) → 到位 → GO_To(x,y,ang)（同一点，只加角度）。 */
    if (ang[task][i] != 0 && s_rot2[task] == 0 &&
        ((task == 1 && i == 1) || c == 13)) {
        GO_To(tx, ty, 0.0f);           /* ① 只走位置，不转 */
        s_rot2[task] = 1;
        s_sent[task] = 1;
        return;
    }

    if (s_rot2[task] == 2) {               /* ② 原地补转（同一格、同一点，只加角度） */
        GO_To(tx, ty, (float)ang[task][i]);
        s_rot2[task] = 0;
        s_sent[task] = 1;
        return;
    }

    GO_To(tx, ty, (float)ang[task][i]);   /* 其它情况：位置 + 角度一起下发 */
    s_sent[task] = 1;
}

/* 1 = 这条路线已经走完（还没收到路线时返回 0，别让 case 提前跳走） */
uint8_t Follow_WalkDone(uint8_t task_id)
{
    if (task_id < 1 || task_id > FOLLOW_TASK_MAX) return 0;
    if (path_n[task_id] == 0) return 0;
    return (s_walk[task_id] > path_n[task_id]) ? 1 : 0;
}

uint8_t Follow_WalkIdx(uint8_t task)   /* 诊断：当前走到第几对（1..path_n，>path_n = 走完）*/
{
    if (task < 1 || task > FOLLOW_TASK_MAX) return 0;
    return s_walk[task];
}

void Follow_WalkReset(void)
{
    int i;

    for (i = 0; i <= FOLLOW_TASK_MAX; i++) {
        s_walk[i] = 0;
        s_sent[i] = 0;
        s_rot2[i] = 0;
    }
    s_pre15 = 0;
}
