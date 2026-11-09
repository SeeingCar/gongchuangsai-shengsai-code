/* See.c —— 摄像头/扫码串口接收与解析（协议见 See.h）。 ⚠ 用帧队列而非单缓冲：单缓冲会被下一帧覆盖 ⇒ 丢帧；队列满丢新帧、不覆盖旧帧。 */
#include "See.h"
#include "Follow.h"        /* 上位机路线帧入口 Follow_RxByte */
#include "usart.h"          /* huart3 = USART3（PD8/PD9）；USART6 归陀螺仪 */
#include "main.h"
#include <string.h>
#include <stdlib.h>

volatile int cam_color   = 0;
volatile int cam_cx      = 0;
volatile int cam_cy      = 0;
volatile int cam_has_obj = 0;
volatile int cam_target_id = 1;     /* 追踪目标 id（1~9）：由 Grab 抓起一块后推进到下一块物料颜色*/
volatile float cam_px_to_cm = 0.0326f;/* 像素→厘米比例尺：cx 30.7px = 1cm（640×480 实测 32/31/30/30）*/
volatile float cam_px_to_cm_y = 0.0303f;/* cy 方向比例尺：cy 33.0px = 1cm（640×480 实测 26/39/32→均值），M2 前馈用；
                                         *   底盘 cx 和 M2 cy 不是同一个机械轴，像素/厘米可能不一致 */
/* 爪子中心在画面里的 cx 像素位置：底盘 case2 的对准目标 + 手臂 A1 判停的横向条件都用它。 默认 CX_TARGET_PX。标定： 物料放正中、手动把爪子中线对准它，读打印第 3 列就是真值。 */
volatile float cam_grip_cx_px = CX_TARGET_PX;
/* cx 极性开关：0 = 不翻（+cx = 车头方向），1 = 翻。只对收到的 cx 取负；目标点 cam_grip_cx_px 是独立常量，不跟着翻 */
/* ⚠ 默认值 = 1：**实测跑飞证明原符号是正反馈**（2026 换 640×480 相机后）。 推导：车往 +X 走 1cm 时 cx 变化 g px；g<0 时 step=−K·e 变成正反馈， 每帧误差 ×1.41，1 秒放大 160 倍 —— 与"持续加速走远"的现象一致。
 * 若哪天相机又换装法导致方向反过来，把这里改回 0 即可。 */
volatile int cam_cx_invert = 1;
/* cy 极性开关：与 cx 同理，新相机看起来是**装了 180°**（两轴符号同时反）。 实测证据：cy 恒定在 +175（= 5.3cm）不收敛 ⇒ 手臂被正反馈顶到限位动不了。 0 = 不翻 / 1 = 翻。
 * 翻转同样只改源，前馈公式 v_meas = v_arm − rate×scale 自动保持一致。 */
volatile int cam_cy_invert = 1;

volatile int cam_ring_no = 0;
volatile int cam_ring_cx = 0;
volatile int cam_ring_cy = 0;
/* 8 号环专用槽：现场相机会 7/8 交替发，上面的 cam_ring_* 是"最后一帧任意环"会被冲掉；
 * 底盘 case 5 只认 8 号，所以单独留一份只被 8 号帧更新的坐标与时刻（7/9 号帧不碰它）。 */
static volatile int      s_r8_cx = 0;
static volatile int      s_r8_cy = 0;
static volatile uint32_t s_r8_ms = 0;
static uint8_t see_ring8_fresh(void);   /* 8 号槽是否新鲜：定义在下面 align 那一段，去重门要用到 */
volatile int cam_has_ring = 0;
volatile uint32_t cam_frame_seq = 0;   /* 物料帧序号 +1，判新帧用 */
volatile uint32_t cam_last_ms   = 0;   /* 最后一次成功解析物料帧的时刻（判数据新旧）*/

/* 任意颜色物料帧：第一阶段“先追任意物料定基准”用；与按目标色过滤的 cam_color/cam_frame_seq 分开。 */
volatile int      cam_any_color = 0;
volatile uint32_t cam_any_seq   = 0;

/* 最近 24 帧的颜色历史（所有颜色都记）：判"张开后的 6 帧里有没有目标色"用。 24 是为了剔除掉某些颜色后仍能凑满 6 帧（见 See_RecentColorsSince）。 */
#define SEE_HIST_N  24
static volatile uint8_t  s_hist_col[SEE_HIST_N];
static volatile uint32_t s_hist_ms[SEE_HIST_N];   /* 每帧的到达时刻（ms），"只要抬升之后的帧"用 */
static volatile uint32_t s_hist_cnt = 0;      /* 累计解析出的物料帧数 */

void see_hist_push(uint8_t col)
{
    uint32_t i = s_hist_cnt % SEE_HIST_N;

    s_hist_col[i] = col;
    s_hist_ms[i]  = HAL_GetTick();
    s_hist_cnt++;
}



volatile uint32_t cam_ring_seq  = 0;   /* 圆环帧序号 +1 */

volatile int qr_c0[3]        = {0, 0, 0};
volatile int qr_c1[3]        = {0, 0, 0};
volatile int qr_c2[3]        = {0, 0, 0};   /* 第二遍：3 个抓取物料颜色（二维码 C 段）*/
/* 对准源切换（用户 2026-10-31）：case 5/6 认圆环帧（8 号环）， case 11 认"指定颜色的**物料帧**"（= 上一次放进 8 号环的那块料）。 */
volatile int  cam_align_sel = 0;   /* 0 = 圆环帧；1 = 指定颜色物料帧 */
volatile int      cam_sel_color = 0;   /* 目标物料颜色 1..6 */
volatile int      cam_sel_cx    = 0;
volatile int      cam_sel_cy    = 0;
volatile uint32_t cam_sel_seq   = 0;
volatile uint32_t cam_sel_ms    = 0;
volatile uint32_t cam_ring_ms   = 0;   /* 最近一次圆环帧时刻（sel 模式的兜底判据）*/

volatile int qr_c0a[3]       = {0, 0, 0};   /* 第 1 轮物料的**颜色**快照（A 段）— 第二轮 case11/12 要用它查"哪个色属于哪个环"*/
volatile int qr_c1a[3]       = {0, 0, 0};   /* 第 1 轮物料的**环号**快照（B 段）*/
volatile int qr_c3[3]        = {0, 0, 0};   /* 第二遍：3 个环号（二维码 D 段，1/2/3 已 +6）*/
volatile int qr_has_data     = 0;
volatile uint32_t qr_cam_n   = 0;     /* 相机扫到几帧合法码（诊断）*/
volatile int      qr_cam[6]  = {0};   /* 相机扫到的 6 位（诊断）*/

/* 每一路（物料 1~6、圆环 7~9）最近一次的 cx/cy（只给遥测看）。 下标：物料直接用颜色号 1..6；圆环用 环号-7（7→0, 8→1, 9→2）。0 = 还没见过。 */

volatile uint8_t see_rx_byte = 0;
/* 上位机路线握手（用户 2026-11-01）：
 *   ① See_SendSide()：把本机出发边发给上位机，格式 '#2*' = 左（follow_side_key=0）、'#1*' = 右（=1）；
 *   ② 上位机随后发路线帧 '@任务+格号(角)+…%'，字节由 See_PreFlowByte() 送进 Follow_RxByte 入表；
 *   ③ See_RouteReceived()：**8 段（任务 1..8）全部进表**才返回 1。*/
static   int s_preflow     = 1;       /* 1 = 还在握手（相机/扫码字节不解析）*/

/* 把出发边发给上位机：side=0 发 '#2*'（左），side=1 发 '#1*'（右）*/
void See_SendSide(uint8_t side)
{
    uint8_t buf[3];

    buf[0] = (uint8_t)'#';
    buf[1] = (side != 0) ? (uint8_t)'1' : (uint8_t)'2';   /* 0 → '#2*'（左）；1 → '#1*'（右）*/
    buf[2] = (uint8_t)'*';

    s_preflow     = 1;
    (void)HAL_UART_Transmit(&huart3, buf, 3, 200);
}

/* 握手阶段：字节全部送进路线解析（相机/扫码不解析）。返回 1 = 已消费 */
uint8_t See_PreFlowByte(uint8_t b)
{
    if (!s_preflow) return 0;
    Follow_RxByte(b);
    return 1;
}

/* 1 = 8 段路线（任务 1..8）全都收到了；少任何一段都返回 0 */
uint8_t See_RouteReceived(void)
{
    int i;

    for (i = 1; i <= 8; i++) {
        if (path_n[i] == 0) return 0;
    }
    return 1;
}

void See_PreFlowDone(void) { s_preflow = 0; }

/* ISR 拼包 + 帧队列 */
static uint8_t  see_isr_buf[SEE_RX_BUF_SIZE];     /* ISR 拼包用 */
static uint16_t see_isr_idx = 0;
static uint8_t  see_q[SEE_FRAME_MAX][SEE_RX_BUF_SIZE];   /* 帧队列 */
static uint8_t  see_q_len[SEE_FRAME_MAX];
static uint16_t see_q_head = 0;                   /* ISR 写 */
static uint16_t see_q_tail = 0;                   /* 任务读 */
static osSemaphoreId_t semSeeData = NULL;         /* 队列非空 */
static uint8_t  see_cur[SEE_RX_BUF_SIZE];         /* 任务当前在解析的那一帧 */
static uint16_t see_cur_len = 0;

void See_Init(void)
{
    semSeeData  = osSemaphoreNew(1, 0, NULL);
    see_q_head  = 0;
    see_q_tail  = 0;
    see_isr_idx = 0;
    see_cur_len = 0;
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&see_rx_byte, 1);
}

/* 逐字节拼包（中断里调）：见到任一包头就丢弃半帧重开，不会永久错位 */
void See_RxISR(uint8_t byte)
{
    uint16_t next;

    if (byte == '$' || byte == '#') {             /* 包头：重开 */
#if (SEE_QR_PARSE_EN == 0)
        /* 扫码停用时 '#' 帧**在中断层直接丢**，不占队列 —— 否则相机一直发 #...& 会 把队列（SEE_FRAME_MAX）占满，满的时候新帧被丢弃，颜色帧就会被挤掉。 */
        if (byte == '#') { see_isr_idx = 0; return; }
#endif
        see_isr_buf[0] = byte;
        see_isr_idx    = 1;
        return;
    }

    if (see_isr_idx == 0) return;                 /* 还没包头，字符全丢 */

    if (see_isr_idx >= SEE_RX_BUF_SIZE) {         /* 防越界：当坏帧重开 */
        see_isr_idx = 0;
        return;
    }
    see_isr_buf[see_isr_idx++] = byte;

    if (!((byte == '%' && see_isr_buf[0] == '$') ||
          (byte == '&' && see_isr_buf[0] == '#'))) {
        return;
    }

    next = (uint16_t)((see_q_head + 1) % SEE_FRAME_MAX);   /* 满则丢新帧，不覆盖旧帧 */
    if (next != see_q_tail) {
        memcpy(see_q[see_q_head], see_isr_buf, see_isr_idx);
        see_q_len[see_q_head] = (uint8_t)see_isr_idx;
        see_q_head = next;
        if (semSeeData) osSemaphoreRelease(semSeeData);
    }
    see_isr_idx = 0;
}

uint16_t See_WaitFrame(uint32_t timeout)
{
    if (osSemaphoreAcquire(semSeeData, timeout) != osOK) return 0;

    see_cur_len = see_q_len[see_q_tail];
    memcpy(see_cur, see_q[see_q_tail], see_cur_len);
    see_q_tail  = (uint16_t)((see_q_tail + 1) % SEE_FRAME_MAX);
    return see_cur_len;
}

/* 抠出包头与包尾之间的内容成 C 字符串；成功返回长度，失败 -1 */
static int see_strip(const uint8_t *pkt, uint16_t len, char *out, int out_sz, char tail)
{
    int end = -1;
    int i;

    if (len < 3) return -1;
    for (i = 1; i < (int)len; i++) {
        if (pkt[i] == (uint8_t)tail) { end = i; break; }
    }
    if (end < 0) return -1;
    if (end - 1 > out_sz - 1) return -1;

    memcpy(out, &pkt[1], (size_t)(end - 1));
    out[end - 1] = '\0';
    return end - 1;
}

static int see_split_atoi(char *s, char sep, int *out, int n)
{
    char  sepbuf[2];
    char *tok;
    int   cnt = 0;

    sepbuf[0] = sep; sepbuf[1] = '\0';   /* strtok 要"以 0 结尾的分隔符串"；直接传 &sep 会顺着栈扫 0（UB） */
    tok = strtok(s, sepbuf);
    while (tok != NULL && cnt < n) {
        out[cnt++] = atoi(tok);
        tok = strtok(NULL, sepbuf);
    }
    return cnt;
}

/* 三位十进制数按位拆开：d[0]=百位、d[1]=十位、d[2]=个位 */
static void see_digits(int v, volatile int *d)
{
    if (v < 0) v = -v;
    d[0] = (v / 100) % 10;
    d[1] = (v / 10) % 10;
    d[2] =  v % 10;
}

/* 解析当前帧（任务里调）：按包头分流，填 cam_* / qr_* */
void See_ParseFrame(void)
{
    char str[SEE_RX_BUF_SIZE];
    int  vals[4] = {0, 0, 0, 0};
    int  n;

    if (see_cur_len == 0) return;

    if (see_cur[0] == '$') {                      /* 相机帧 $颜色!X!Y% */
        if (see_strip(see_cur, see_cur_len, str, (int)sizeof(str), '%') < 0) { return; }
        n = see_split_atoi(str, '!', vals, 3);
        if (n < 3) { return; }

        if (vals[0] >= 1 && vals[0] <= 9) {
            if (vals[0] <= 6) {
                int cx = vals[1];
                int cy = vals[2];

                if (cam_cx_invert) cx = -cx;   /* 极性开关：打印和控制用同一个符号约定 */
                if (cam_cy_invert) cy = -cy;

                {
                    int col = vals[0];                        /* 6 = 浅蓝，**不再**归一成 3 */

                    /* 目标色坐标只由目标色刷新：画面里另有同色块时若也覆盖，底盘/手臂会追着“上一帧那块料”跑（实测 cx 60ms 跳 210px）。 */
                    see_hist_push((uint8_t)col);           /* 颜色历史（判抓取成功用）*/

                    /* 任意颜色都更新一份“最近物料帧”，给第一阶段找任意物料用。 */
                    cam_any_color = col;
                    cam_any_seq++;

                    /* 选定颜色通道（case 11 对准用）。
                     * ⚠ 现场实测：车上转盘里还有一块同色料 ⇒ 同色帧在两处来回跳（cx -100 ↔ +133），
                     *   底盘就在两块之间来回纠。这里加"同色去重门"：只认与 8 号环相差 ≤RING_SEL_GATE_PX 的那块。*/
                    if (See_SameColor(col, cam_sel_color)) {
                        int take = 1;

                        if (cam_align_sel && see_ring8_fresh()) {
                            int dx = cx - s_r8_cx;
                            int dy = cy - s_r8_cy;

                            if (dx < 0) dx = -dx;
                            if (dy < 0) dy = -dy;
                            if ((dx > RING_SEL_GATE_PX) || (dy > RING_SEL_GATE_PX)) take = 0;
                        }
                        if (take) {
                            cam_sel_cx  = cx;
                            cam_sel_cy  = cy;
                            cam_sel_seq++;
                            cam_sel_ms  = HAL_GetTick();
                        }
                    }

                    if (See_SameColor(col, cam_target_id)) {
                        cam_color   = col;
                        cam_cx      = cx;
                        cam_cy      = cy;
                        cam_has_obj = 1;
                        cam_frame_seq++;
                        cam_last_ms = HAL_GetTick();
                    }
                }
            } else {
                cam_ring_no  = vals[0];
                cam_ring_cx  = cam_cx_invert ? -vals[1] : vals[1];
                cam_ring_cy  = cam_cy_invert ? -vals[2] : vals[2];
                cam_has_ring = 1;
                cam_ring_ms  = HAL_GetTick();
                cam_ring_seq++;

                /* 只有 8 号环刷新专用槽：7/9 号帧既不覆盖、也不延长它的寿命 */
                if (cam_ring_no == 8) {
                    s_r8_cx = cam_ring_cx;
                    s_r8_cy = cam_ring_cy;
                    s_r8_ms = cam_ring_ms;
                }

                if (cam_ring_no >= 7 && cam_ring_no <= 9) {
                }
            }
        }
    } else if (see_cur[0] == '#') {               /* 扫码帧 #AAA+BBB+CCC+DDD& */
#if (SEE_QR_PARSE_EN == 0)
        /* SEE_QR_PARSE_EN = 0 时：串口的码一律不用，二维码只由 Set() 手动给 */
        return;
#else
        /* 噪声凑出来的短帧（如 "#&"）直接丢：不算成帧、也不算 bad */
        if (see_cur_len < 6) { return; }

        if (see_strip(see_cur, see_cur_len, str, (int)sizeof(str), '&') < 0) { return; }
        n = see_split_atoi(str, '+', vals, 4);
        if (n < 4) { return; }

        /* 先把**相机扫到的**记进诊断量（不受 Set() 和 qr_has_data 影响）， 串口每 1 秒那行 Z,… 就能看出到底扫上没有。 */
        see_digits(vals[0], qr_cam);
        see_digits(vals[1], &qr_cam[3]);
        if (qr_cam[3] >= 1 && qr_cam[3] <= 3) qr_cam[3] += 6;   /* 1/2/3 → 7/8/9 */
        if (qr_cam[4] >= 1 && qr_cam[4] <= 3) qr_cam[4] += 6;
        if (qr_cam[5] >= 1 && qr_cam[5] <= 3) qr_cam[5] += 6;
        /* 6 = 浅蓝，保留原值，不再归一成 3 */
        qr_cam_n++;

        /* 第 3、4 段 = **第二遍**的 3 色 / 3 环（C/D 段）。每帧都更新， 不受 qr_has_data 影响 —— 这样第一遍可以硬编码、第二遍用真扫码。 */
        {
            int t2[3];

            see_digits(vals[2], t2);
            qr_c2[0] = t2[0]; qr_c2[1] = t2[1]; qr_c2[2] = t2[2];

            see_digits(vals[3], t2);
            if (t2[0] >= 1 && t2[0] <= 3) t2[0] += 6;      /* 1/2/3 → 7/8/9 */
            if (t2[1] >= 1 && t2[1] <= 3) t2[1] += 6;
            if (t2[2] >= 1 && t2[2] <= 3) t2[2] += 6;
            qr_c3[0] = t2[0]; qr_c3[1] = t2[1]; qr_c3[2] = t2[2];
        }

        /* 工作用的三组数组只认**第一帧**（Set() 已经灌过就不覆盖）*/
        if (qr_has_data) return;

        qr_c0[0] = qr_cam[0];  qr_c0[1] = qr_cam[1];  qr_c0[2] = qr_cam[2];
        See_UpdateBlueMerge();                 /* 3/6 是否等价由这三块料决定 */
        qr_c1[0] = qr_cam[3];  qr_c1[1] = qr_cam[4];  qr_c1[2] = qr_cam[5];
        qr_has_data     = 1;
#endif
    }
}

/* 有新帧则更新 *last_seq 并返回 1；两状态机各持一份，互不干扰。⚠ 数帧必须用它，否则连续帧判据/判停会失守 */
uint8_t See_IsNewFrame(uint32_t *last_seq)
{
    uint32_t now = cam_frame_seq;

    if (*last_seq == now) return 0;
    *last_seq = now;
    return 1;
}

/* 对准源访问：cx/cy 的语义与圆环帧一致（同一套 RING_CX/CY_TGT 目标点）。*/
void See_AlignSetSel(uint8_t sel, uint8_t color)
{
    cam_align_sel = sel ? 1 : 0;
    if (color >= 1 && color <= 6) cam_sel_color = (int)color;
}

/* sel 模式：颜色帧新鲜就用颜色帧；否则退回 8 号环帧（避免颜色通道丢了就一直对不上）*/
static uint8_t see_sel_fresh(void)  { return ((HAL_GetTick() - cam_sel_ms)  < 150) ? 1 : 0; }
static uint8_t see_ring8_fresh(void){ return ((HAL_GetTick() - s_r8_ms) < 150 && s_r8_ms != 0) ? 1 : 0; }

int See_AlignCx(void) { if (!cam_align_sel) return s_r8_cx; return see_sel_fresh() ? cam_sel_cx : s_r8_cx; }
int See_AlignCy(void) { if (!cam_align_sel) return s_r8_cy; return see_sel_fresh() ? cam_sel_cy : s_r8_cy; }

uint8_t See_AlignNew(uint32_t *last_seq)
{
    uint32_t s;

    if (!cam_align_sel) return See_IsNewRingFrame(last_seq);
    s = see_sel_fresh() ? cam_sel_seq : cam_ring_seq;   /* 跟当前实际使用的源保持一致 */
    if (*last_seq == s) return 0;
    *last_seq = s;
    return 1;
}

uint8_t  See_AlignSelGet(void)   { return cam_align_sel; }
uint8_t  See_AlignColorGet(void) { return (uint8_t)cam_sel_color; }
uint32_t See_AlignSeqGet(void)   { return cam_sel_seq; }

uint8_t See_AlignSeen(void)
{
    if (cam_align_sel) return (see_sel_fresh() || see_ring8_fresh()) ? 1 : 0;
    return see_ring8_fresh();          /* 只认 8 号槽：7/9 号帧交替不会让它忽真忽假 */
}

uint8_t See_IsNewRingFrame(uint32_t *last_seq)
{
    uint32_t now = cam_ring_seq;

    if (*last_seq == now) return 0;
    *last_seq = now;
    return 1;
}

/* 取 **since_ms 之后**（含）的最近 n 帧颜色，跳过颜色 skip（1..6；0=不跳）。 用途：抓取检查的 6 帧只允许是"抬升 UD 完成之后"拍的，绝不能挖到抬升之前的旧帧。 不够 n 帧时前面补 0（宁可少，不拿旧帧凑数）。 */
void See_RecentColorsSince(uint8_t *out, uint8_t n, uint8_t skip, uint8_t skip2, uint32_t since_ms)
{
    uint32_t cnt = s_hist_cnt;
    int  i, m = 0, scan;

    if (out == 0) return;
    for (i = 0; i < n; i++) out[i] = 0;
    if (n > SEE_HIST_N) n = SEE_HIST_N;

    if (skip  < 1 || skip  > 6) skip  = 0;
    if (skip2 < 1 || skip2 > 6) skip2 = 0;

    scan = (cnt > (uint32_t)SEE_HIST_N) ? (uint8_t)SEE_HIST_N : (uint8_t)cnt;
    for (i = 1; i <= scan && m < n; i++) {
        uint32_t k = (cnt - i) % SEE_HIST_N;

        if ((int32_t)(s_hist_ms[k] - since_ms) < 0) break;   /* 比抬升时刻还早：后面的更旧，直接停 */
        if (skip  != 0 && See_SameColor(s_hist_col[k], skip))  continue;   /* 第 1 块的色：不算 */
        if (skip2 != 0 && See_SameColor(s_hist_col[k], skip2)) continue;   /* 第 2 块的色：不算 */
        out[n - 1 - m] = s_hist_col[k];
        m++;
    }
}

/* 这帧物料是不是目标色；target 非法（0 = qr 未收到 / 超范围）一律 0，宁可继续扫 */
/* 1 = 3 和 6 视为同一颜色（浅蓝/深蓝分不稳）。由扫码结果决定，见 See_UpdateBlueMerge()。 */
static int s_blue_merge = 0;

uint8_t See_SameColor(int a, int b)
{
    if (a == b) return 1;
    if (s_blue_merge) {
        if ((a == 3 && b == 6) || (a == 6 && b == 3)) return 1;
    }
    return 0;
}

void See_UpdateBlueMerge(void)
{
    int i, has3 = 0, has6 = 0;

    for (i = 0; i < 3; i++) {
        if (qr_c0[i] == 3) has3 = 1;
        if (qr_c0[i] == 6) has6 = 1;
    }
    s_blue_merge = 0;
    if (g_blue_merge_key != 0 && !(has3 && has6)) {
        s_blue_merge = 1;
    }
}

uint8_t See_IsTarget(int target)
{
    if (target < 1 || target > 6) return 0;
    if (!cam_has_obj)             return 0;
    if (!See_DataFresh(SEE_FRESH_MS)) return 0;   /* 过期数据一律不算"看到" */
    return See_SameColor(cam_color, target);
}


/* 相机数据是否新鲜：最后成功帧距今 < max_age_ms */
bool See_DataFresh(uint32_t max_age_ms)
{
    if (cam_frame_seq == 0) return false;          /* 一帧都没来过 */
    return (HAL_GetTick() - cam_last_ms) < max_age_ms;
}
