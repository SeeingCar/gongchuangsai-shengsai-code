/* zdt_motor.c —— ZDT X42S 闭环步进：三总线 TTL 收发、位置/速度/回零命令、位置反馈解析 */
#include "zdt_motor.h"
#include "usart.h"
#include "hwt601.h"
#include "See.h"
#include "Follow.h"         /* 路径规划帧：与相机帧共用 USART3 */
#include "6020.h"           /* USART1 调参命令 */
#include "FreeRTOS.h"       /* vTaskSuspendAll / xTaskResumeAll：总线发送互斥 */
#include "task.h"
#include <string.h>
#include <math.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* ⚠ 轴↔总线按实测接线：A=USART2{1,4}、B=UART4{3,6}、C=UART5{2,7}，改线必须同步改这里。 ⚠ 6 号是底盘左前轮；若把 6 也放进 C，bus_of 按 A→B→C 先命中 B，手臂 UD 会打到车轮。 */
static const uint8_t g_busA_axes[] = { 1, 4 };
static const uint8_t g_busB_axes[] = { 3, 6 };
static const uint8_t g_busC_axes[] = { 2, 7 };

/* 反馈：ISR 写 zdt_raw + zdt_last_update；任务侧 ZDT_UpdateAngle() 换算 zdt_angle（中断内不做浮点） */
volatile int32_t  zdt_raw[ZDT_MOTOR_MAX]         = {0};
volatile uint32_t zdt_last_update[ZDT_MOTOR_MAX]  = {0};
volatile float    zdt_angle[ZDT_MOTOR_MAX]        = {0};
/* 1 = 该轴已把**真实帧**换算成 zdt_angle（与年龄无关，上电后只会 0→1）。 差分测速用它建基准：在它为 0 之前 zdt_angle 恒为 0，拿它对 0 求差会造出"0→真值"的巨跳。 */
volatile uint8_t  zdt_ready[ZDT_MOTOR_MAX]        = {0};

#define ZDT_FRAME_TIMEOUT_MS   5      /* 收帧超时(ms)：正常帧字节间隔 <1ms */

typedef struct {
    UART_HandleTypeDef *huart;
    const uint8_t     *axes;
    int            axis_cnt;

    int  st;                      /* 0=idle 1=func 2=sign 3=data 4=end */
    int  cur_addr;
    int  sign;
    uint8_t  pos_buf[4];
    int  pos_idx;
    uint32_t last_rx_tick;            /* 最近一次收到字节的时刻(ms) */
} ZdtBusCtx_t;

static ZdtBusCtx_t g_bus[ZDT_BUS_MAX];

/* 逐字节拼应答帧 {Addr,36,SIGN,P3..P0,6B}，地址 1..7 才收（7 = 手臂上下轴）*/
static void rx_byte_process(ZdtBusCtx_t *b, uint8_t byte)
{
    b->last_rx_tick = HAL_GetTick();

    switch (b->st) {
    case 0:
        if (byte >= 0x01 && byte < ZDT_MOTOR_MAX) {
            b->cur_addr = byte;
            b->st = 1;
        }
        break;
    case 1:
        if (byte == 0x36)      b->st = 2;
        else if (byte == 0x9A) b->st = 5;   /* 回零命令应答：Addr,9A,9F,6B */
        else                   b->st = 0;
        break;
    case 2:
        b->sign = byte;
        b->pos_idx = 0;
        b->st = 3;
        break;
    case 3:
        if (b->pos_idx < 4) {
            b->pos_buf[b->pos_idx++] = byte;
        }
        if (b->pos_idx >= 4) {
            b->st = 4;
        }
        break;
    case 4:
        if (byte == ZDT_TAIL && b->cur_addr >= 1 && b->cur_addr < ZDT_MOTOR_MAX) {
            uint32_t raw = ((uint32_t)b->pos_buf[0] << 24) |
                           ((uint32_t)b->pos_buf[1] << 16) |
                           ((uint32_t)b->pos_buf[2] << 8)  |
                           ((uint32_t)b->pos_buf[3]);
            int32_t sraw = (int32_t)raw;
            if (b->sign == 0x01) {
                sraw = -sraw;
            }
            zdt_raw[b->cur_addr]        = sraw;
            zdt_last_update[b->cur_addr] = HAL_GetTick();
        }
        b->st = 0;
        b->pos_idx = 0;
        break;
    case 5:
        b->st = 0;                     /* 0x9A 回零应答：只收帧复位，不判完成（固定等 5s） */
        break;
    default:
        b->st = 0;
        b->pos_idx = 0;
        break;
    }
}

/* HAL 单字节接收落点：必须是文件作用域 static，不能放栈上 */
static volatile uint8_t zdt_rx_byte_A = 0;
static volatile uint8_t zdt_rx_byte_B = 0;
static volatile uint8_t zdt_rx_byte_C = 0;

/* 收完一字节立刻重挂 Receive_IT，否则只收到 1 字节 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_bus[ZDT_BUS_A].huart) {
        rx_byte_process(&g_bus[ZDT_BUS_A], zdt_rx_byte_A);
        HAL_UART_Receive_IT(huart, (uint8_t *)&zdt_rx_byte_A, 1);
    } else if (huart == g_bus[ZDT_BUS_B].huart) {
        rx_byte_process(&g_bus[ZDT_BUS_B], zdt_rx_byte_B);
        HAL_UART_Receive_IT(huart, (uint8_t *)&zdt_rx_byte_B, 1);
    } else if (huart == g_bus[ZDT_BUS_C].huart) {
        rx_byte_process(&g_bus[ZDT_BUS_C], zdt_rx_byte_C);
        HAL_UART_Receive_IT(huart, (uint8_t *)&zdt_rx_byte_C, 1);
    } else if (huart->Instance == USART6) {
        extern volatile uint8_t hwt_rx_byte;
        HWT601_ByteRx(hwt_rx_byte);
        HAL_UART_Receive_IT(huart, (uint8_t *)&hwt_rx_byte, 1);
    } else if (huart->Instance == USART3) {
        /* ★ 上位机握手阶段（等 0/1 + 等路线帧）：字节全部交给握手逻辑，相机/扫码不解析 */
        if (!See_PreFlowByte(see_rx_byte)) {
            See_RxISR(see_rx_byte);
#if (FOLLOW_RX_EN != 0)
            Follow_RxByte(see_rx_byte);   /* 上位机路径规划帧（FOLLOW_RX_EN = 1 才收）*/
#endif
        }
        HAL_UART_Receive_IT(huart, (uint8_t *)&see_rx_byte, 1);
    } else if (huart->Instance == USART1) {
        Grab6020_CmdRxISR();
    }
}

static void bus_reset_stale(ZdtBus_t bus)
{
    ZdtBusCtx_t *p = &g_bus[bus];
    if (p->st != 0 && (HAL_GetTick() - p->last_rx_tick) > ZDT_FRAME_TIMEOUT_MS) {
        p->st = 0;
        p->pos_idx = 0;
    }
}

static void bus_drop_frame(ZdtBus_t bus)
{
    g_bus[bus].st = 0;
    g_bus[bus].pos_idx = 0;
}

/* ⚠ HAL 出错时已调 EndRxTransfer（RxState 回 READY），不重挂 Receive_IT 就永久哑掉且不报错。 丢半帧即可：各传感器帧头会自行重同步。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_bus[ZDT_BUS_A].huart) {
        bus_drop_frame(ZDT_BUS_A);
        HAL_UART_Receive_IT(huart, (uint8_t *)&zdt_rx_byte_A, 1);
    } else if (huart == g_bus[ZDT_BUS_B].huart) {
        bus_drop_frame(ZDT_BUS_B);
        HAL_UART_Receive_IT(huart, (uint8_t *)&zdt_rx_byte_B, 1);
    } else if (huart == g_bus[ZDT_BUS_C].huart) {
        bus_drop_frame(ZDT_BUS_C);
        HAL_UART_Receive_IT(huart, (uint8_t *)&zdt_rx_byte_C, 1);
    } else if (huart->Instance == USART6) {
        extern volatile uint8_t hwt_rx_byte;
        HAL_UART_Receive_IT(huart, (uint8_t *)&hwt_rx_byte, 1);
    } else if (huart->Instance == USART3) {
        HAL_UART_Receive_IT(huart, (uint8_t *)&see_rx_byte, 1);
    } else if (huart->Instance == USART1) {
        Grab6020_CmdStart();
    }
}

/* ⚠ 发送也依赖 HAL_GetTick() 做超时：中断风暴饿死 tick 时这里照样卡死 */
static bool zdt_send(ZdtBus_t bus, const uint8_t *data, uint16_t len)
{
    bool ok;

    bus_reset_stale(bus);
    if (bus >= ZDT_BUS_MAX) return false;

    /* ★★ 发送互斥（关任务调度、不关中断）： 同一条总线上有两个任务在写，例如 UART5 上 MotorC(prio32) 每 20ms 读 2/7 号， 而 StartTask→GRAB_Update(prio26) 也在发 M2 速度帧。
      * 低优先级那一帧被抢断时，HAL 的 gState 会卡在 BUSY_TX，另一方的 HAL_UART_Transmit 直接返回 HAL_BUSY ⇒ 帧被静默丢弃（实测现象：cmd 连续 16 帧为负、cm
      * 却反向跑了 6.3cm）。 vTaskSuspendAll 只禁任务切换：SysTick 与串口中断照常，所以不会丢相机字节。 最坏情况卡住的时间由下面 20ms 超时兜底。 */
    vTaskSuspendAll();
    /* ★ 超时 20ms → 5ms：最长帧 21B@115200 ≈ 1.82ms，5ms 已留 2.7 倍余量。 原来 20ms 意味着总线卡住时 vTaskSuspendAll 会让**整个调度器**冻结 20ms/帧（一拍 2~6 帧）。 */
    ok = (HAL_UART_Transmit(g_bus[bus].huart, (uint8_t *)data, len, 5) == HAL_OK);
    (void)xTaskResumeAll();

    return ok;
}

static ZdtBus_t bus_of(uint8_t addr)
{
    int i;
    for (i = 0; i < g_bus[ZDT_BUS_A].axis_cnt; i++) {
        if (g_bus[ZDT_BUS_A].axes[i] == addr) return ZDT_BUS_A;
    }
    for (i = 0; i < g_bus[ZDT_BUS_B].axis_cnt; i++) {
        if (g_bus[ZDT_BUS_B].axes[i] == addr) return ZDT_BUS_B;
    }
    for (i = 0; i < g_bus[ZDT_BUS_C].axis_cnt; i++) {
        if (g_bus[ZDT_BUS_C].axes[i] == addr) return ZDT_BUS_C;
    }
    return ZDT_BUS_A;   /* 未知地址兜底走 A */
}

void ZDT_ReadPos(uint8_t addr)
{
    uint8_t cmd[3] = { addr, 0x36, ZDT_TAIL };
    zdt_send(bus_of(addr), cmd, 3);
}

/* 回零：仿 hg 的 Sigan_Home，0x9A + 模式 0x02（无限位碰撞回零）。 只发命令，不等完成（StartTask 固定等 5s）。 */
void ZDT_Home(uint8_t addr)
{
    uint8_t cmd[5] = { addr, 0x9A, 0x02, 0x00, ZDT_TAIL };

    if (addr < 1 || addr >= ZDT_MOTOR_MAX) return;
    zdt_send(bus_of(addr), cmd, 5);
}

/* AA 广播帧：{00,AA,LEN_H,LEN_L,子命令…,6B}，LEN = 子命令字节数 + 5 */
#define MMCL_BUF_MAX    32      /* 速度子命令 8B、停止 5B，32B 够两条 */

static uint8_t  s_mmcl_buf[ZDT_BUS_MAX][MMCL_BUF_MAX];
static uint16_t s_mmcl_len[ZDT_BUS_MAX];
static uint8_t  s_mmcl_cnt[ZDT_BUS_MAX];

void ZDT_MMCL_AddSpeed(ZdtBus_t bus, uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc)
{
    if (bus >= ZDT_BUS_MAX || (s_mmcl_len[bus] + 8) > MMCL_BUF_MAX) {
        return;
    }
    uint8_t *p = &s_mmcl_buf[bus][s_mmcl_len[bus]];
    p[0] = addr;
    p[1] = 0xF6;
    p[2] = dir;
    p[3] = (uint8_t)(rpm >> 8);
    p[4] = (uint8_t)(rpm & 0xFF);
    p[5] = acc;
    p[6] = 0x00;                                    /* sync=0：随 AA 帧一起执行 */
    p[7] = ZDT_TAIL;
    s_mmcl_len[bus] += 8;
    s_mmcl_cnt[bus]++;
}

uint8_t ZDT_MMCL_Send(ZdtBus_t bus)
{
    if (bus >= ZDT_BUS_MAX || s_mmcl_len[bus] == 0) {
        return 0;
    }
    uint16_t n   = s_mmcl_len[bus];
    int  cnt = s_mmcl_cnt[bus];
    uint8_t  frame[MMCL_BUF_MAX + 5];
    uint16_t total = n + 5;

    frame[0] = 0x00;
    frame[1] = 0xAA;
    frame[2] = (uint8_t)(total >> 8);
    frame[3] = (uint8_t)(total & 0xFF);
    memcpy(&frame[4], s_mmcl_buf[bus], n);
    frame[4 + n] = ZDT_TAIL;

    s_mmcl_len[bus] = 0;
    s_mmcl_cnt[bus] = 0;
    zdt_send(bus, frame, total);
    return cnt;
}

/* 位置帧 13B：{Addr,FD,Dir,SP_H,SP_L,ACC,P3..P0,Mode,SYNC,6B}；3200 脉冲/圈 = 1.8° 电机 16 细分 */
#define ZDT_PULSES_PER_REV     3200.0f
#define ZDT_MODE_ABS_TAKEOVER  0x01   /* Mode=1：绝对值模式接管 */

void ZDT_PosCtrl(uint8_t addr, float deg, uint16_t speed_rpm, uint8_t acc)
{
    int  dir = (deg < 0.0f) ? 0x01 : 0x00;
    uint32_t clk;
    uint8_t  cmd[13];

    if (addr < 1 || addr >= ZDT_MOTOR_MAX) return;
    clk = (uint32_t)(fabsf(deg) * (ZDT_PULSES_PER_REV / 360.0f));

    cmd[0]  = addr;
    cmd[1]  = 0xFD;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(speed_rpm >> 8);
    cmd[4]  = (uint8_t)(speed_rpm & 0xFF);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(clk >> 24);
    cmd[7]  = (uint8_t)(clk >> 16);
    cmd[8]  = (uint8_t)(clk >> 8);
    cmd[9]  = (uint8_t)(clk);
    cmd[10] = ZDT_MODE_ABS_TAKEOVER;
    cmd[11] = 0x00;
    cmd[12] = ZDT_TAIL;
    zdt_send(bus_of(addr), cmd, 13);
}

/* 单机速度模式：{addr,F6,dir,SP_H,SP_L,acc,sync=0,6B}，和打包帧同格式 */
void ZDT_SpeedCtrl(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc)
{
    uint8_t cmd[8];

    if (addr < 1 || addr >= ZDT_MOTOR_MAX) return;
    cmd[0] = addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(rpm >> 8);
    cmd[4] = (uint8_t)(rpm & 0xFF);
    cmd[5] = acc;
    cmd[6] = 0x00;
    cmd[7] = ZDT_TAIL;
    zdt_send(bus_of(addr), cmd, 8);
}

/* 任务侧换算 raw→角度(°)；65536 = 一整圈（16 位角度） */
void ZDT_UpdateAngle(uint8_t addr)
{
    if (addr < 1 || addr >= ZDT_MOTOR_MAX) return;
    zdt_angle[addr] = (float)zdt_raw[addr] * (360.0f / 65536.0f);
    /* 只有"真的收到过一帧"才置位：先写角度、后置标志 ⇒ 看到标志时 angle 一定已是真值。 ⚠ 判的是 last_update 的哨兵，不是年龄 —— 本函数每 10ms 都被调用（没新帧也调）， 若无条件置位就会在 raw 仍是 0 时宣布"就绪"，那正是要防的那个坑。 */
    if (zdt_last_update[addr] != 0xFFFFF000U) zdt_ready[addr] = 1;
}

/* 上电以来该轴出过至少一帧位置应答（不看年龄、永不回假）：给差分基线用 */
bool ZDT_HasAngle(uint8_t addr)
{
    if (addr < 1 || addr >= ZDT_MOTOR_MAX) return false;
    return zdt_ready[addr] != 0;
}

bool ZDT_AngleFresh(uint8_t addr, uint32_t max_age_ms)
{
    if (addr < 1 || addr >= ZDT_MOTOR_MAX) return false;
    return (HAL_GetTick() - zdt_last_update[addr]) < max_age_ms;
}

void ZDT_MotorInit(void)
{
    int i;

    for (i = 0; i < ZDT_MOTOR_MAX; i++) {
        zdt_raw[i]         = 0;
        zdt_angle[i]       = 0.0f;
        zdt_ready[i]       = 0;
        zdt_last_update[i] = 0xFFFFF000U;   /* 远古值：让 ZDT_AngleFresh 上电即为假 */
    }

    g_bus[ZDT_BUS_A].huart    = &huart2;
    g_bus[ZDT_BUS_A].axes     = g_busA_axes;
    g_bus[ZDT_BUS_A].axis_cnt = sizeof(g_busA_axes);
    g_bus[ZDT_BUS_A].st       = 0;

    g_bus[ZDT_BUS_B].huart    = &huart4;
    g_bus[ZDT_BUS_B].axes     = g_busB_axes;
    g_bus[ZDT_BUS_B].axis_cnt = sizeof(g_busB_axes);
    g_bus[ZDT_BUS_B].st       = 0;

    g_bus[ZDT_BUS_C].huart    = &huart5;
    g_bus[ZDT_BUS_C].axes     = g_busC_axes;
    g_bus[ZDT_BUS_C].axis_cnt = sizeof(g_busC_axes);
    g_bus[ZDT_BUS_C].st       = 0;

    HAL_UART_Receive_IT(&huart2, (uint8_t *)&zdt_rx_byte_A, 1);
    HAL_UART_Receive_IT(&huart4, (uint8_t *)&zdt_rx_byte_B, 1);
    HAL_UART_Receive_IT(&huart5, (uint8_t *)&zdt_rx_byte_C, 1);
}

void ZDT_Poll(void)
{
    ZdtBus_t b;
    for (b = ZDT_BUS_A; b < ZDT_BUS_MAX; b++) {
        bus_reset_stale(b);
    }
}
