/* hwt601.c —— HWT601-AGV-485 MODBUS-RTU 驱动：USART6(PC6/PC7)、115200 8N1、从机 0x50、无 485 方向脚，对外只给 Yaw。 */
#include "hwt601.h"
#include "usart.h"
#include <string.h>

/* zdt_motor.c 的 RxCplt 回调 extern 引用此字节，故非 static */
volatile uint8_t hwt_rx_byte = 0;

/* ISR 与 HWT601_Poll 都会写，故 volatile */
static struct {
    volatile uint8_t  rx[40];              /* 一帧 29B，40 留余量 */
    volatile int  rx_len;
    volatile uint32_t last_rx_tick;        /* 最后一个字节的时刻（判半帧静默） */
    volatile int  wait_reply;          /* 1=请求在途，防在途重复发 */
    uint32_t          req_tick;            /* 请求发出时刻（仅 wait_reply=1 时有效） */
    volatile int  tx_busy;             /* 1=发送中（等 TC 中断清 0） */
    uint8_t           tx_buf[8];
} st_run;

static struct {
    volatile float    yaw, ax, ay;         /* °[0,360) / m/s² / m/s² */
    volatile int16_t  raw[HWT601_REG_READ_N];  /* 整帧原始寄存器（顺序 0x34..0x3F），认寄存器用 */
    volatile uint32_t last_ok;
} st_out = { 0.0f, 0.0f, 0.0f, {0}, 0xFFFFF000U };  /* 哨兵初值：上电未收到帧时 IsFresh() 判否 */

#define HWT601_RPLY_LEN   (3 + HWT601_REG_READ_N * 2 + 2)   /* 29B = 头 3B + 数据 24B + CRC 2B */

static uint16_t crc16_modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6) {
        st_run.tx_busy = 0;
    }
}

/* ⚠ 必须 memcpy：Transmit_IT 只登记指针，栈数组一返回就失效；启动失败也要 AbortTransmit，否则 TC 中断丢了会把 gState 卡在 BUSY_TX */
static bool tx_start(const uint8_t *buf, uint16_t len)
{
    if (st_run.tx_busy || len > sizeof(st_run.tx_buf)) return false;

    memcpy(st_run.tx_buf, buf, len);
    st_run.tx_busy = 1;

    if (HAL_UART_Transmit_IT(&huart6, st_run.tx_buf, len) != HAL_OK) {
        (void)HAL_UART_AbortTransmit(&huart6);   /* 回收可能卡住的 BUSY_TX */
        st_run.tx_busy = 0;
        return false;
    }
    return true;
}

/* 固定请求，线上恒为 50 03 00 34 00 0C 09 80（读 0x34 起 12 个寄存器） */
static bool req_read_all(void)
{
    uint8_t cmd[8];
    uint16_t crc;
    cmd[0] = HWT601_MODBUS_ADDR;
    cmd[1] = 0x03;
    cmd[2] = (uint8_t)(HWT601_REG_AX >> 8);
    cmd[3] = (uint8_t)(HWT601_REG_AX & 0xFF);
    cmd[4] = 0x00;
    cmd[5] = HWT601_REG_READ_N;
    crc = crc16_modbus(cmd, 6);
    cmd[6] = (uint8_t)(crc & 0xFF);         /* CRC 低字节在前 */
    cmd[7] = (uint8_t)(crc >> 8);
    return tx_start(cmd, 8);
}

/* ISR 只存字节 + 记时刻，拼帧/校验/换算在 HWT601_Poll */
void HWT601_ByteRx(uint8_t byte)
{
    if (st_run.rx_len < 40) {
        st_run.rx[st_run.rx_len++] = byte;
    } else {
        st_run.rx_len = 0;                   /* 溢出：整帧丢弃重收 */
    }
    st_run.last_rx_tick = HAL_GetTick();
}

/* 关 RXNE 再快照清零：解析只读副本，不会读到半更新的帧 */
static uint8_t capture_frame(uint8_t *dst, uint8_t dst_max)
{
    int i, n;

    __HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);   /* 停 RX，防 ISR 写入 */
    n = (st_run.rx_len < dst_max) ? st_run.rx_len : dst_max;
    for (i = 0; i < n; i++) {
        dst[i] = st_run.rx[i];
    }
    st_run.rx_len = 0;                              /* 清缓冲（中断关闭中，安全） */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    return n;
}

/* 长度 / 地址(0x50) / 功能码 / 字节数 / CRC 全过才更新数据；数据区 int16 大端：第 k 个寄存器(0x34+k) 在 frame[3+2k]/frame[4+2k]。
 * 寄存器表（实测认领）：0x34 AX、0x35 AY、0x36 AZ、0x37 GX、0x38 GY、0x39 GZ、0x3F Yaw；0x3A~0x3C 恒 0，0x3D/0x3E 小量程未认。 */
static void parse_frame(const uint8_t *frame, uint8_t n)
{
    int k;

    if (n < HWT601_RPLY_LEN) {
        return;
    }
    if (frame[0] != HWT601_MODBUS_ADDR || frame[1] != 0x03 ||
        frame[2] != (HWT601_REG_READ_N * 2)) {
        return;
    }
    uint16_t crc_calc = crc16_modbus(frame, 3 + 24);
    uint16_t crc_rx   = (uint16_t)frame[HWT601_RPLY_LEN - 2] |
                        ((uint16_t)frame[HWT601_RPLY_LEN - 1] << 8);
    if (crc_calc != crc_rx) {
        return;
    }

    /* 12 个全存：未认的靠标定时"抬车/推车看哪个数变"认领 */
    for (k = 0; k < HWT601_REG_READ_N; k++) {
        st_out.raw[k] = (int16_t)(((uint16_t)frame[3 + 2 * k] << 8) | frame[4 + 2 * k]);
    }

    /* ⚠ 全工程只有这一处产生航向；0~360 被裸相减会在跨 0° 时得 ±360 错值，任何角度差都走 mf_ang_err()。 */
    st_out.yaw = (float)st_out.raw[11] * HWT601_ANGLE_SCALE;   /* +360 归一：无符号 0~360 与有符号 ±180 两种发法都还原成 [0,360) */
    if (st_out.yaw < 0.0f) st_out.yaw += 360.0f;
    st_out.last_ok = HAL_GetTick();
}

void HWT601_Init(void)
{
    st_run.rx_len = 0;
    st_run.last_rx_tick = 0;
    st_out.last_ok = 0xFFFFF000U;
    st_run.wait_reply = 0;
    st_run.req_tick = 0;
    st_run.tx_busy = 0;
    HAL_UART_Receive_IT(&huart6, (uint8_t *)&hwt_rx_byte, 1);
}

/* 周期调（HWT601_POLL_PERIOD_MS）：① 收满帧→快照解析 ② 半帧静默丢弃 ③ 请求超时 ④ 空闲才发下一条（先发再置 wait_reply，否则超时起点错）。 ⚠ 收帧判据用
 * >=（多余字节交给校验否掉）；解析成败都要解除 wait_reply，否则坏帧会让轮询永久停摆。 */
void HWT601_Poll(void)
{
    uint8_t frame[40];
    uint32_t now = HAL_GetTick();

    if (st_run.rx_len >= HWT601_RPLY_LEN) {
        capture_frame(frame, sizeof(frame));
        parse_frame(frame, HWT601_RPLY_LEN);
        st_run.wait_reply = 0;
        st_run.req_tick = 0;
    }
    else if (st_run.rx_len > 0 && (now - st_run.last_rx_tick) > HWT601_FRAME_GAP_MS) {
        capture_frame(frame, sizeof(frame));
        st_run.wait_reply = 0;
    }

    if (st_run.wait_reply && (now - st_run.req_tick) > HWT601_REPLY_TIMEOUT_MS) {
        st_run.wait_reply = 0;
    }

    if (st_run.rx_len == 0 && !st_run.wait_reply) {
        if (req_read_all()) {
            st_run.wait_reply = 1;
            st_run.req_tick = now;
        }
    }
}

float HWT601_GetYaw(void) { return st_out.yaw; }   /* 消费方 MotorTx（freertos.c）填 in.imu_yaw */

/* 上电以来是否收到过至少一帧好帧：只看"有没有来过"，不看"多久以前"。 专给起步门用（Motion_Home 要等它）⇒ **不会**因为运行中丢帧翻假，也没有超时参数可调错。 初值哨兵 0xFFFFF000 让它上电即为假，收到第一帧后永久为真。 */
bool HWT601_HasFrame(void)
{
    return st_out.last_ok != 0xFFFFF000U;
}
