/* * hwt601.h —— HWT601-AGV-485 姿态驱动：USART6(PC6/PC7)、115200 8N1、Modbus-RTU 从机 0x50、无主动回传只能轮询，对外只给 Yaw。 */
#ifndef __HWT601_H
#define __HWT601_H

#include <stdint.h>
#include <stdbool.h>

#define HWT601_MODBUS_ADDR   0x50    /* 出厂从机地址 */
#define HWT601_REG_AX        0x34    /* 连读起始寄存器 */
#define HWT601_REG_READ_N    12      /* 连读个数：0x34~0x3F */

#define HWT601_ANGLE_SCALE   (180.0f / 32768.0f)      /* Yaw 每 LSB = ° */
/* 加速度量程 ±4g（实测：静止 AZ 恒 8172 count = 1g ⇒ 32768/8172 ≈ 4g）；⚠ 别用不知道抬了多高的读数反推量程，曾误判成 ±16g。
 * 本条只记录量程结论：ax/ay 目前不计算，重启加速度时要按这个量程算。 */

/* 陀螺 Z 量程 ±400°/s 是估算值（积分闭合 ≈82 count/(°/s)）；准确值待定速标定：匀速转一圈记用时 T ⇒ 360/T。 */

#define HWT601_POLL_PERIOD_MS    10                            /* 轮询周期 ms：10 = 100Hz */
#define HWT601_FRAME_GAP_MS      (HWT601_POLL_PERIOD_MS + 5)   /* 静默超此值 → 判半帧 */
#define HWT601_REPLY_TIMEOUT_MS  (HWT601_POLL_PERIOD_MS + 5)   /* 全无字节 → 判丢，允许重发 */

void  HWT601_Init(void);                  /* 挂 USART6 单字节 IT 接收 */
void  HWT601_Poll(void);                  /* 周期调：HWT601_POLL_PERIOD_MS 一次 */
void  HWT601_ByteRx(uint8_t byte);        /* 由 HAL_UART_RxCpltCallback(zdt_motor.c) 分发进来 */

float HWT601_GetYaw(void);                /* 航向角(°)，[0,360) */
bool  HWT601_HasFrame(void);              /* 上电以来收到过至少一帧好帧：不看年龄、永不回假，起步门专用 */

#endif /* __HWT601_H */
