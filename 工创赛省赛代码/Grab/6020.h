/* 6020.h —— DJI GM6020 旋转关节（ID=1）双环：角度环 → 角速度环 → 0x1FF 电压给定。 0x1FF：电压给定 ±25000；反馈 0x205：[0..1]机械角 0~8191、[2..3]rpm、[4..5]转矩电流；[6] 不用。不用 IMU。
 * 用法：Init() 开机一次；Update() 每 5ms（跑在 Motor6020 任务里）；CanRx() 放 CAN1 RX0 中断；SetAngle(°) 给目标。 */
#ifndef __6020_H
#define __6020_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

void Grab6020_Init(void);                      /* 开机一次：配 CAN 滤波器 + 启动 */
void Grab6020_Update(void);                    /* 每 5ms：角度环 → 角速度环 → 发帧 */
void Grab6020_CanRx(CAN_HandleTypeDef *hcan); /* CAN1 RX0 中断里收反馈 */

/* 目标关节角(°)；rate_step = 角速度上升限幅(°/s 每 5ms)，0 = 用默认 M6020_RATE_STEP。 给更小的值就转得慢；配合 Grab6020_SetRateMax() 压最高速。非阻塞。 */
void  Grab6020_SetAngle(float deg, float rate_step);
void  Grab6020_SetRateMax(float dps);          /* 最高转速 °/s，<=0 恢复默认 600 */
float Grab6020_GetAngle(void);        /* 当前关节角(°)，上电为 0 */
bool  Grab6020_Arrived(void);         /* 到位：角度进容差 且 角速度够小 */
bool Grab6020_Arrived_Big(void);   /* 宽松到位：角度容差 1 度（速度门限同 Arrived）；case6 不用它 */

float    Grab6020_GetTarget(void);    /* 目标关节角 ° */

void Grab6020_Stop(void);             /* 目标 = 当前角 */

/* 串口在线调参（USART1，编号-数值，换行结束） */
void    Grab6020_CmdStart(void);              /* 挂第一字节接收（Init 里调） */
void    Grab6020_CmdRxISR(void);              /* USART1 RX 中断里调 */
uint8_t Grab6020_CmdPoll(char *out, uint8_t max);  /* 任务里调：1=取到一行 */

#endif /* __6020_H */
