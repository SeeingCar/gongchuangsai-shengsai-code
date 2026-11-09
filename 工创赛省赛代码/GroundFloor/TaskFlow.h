/* TaskFlow.h —— 底盘流程状态机（第一台；第二台是 grab.c，两台在同一个 while 里各推进一步）。
 * 用法：循环外 TaskFlow_Init() + TaskFlow_Ready()，循环里 TaskFlow_Update() + GRAB_Update() + osDelay(20)。 */
#ifndef __TASKFLOW_H
#define __TASKFLOW_H

#include <stdint.h>
#include "chassis_cfg.h"     /* CTRL_DT_MS */


/* 实现在 freertos.c：里程计 / 四轮差分基准 / Go 基准必须同一时刻归零，否则造假位移/假航向；返回 1 = 已归零就绪。 */
uint8_t Motion_Home(void);
/* 只清 XY、不动航向：留给"起点摆位后重定原点"的场景。本工程 0 调用——实跑的是 Motion_Home（航向基准只允许上电锁一次，重锁会把车头当新基准）。 */
uint8_t Motion_ZeroXY(void);

void    TaskFlow_Init(void);      /* StartTask 里调一次：清状态 */
void    TaskFlow_Ready(void);     /* StartTask 里、循环之前调一次：等到数据就绪 + 取初始位姿 */
void    TaskFlow_Update(void);    /* 循环里每 20ms 一次：推进一步 */

/* 在线调参只剩串口 0（急停/走）和 1/3/5（case2 与 case5 两轮的方向）；2/4 归手臂。K/KD/FF/步长/死区/低通/比例尺的串口项已删，改宏后重编。 */
float    TaskFlow_CarV(void);        /* 车在"跟踪轴"上的速度 cm/s（右出发=任务X，左出发=任务Y）*/
uint8_t  TaskFlow_BaseValid(void);   /* 1 = 基准已定 */
extern const uint8_t g_case6_key;    /* 1 = 走 7/9 的 PD 对准；0 = 只用理论值直接放料 */
extern const uint8_t g_case11_key;   /* 1 = 走 case 11；0 = 跳过 case 11 直接 case 12 */
extern const uint8_t g_grab0;         /* 1 = 单独测抓取：跳过前段直接进 case 1，抓完三块就停（定义在 freertos.c）*/
extern const uint8_t g_base_en;       /* 1 = 完整基准判定；0 = 直接抓（基准恒 0，判停用抓取门，成立即抓）*/
extern int       g_c12_ready;    /* 1 = 底盘已到 8 号环位置（仅 case11_key=0 时手臂等它）*/
extern int       Go_now;         /* 底盘当前 case（诊断/遥测用）*/
extern int       big_round;      /* 0 = 第1遍（A/B 段），1 = 第2遍（C/D 段）*/
float   TaskFlow_SetSign(uint8_t i, float v);   /* 1=case2底盘 2=case5一轮 3=case5二轮；1 正 / 0 反 */

#endif /* __TASKFLOW_H */
