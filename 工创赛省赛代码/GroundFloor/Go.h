#ifndef __GO_H
#define __GO_H

/* Go.h —— 底盘运动接口：目标下发 + 相差距离 + 到位判定（实现在 Go.c）。 */

#include <stdint.h>
#include "ins.h"        /* InsState_t：位姿的唯一定居处，Go 只读它 */

typedef struct {
    float ex, ey, eyaw;         /* 相差距离：目标 − 当前（世界系 cm / °） */
    float tar_x, tar_y;         /* 要去哪：目标位置（任务系 cm） */
    float tar_yaw;              /* 目标角度 ° */
    uint32_t tar_seq;           /* 目标序号：换目标 +1（到位标志靠它认领） */
    uint32_t arrived_seq;       /* 控制周期最近一次判定用的是哪一号目标 */
    int  arrived;           /* 1 = 当前目标已到位 */
    float tol_xy;               /* 位置死区 cm */
    float tol_yaw;              /* 角度死区 ° */
    float yaw0;                 /* 任务系基准 = 开跑车头方向 °（GO_Home 锁一次） */
    int held;               /* 1 = 已就地停住：目标冻在当前位姿，控制周期出零 */
    int split;              /* 1 = 分离式转弯的直行段：航向还锁着 */
    float lock_yaw;             /* 直行段锁住的航向 = 下目标那一刻的车头 ° */
    float ff_vx, ff_vy;         /* 跟踪目标速度前馈，世界系 cm/s */
} ChassisCtl_t;

/* 两种下目标：GO 相对（加到当前目标上）、GO_To 任务系绝对（+X = 开跑车头方向）。 剧本（TaskFlow.c）用法：下目标 → 每拍问 GO_Arrived() → 到点换下一步。 */
void    GO_Init(const InsState_t *pose);           /* 上电一次：绑定位姿（只借指针）+ 填死区 */
uint8_t GO_Home(void);                             /* 锁任务系基准航向（位置归零由 Motion_Home 做；只上电一次） */
void    GO_ZeroXY(void);                           /* 只清目标 XY，不动 yaw0 / tar_yaw（起点摆位后重定原点） */

void    GO(float dx, float dy, float dyaw); /* 相对：加到当前目标上，立刻返回：GO(60,0,0) 沿 +X 走 60cm，GO(0,0,90) 原地转 90°；平移 + 大角度 = 先直行、到点前再甩头（只有它吃 GO_TURN_LATE_*，见
                                             * chassis_cfg.h） */
void    GO_To(float x, float y, float dyaw);       /* 任务系**绝对**位置 + **相对**朝向：dyaw 加到当前目标朝向（0 = 不转）；同 GO 的分离式转弯，最后 GO_TURN_LATE_CM 才甩头。
                                                    * ⚠ 相对角是累加的：同一条腿只能下发一次，每拍重复调会 90→180→270… */
void    GO_TrackX(float step);                     /* 跟踪专用：目标 = 当前实际任务系 X + step，不累积上一目标 */
void    GO_TrackXFF(float step, float ff_vx);      /* 同上，另加**任务系 X** 目标速度前馈 cm/s（内部按 yaw0 转到世界系） */
void    GO_TrackY(float step);                     /* 同 GO_TrackX，但只挪任务系 Y（左出发用） */
void    GO_TrackYFF(float step, float ff_vy);      /* 同上 + 任务系 Y 速度前馈 */
float   GO_GetVy(void);                            /* 车在任务系 Y 上的速度 cm/s */
float   GO_GetVx(void);                            /* 车在任务系 X 上的速度 cm/s（前馈用） */
uint8_t GO_Arrived(void);                          /* 1 = 刚下发的这个目标已到位 */
void    GO_GetTar(float *x, float *y);             /* 读当前目标（任务系 cm）：追踪 / 跑飞保护要 */
float   GO_GetXTask(void);                         /* 只读当前位姿的任务系 X（+X = 开跑车头方向）*/
float   GO_GetYTask(void);                         /* 只读当前位姿的任务系 Y（+Y = 开跑时的左侧）*/
float   GO_GetYawTask(void);                       /* 当前车头相对任务系的方位角 °（0 = 开跑车头，逆时针正）*/
float   GO_GetYawTar(void);                        /* 目标航向（任务系 °，诊断用）*/
void    GO_SetTaskPos(uint8_t axis, float v);      /* 把里程计任务系坐标强行纠正到 v：axis 1=X / 2=Y（另一轴不动）；改完要重设目标 */
void    GO_Stop(void);                             /* 就地停住：目标冻在当前位姿 + 强制出零速度 */

void    GO_SetTol(float xy);                       /* 改到位死区 cm（跟踪用：1.2cm 会把 0.8cm 的小步当成已到位 → 不动） */
void    GO_Crawl(uint8_t on);                      /* 蠕动档：1 = 之后平移按蠕动速度，只影响限速与刹车包线（默认 0） */
void    GO_SetVmax(float v);                       /* 平移速度上限 cm/s：>0 生效，<=0 恢复默认档（只压上限，不改刹车包线） */

void    GO_ControlCycle(float *wheel_rpm);         /* 每 20ms 一拍：算相差距离 → pid → ins */

void    GO_OdomBaseReset(void);                    /* 实现在 freertos.c：四轮差分基准对齐 */

#endif /* __GO_H */
