/* new.c.h —— 新整体流程函数（替代原来的 TaskFlow_Update() + GRAB_Update()）
 * 用法：
 *   - freertos.c 里定义一个定时器或周期任务，每 20ms 调用一次 NewTaskFlow_Update();
 *   - 该函数内部会推进底盘流程和手臂流程的联合状态机。
 */

#ifndef __NEW_C_H
#define __NEW_C_H

#include <stdint.h>
#include "chassis_cfg.h"     /* CTRL_DT_MS */
#include "grab_cfg.h"        /* TRACK_* 等参数 */
#include "M27.h"             /* M27_Arrived() / Grab6020_Arrived_Big() */


/* ========== 流程状态机：用联合状态表示底盘 + 手臂的当前阶段 ========== */

typedef enum {
    NEW_STEP_MOVE_M2_IDLE = 0,      /* case-1: MoveCm(90,0,3,900,100,0,900,100)，M2 原地不动 */
    NEW_STEP_WAIT_ARRIVE,            /* case-0: 等待 M27_Arrived() && Grab6020_Arrived_Big()，超时 5s */
    NEW_STEP_TRACKING,               /* case-2: 追踪模式（不管目标颜色直接追） */
    NEW_STEP_GRAB_JUDGE,             /* case-3: cx<15 && cy<15 => 抓取 */
    NEW_STEP_PLACE_ON_SERVO_M1,      /* 放在"舵机物料 1"上面 */
    NEW_STEP_ROTATE_90,              /* 自转 90° */
    NEW_STEP_Y_LOW_SPEED_MOVE,       /* Y 方向低速平移 */
    NEW_STEP_TRACKING_TARGET,        /* 看见目标：根据 cx/cy 追踪 */
    NEW_STEP_PLACE_DONE              /* 执行放置完成 */
} new_step_t;

/* ========== 全局流程状态（单消费者） ========== */
extern volatile new_step_t g_new_step;
extern volatile uint8_t g_new_substep;     /* 子步：0=N/A，1=等待中，2=抓取判定窗口等 */
extern volatile int       g_new_last_color;/* 当前追踪目标颜色（诊断用）*/

/* ========== API ========== */

void    NewTaskFlow_Init(void);            /* StartTask 里调一次：清状态 */
void    NewTaskFlow_Ready(void);           /* StartTask 循环前调：取初始位姿/等待条件 */
void    NewTaskFlow_Update(void);          /* 循环里每 20ms 调用：推进联合流程 */

/* ========== 内部状态机实现（不要动）========== */

/* case-1: MoveCm(90,0,3,900,100,0,900,100)，M2 原地不动，PD13 之前手臂不伸 */
void _NewStep_MoveM2Idle(void);

/* case-0: 等待 M27_Arrived() && Grab6020_Arrived_Big()，最多 5s（HAL_GetTick()-t0<5000） */
void _NewStep_WaitArrive(uint32_t *p_wait_start);

/* case-2: 追踪模式（不管目标颜色是什么就直接追） */
void _NewStep_Tracking(void);

/* case-3: 抓取判定：cx<15 && cy<15 => 开始抓取 */
void _NewStep_GrabJudge(void);

/* 放在"舵机物料 1"上面 */
void _NewStep_PlaceOnServoM1(void);

/* 自转 90° */
void _NewStep_Rotate90(void);

/* Y 方向低速平移 */
void _NewStep_YLowSpeedMove(void);

/* 看见目标：根据 cx/cy 进行追踪 */
void _NewStep_TrackingTarget(float cam_cx, float cam_cy);

/* 执行放置（最终把料放进圆环） */
void _NewStep_PlaceDone(void);


#endif /* __NEW_C_H */