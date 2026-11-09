/* ins.h —— 底盘运动学换算（正解 INS_Update + 逆解 INS_OutToWheels，一对）。 航向只用 IMU 绝对角（轮子不参与）；周期固定 CTRL_DT_S；单位 cm / ° / rpm。
 * 位姿本身没有 valid 位：运行中不做任何有效性判定（判了就会变成"掉线即停车"）。 唯一需要"数据到底来没来"的地方是起步归零 —— 那次由 Motion_Home() 直接问 HWT601_HasFrame()。 */
#ifndef __INS_H
#define __INS_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"      /* PidOut_t：逆解的输入 */

/* 输入：每个控制周期由 MotorTx 汇总底层数据 */
typedef struct {
    float wheel_rpm[4];     /* 四轮实测转速 rpm（含方向符号，已按 WHEEL_DIR 校正） */
    float imu_yaw;          /* HWT601 航向 °，[0,360)：全链路唯一的航向来源 */
} InsInput_t;

/* 输出：推算出的车体状态（Go 层与 MotorTx 消费） */
typedef struct {
    float x, y;                 /* 位置：世界系 cm */
    float yaw;                  /* 航向 °：就是 IMU 的绝对角，全链路唯一来源（不融合轮速） */
    float body_vx, body_vy;     /* 车体系实测速度 cm/s：横移诊断看这份，纯横移时 body_vx 必须 ≈0 */
    float world_vx, world_vy;   /* 世界系速度 cm/s：位置积分用 */
} InsState_t;

void INS_Init(InsState_t *s);

void INS_Update(InsState_t *s, const InsInput_t *in);

void INS_OutToWheels(float yaw, PidOut_t out, float *wheel_rpm);

/* 只把位置与速度清零，**不动航向**（任务起点当原点用，见 Motion_Home） */
void INS_ZeroPos(InsState_t *s);

#endif /* __INS_H */
