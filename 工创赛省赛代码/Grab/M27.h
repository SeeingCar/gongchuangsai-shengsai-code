/* M27.h —— 手臂两轴绝对位置控制：单位 cm、0cm = 驱动器零点（绝对量，不是上电位置）。 FB = 前后（伸缩）= 电机2；UD = 上下（升降丝杆）= 电机7；都在总线C/UART5。 位置帧 3200
 * 脉冲/圈（发指令），0x36 反馈 65536 计数/圈（读角度），两种尺度别混。 */
#ifndef __M27_H
#define __M27_H

#include <stdbool.h>
#include <stdint.h>

void  M27_Init(void);            /* 上电一次：清状态（0cm 基准在首次读到角度时取） */
void  M27_Update(void);          /* 20ms，仅 MotorC 调：刷新 cm 与到位判定（**不发帧**）*/
void  M27_Send(void);            /* 20ms，仅 MotorC 调：有新目标就发两轴绝对位置帧（间隔 5ms）*/

void    M27_Home(void);          /* 请求 M2/M7 回零（0x9A 02 无限位碰撞回零），由 MotorC 发 */

/* 手臂一次设三轴目标：6020 角度(°, 角加速度) + 前后(cm,rpm,acc) + 上下(cm,rpm,acc)。只改目标，不发帧。 ang_step = 6020 角速度上升限幅（0 = 默认，给 2.5 这种小值就转得慢）。
 * 6020 由 Motor6020 的 5ms 双环发；M2/M7 由 MotorC 调 M27_Send() 发。 ⚠ 三轴都必给（不想动的轴用 Get 回填）；速度 0~3000 rpm、加减速档 0~255，常用 900 / 100。 */
/* M2 模式：0=位置（MoveCm），1=速度（M2Speed）；仅 MotorC 发帧 */
extern volatile int m2_mode;

/* 只改 M2 目标速度 cm/s（正=伸长），不发帧；由 MotorC 的 M27_Send 发 */
void  M2Speed(float cm_s, uint8_t acc);

void  MoveCm(float deg, float ang_step,
             float fb_cm, uint16_t fb_rpm, uint8_t fb_acc,
             float ud_cm, uint16_t ud_rpm, uint8_t ud_acc);

bool  M27_UdArrived(void);      /* 只判上下轴到位 */
bool  M27_Arrived(void);         /* 两轴都进容差（判据是 0x36 反馈，不是驱动器应答） */
bool  M27_FbArrived(void);       /* 只看 FB 到位（扫描 M2 用，UD 不参与） */
extern float cm_2;               /* FB 伸缩当前 cm */
extern float cm_7;               /* UD 升降当前 cm（保持 UD 不动时用它回填目标） */

#endif /* __M27_H */
