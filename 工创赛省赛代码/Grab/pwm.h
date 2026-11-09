/* pwm.h —— 两路 50Hz 舵机：TIM4_CH1=PB6 夹爪、CH2=PB7 转盘（3 槽位）；脉宽 500~2500µs ↔ 0~270°。 舵机走独立 BEC，只与主板共地，不要从板子 5V 轨取电。 */
#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

void Servo_Init(void);                       /* 上电调一次：写安全位 + 启动两路 PWM */
void Servo_SetAngle(float deg);              /* 夹爪角度 */
void Servo2_SetAngle(float deg);             /* 转盘角度（三个物料的角度在 grab_cfg.h）*/
void Gripper_Open(void);
void Gripper_Close(void);

#endif /* __PWM_H */
