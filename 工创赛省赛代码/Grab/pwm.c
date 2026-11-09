/* pwm.c —— 两路 50Hz 舵机：TIM4 PSC=84 → 1MHz（1 计数 = 1µs）、ARR=20000 → 50Hz；CH1=PB6 夹爪、CH2=PB7 转盘。 角度↔脉宽：0°→500µs、270°→2500µs 线性（舵机固定规格，不随调参变）。 */
#include "pwm.h"
#include "tim.h"
#include "grab_cfg.h"

static inline uint32_t servo_pulse(float deg)
{
    float d = deg;

    if (d < 0.0f)   d = 0.0f;
    if (d > 270.0f) d = 270.0f;

    return (uint32_t)(500.0f + (2500.0f - 500.0f) * (d / 270.0f));
}

void Servo_Init(void)
{
    /* ⚠ PWM 未启动时 CCR=0，舵机会往极限拧：先写安全位再开通道 */
    Gripper_Close();                       /* 上电默认闭合 */
    Servo2_SetAngle(CAROUSEL_ANG_INIT);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void Servo_SetAngle(float deg)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, servo_pulse(deg));
}

void Servo2_SetAngle(float deg)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, servo_pulse(deg));
}

void Gripper_Open(void)  { Servo_SetAngle(GRIP_OPEN_DEG);  }
void Gripper_Close(void) { Servo_SetAngle(GRIP_CLOSE_DEG); }
