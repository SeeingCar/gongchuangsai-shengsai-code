/* pid.h —— 位置/航向环接口：相差距离 → 本拍命令；三个环都只用 kp（纯 P，ki/kd 未实现），dt 固定 20ms。 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>     /* uint8_t：PID_Crawl 的档位 */

typedef struct {
    float kp;
} Pid_t;

/* 世界系（不是车体系）平移速度 + 角速度 */
typedef struct {
    float vx, vy;       /* cm/s */
    float w;            /* °/s */
} PidOut_t;

void PID_Init(void);                               /* 上电一次：灌三个环的增益 */
void PID_Clear(void);                              /* 清斜坡基准（停车 / 重新开始时调） */
void PID_SetTrackFF(float vx, float vy);           /* 跟踪目标速度前馈，世界系 cm/s；绕过位置误差死区 */
void PID_Crawl(uint8_t on);                        /* 拨蠕动档：0 = 常规 / 1 = 慢速 */
void PID_SetVmax(float v);                         /* 平移速度上限覆盖 cm/s：>0 生效，<=0 恢复默认档 */
PidOut_t PID_Calc(float ex, float ey, float eyaw); /* 每 20ms 一拍 */

#endif /* __PID_H */
