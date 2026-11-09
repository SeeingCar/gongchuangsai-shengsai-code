/* zdt_motor.h —— 张大头 ZDT X42S 闭环步进（6 轴 / 三总线 / TTL 115200 8N1，帧尾 0x6B） 电机平时静默，只回 0x36 读位置应答 {Addr,36,SIGN,P3..P0,6B}，角度(°)=±raw*360/65536
 * 命令帧：使能{Addr,F3,AB,EN,SYNC,6B}｜速度{Addr,F6,DIR,SP_H,SP_L,ACC,SYNC,6B}（SP=RPM，ACC=0~255 档）｜停止{Addr,FE,98,SYNC,6B} */
#ifndef __ZDT_MOTOR_H
#define __ZDT_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#define ZDT_MOTOR_MAX   8     /* 数组下标按轴号 1..7 使用（0 号空置；手臂上下轴 = 7） */
#define ZDT_TAIL        0x6B

typedef enum {
    ZDT_BUS_A = 0,            /* USART2 : 轴 1,4 */
    ZDT_BUS_B = 1,            /* UART4  : 轴 3,6 */
    ZDT_BUS_C = 2,            /* UART5  : 轴 2(前后),7(上下) */
    ZDT_BUS_MAX
} ZdtBus_t;

void  ZDT_MotorInit(void);            /* 上电调一次：绑定串口、启动 IT 接收 */
void  ZDT_Poll(void);                 /* 周期调（如 10ms）：收帧超时复位，防状态机卡死 */

void  ZDT_ReadPos(uint8_t addr);                /* 请求位置；应答由 ISR 填 zdt_angle[] */

/* 0x9A 回零（无限位碰撞模式）：只发一次命令，完成与否由 StartTask 的固定延时处理 */
void  ZDT_Home(uint8_t addr);

/* 打包式：先 AddSpeed 攒子命令，再一次 Send(bus) 组 AA 帧发出并清空缓存 */
void    ZDT_MMCL_AddSpeed(ZdtBus_t bus, uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc);
uint8_t ZDT_MMCL_Send(ZdtBus_t bus);            /* 返回本次打包的子命令条数 */

void ZDT_PosCtrl(uint8_t addr, float deg, uint16_t speed_rpm, uint8_t acc);
/* 速度模式（0xF6）：dir 0=角度增大, 1=角度减小；rpm 0~3000；acc 0~255 */
void ZDT_SpeedCtrl(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc);

/* ISR 只写 zdt_raw + zdt_last_update(32 位原子写)；zdt_angle 由任务侧 ZDT_UpdateAngle() 换算 */
extern volatile int32_t  zdt_raw[ZDT_MOTOR_MAX];        /* 带符号原始脉冲(多圈) */
extern volatile uint32_t zdt_last_update[ZDT_MOTOR_MAX];/* 最后一次成功应答时刻(ms) */
extern volatile float    zdt_angle[ZDT_MOTOR_MAX];      /* 多圈角度(°)（任务侧刷新） */

void ZDT_UpdateAngle(uint8_t addr);   /* 任务侧：raw → zdt_angle（读位置后调用） */
bool ZDT_AngleFresh(uint8_t addr, uint32_t max_age_ms);  /* 位置数据是否在 max_age 内更新过 */
bool ZDT_HasAngle(uint8_t addr);      /* 上电以来该轴出过至少一帧应答：不看年龄、永不回假（差分基线用） */

#endif /* __ZDT_MOTOR_H */
