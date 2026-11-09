/* Grab.h —— 手臂流程状态机（第二台，第一台是 TaskFlow.c）；⚠ case 里不能写 while 等待，会饿死另一台 */
#ifndef __GRAB_H
#define __GRAB_H

#include <stdbool.h>
#include <stdint.h>

void GRAB_Init(void);       /* 上电一次（osKernelStart 之前） */

/* 两个阀门：tf_gate=0 → 底盘停，grab_gate=0 → 手臂停；在 Grab.c 里定义，StartTask 定初值 */
extern int tf_gate;
extern int grab_gate;
extern int Grab_now;

/* 抓取顺序 0..2：索引扫码帧 qr_c0[]；两台共用这一份，抓完一个 +1，两层同时换目标色 */
extern int grab_id;

/* 手臂 → 底盘：1 = 这块料抓完了；底盘 case 3 原地等它，读完清 0（单消费者）*/
extern int grab_id_done;

/* 手臂 → 底盘：grab_allow=1 表示手臂 A1 已对准并停稳，请求抓取。 */
extern int grab_allow;
/* 底盘 → 手臂：ground_allow=1 表示底盘已停稳且安全线通过，允许进 case 3。 */
extern int ground_allow;
/* 底盘 → 手臂：1 = 底盘已在跟踪(case 2)。手臂判停必须等它，否则巡逻途中就提前抓 */
extern int tf_tracking;

/* 底盘 → 手臂：1 = 第二轮开始，手臂从 case 4 收工位进 case 5；⚠ 不能只靠 grab_gate 放开（底盘还在按 Grab_Done 等它，死锁）*/
extern int grab_round2;
/* 底盘 → 手臂：1 = **第二遍开始**（TF case 9 置 1；手臂 case 9 消费后清 0）*/
extern int grab_round3;

/* 底盘 → 手臂：1 = 圆环已对到画面中心（底盘 case 6/8 收敛）；手臂 case 6 收回 FB 后停住等它；单写者单读者 */
extern int ring_ok;

/* 手臂当前子步（底盘 case 5 用它等 5-0 摆姿态到位）*/
extern int grab_sub;
/* 1 = 三块料已抓完 ⇒ 串口停掉周期打印（保住抓取阶段的日志）*/
extern int grab_log_done;

/* case6 现算的 θ（°）= atan(15/M2₈) 与斜边 cm（诊断用，C 行）*/
extern float grab_k6_th;
extern float grab_k6_m2;
/* 上一次成功抓取那一刻的 M2 长度 cm（0=还没抓过）：
 *   < TF_M2_GRAB_CM ⇒ 底盘沿 −X 巡逻、手臂收到 3cm
 *   ≥ TF_M2_GRAB_CM ⇒ 底盘沿 +X 巡逻、手臂停在 8cm */
extern float grab_last_grab_cm;

void Grab_Start(void);      /* StartTask 的起点岔路走完后调：放开手臂状态机 */
/* 手动指定物料与放环顺序（覆盖扫码）：m1..m3 颜色 1~6，r1..r3 环序 1~3 */
/* 12 个参数：前 6 = 第一遍（3 色 + 3 环），后 6 = 第二遍（3 色 + 3 环）；环号可写 1/2/3 或 7/8/9 */
void Set(uint8_t m1, uint8_t m2, uint8_t m3,
         int r1, uint8_t r2, uint8_t r3,
         int m4, uint8_t m5, uint8_t m6,
         int r4, uint8_t r5, uint8_t r6);

void GRAB_Update(void);     /* 循环里每 20ms 一次：推进一步（阀门关着就直接返回） */

bool    Grab_Done(void);    /* 走完且真停在收工位（6020 180°、FB 1cm，见 Grab.c 4-0）；⚠ 不是"开始收工"：底盘 case 4 靠它保证"手臂先归位、再自转 180°" */
float   Grab_CalAng(uint8_t i); /* 标定：第 i 组 6020 角度 */
float   Grab_CalM5(uint8_t i);  /* 标定：第 i 组 M2 长度 */
uint8_t Grab_ChkFrame(uint8_t mat, uint8_t i);  /* 第 mat 块抓取判定窗口第 i 帧颜色 */

/* 切到"第二遍"：把 qr_c2/qr_c3 换进工作数组 qr_c0/qr_c1，并清第二遍该清的标志 （grab_id/grab_id_done/s_no_pick/s_place_i/s_pick_i/s_log_done 等）。
 * ⚠ 不动 s_cal_ang/s_cal_m5/s_last_grab_cm；基准(s_base_*)在底盘侧。 */
void Grab_UseRound2(void);

/* "上一次放进 8 号环的那块料"的颜色（查第 1 轮绑定 qr_c0a/qr_c1a）；0 = 没配好 */
uint8_t Grab_Ring8Color(void);
float   Grab_K6M2Of(uint8_t ring);   /* 7/9 号 M2 长度（带横向偏置模型），ring=7/8/9 */

/* 方向开关（串口 2/4 号）：1=正常 / 0=反向。其余调参项已删，用 grab_cfg.h 的宏 */
float   Grab_SetSign(uint8_t i, float v);   /* 1=case2 M2  2=case5 M2；1 正常 / 0 反向 */

#endif /* __GRAB_H */
