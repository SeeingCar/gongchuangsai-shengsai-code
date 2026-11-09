/* Follow.h —— 上位机“路径规划”帧接收 + 格点坐标表（与相机帧共用 USART3，见 zdt_motor.c 的 RX 分发）。
  * 帧格式（纯 ASCII）：'@' 任务号 '+' 格号 '(' 相对角 ')' '+' 格号 '(' 相对角 ')' … '%' 例：@1+21(90)+11(90)% → 任务 1：走到 21 号格并转 +90° →
  * 走到 11 号格并转 +90° @2+14(0)+12(0)% → 任务 2：走到 14（不转）→ 走到 12（不转） · 任务号 1~10，同一任务号只应出现一次（重发以最后一次为准，不一致会计数） · 格号
  * **1~25**（没有 0 号；7/9/17/19 这 4 个内圈格不用，出现即整帧判非法） · 相对角：加到**当前目标朝向**上的度数，逆时针为正，可带负号（-90）；0 = 不转 ·
  * 任务号后面的数**必须成对**（格号, 相对角），个数为奇数整帧丢弃 · 单条路线最多 FOLLOW_CELL_MAX 对 · 帧外的任何字符都忽略；'@' 重新开始，'%' 结束（')' 只是角度结束，不是帧尾）
  * 存放：path[任务号][第 i 对] = 格号，ang[任务号][第 i 对] = 相对角，path_n[任务号] = 对数。 收到 @1+21(90)+11(90)% ⇒ path[1][1]=21,
  * ang[1][1]=90, path[1][2]=11, ang[1][2]=90, path_n[1]=2 坐标：50 个宏（左 25 + 右 25，每个是一个点 {x,y}），启动时按 follow_side 拆进
  * x[26] / y[26]（格号直接当下标）。走到某格 = GO_To(x[格号], y[格号], 相对角)。 坐标是**任务系 cm**：原点 = Motion_Home/GO_Home 那一刻的停车点，+X =
  * 开跑车头方向，+Y = 左侧。 */
#ifndef __FOLLOW_H
#define __FOLLOW_H

#include <stdint.h>
/* 上位机路径规划总开关：1 = 收上位机发来的 @任务+格号(角)…% 帧（正式比赛接上位机用这个）；
 * 0 = 帧全部忽略，路线由 Follow_Set_L/_R 在启动时灌进去（台架 / 自立跑用）。
 * ⚠ 关掉又没调 Follow_Set_L/_R ⇒ path_n 全 0 ⇒ case -1 一直等。 */
#define FOLLOW_RX_EN       1   /* 2026-11-01：改由上位机发路线（摄像头串口 USART3），必须打开 */

#define FOLLOW_TASK_MAX    10      /* 任务号 1..10（0 号行空置） */
#define FOLLOW_CELL_MAX    10      /* 单条路线最多几个格号（0 号列空置） */
#define FOLLOW_CELL_NUM    26      /* 格号 1..25；数组开 26 只为让 x[25] 合法，0 号空着不用 */

/* ============================ 50 个格点宏（在这里填数） ============================ 每个宏 = 一个点 { x, y }，单位 cm；格号 1~25，没有 0 号。
 * CELL_Ln = 车**从左侧出发**时 n 号格的格心； CELL_Rn = 车**从右侧出发**时 n 号格的格心（两套互相独立，不是镜像）。 */
#define CELL_L1    {   20.0f,187.0f }
#define CELL_L2    {   20.0f,135.0f }
#define CELL_L3    {   20.0f,104.0f }
#define CELL_L4    {   20.0f,45.0f  }
#define CELL_L5    {   20.0f,19.0f  }
#define CELL_L6    {   45.0f,187.0f }
#define CELL_L8    {   45.0f,104.0f }
#define CELL_L10   {   45.0f,19.0f  }
#define CELL_L11   {  104.0f,187.0f }
#define CELL_L12   {  104.0f,135.0f }
#define CELL_L13   {  104.0f,104.0f }
#define CELL_L14   {  104.0f,45.0f  }
#define CELL_L15   {  104.0f,19.0f  }
#define CELL_L16   {  135.0f,187.0f }
#define CELL_L18   {  135.0f,104.0f }
#define CELL_L20   {  135.0f,19.0f  }
#define CELL_L21   {  196.0f,187.0f }
#define CELL_L22   {  196.0f,135.0f }
#define CELL_L23   {  196.0f,104.0f }
#define CELL_L24   {  196.0f,45.0f  }
#define CELL_L25   {  196.0f,19.0f  }

#define CELL_R1    {  187.5f,187.0f }
#define CELL_R2    {  135.0f,187.0f }
#define CELL_R3    {  105.5f,187.0f }
#define CELL_R4    {   45.0f,187.0f }
#define CELL_R5    {   20.0f,187.0f }
#define CELL_R6    {  187.5f,135.0f }
#define CELL_R8    {  105.5f,135.0f }
#define CELL_R10   {   20.0f,135.0f }
#define CELL_R11   {  187.5f,104.0f }
#define CELL_R12   {  135.0f,104.0f }
#define CELL_R13   {  105.5f,104.0f }
#define CELL_R14   {   45.0f,104.0f }
#define CELL_R15   {   20.0f,104.0f }
#define CELL_R16   {  187.5f,45.0f  }
#define CELL_R18   {  105.5f,45.0f  }
#define CELL_R20   {   20.0f,45.0f  }
#define CELL_R21   {  187.5f,11.0f  }
#define CELL_R22   {  135.0f,11.0f  }
#define CELL_R23   {  105.5f,11.0f  }
#define CELL_R24   {   45.0f,11.0f  }
#define CELL_R25   {   20.0f,11.0f  }

/* 出发方向变量：0 = 左（x[]/y[] 取 CELL_L*），1 = 右（取 CELL_R*）。 直接给这个变量赋值就行 —— 下一次 Follow_Walk() 会自动把新方向重新拆进 x[]/y[]， 不需要额外调用（想立刻生效就调 Follow_SetSide(0/1)）。 */
extern int follow_side;

/* 格点数组：格号直接当下标 —— GO_To(x[11], y[11], 0.0f)。 */
extern float x[FOLLOW_CELL_NUM];
extern float y[FOLLOW_CELL_NUM];

/* 收到的路线（成对存）：path[任务号][第 i 对] = 格号；ang[任务号][第 i 对] = 相对角； path_n[任务号] = 这条路线有几对。行/列都从 1 开始用（0 号空着），开 11 是因为 任务号能到 10、对数能到 10。
  * 例：收到 @1+21(90)+11(90)% ⇒ path[1][1]=21, ang[1][1]=90 path[1][2]=11, ang[1][2]=90, path_n[1]=2 走第一对 =
  * GO_To(x[path[1][1]], y[path[1][1]], ang[1][1])。 */
extern uint8_t path[FOLLOW_TASK_MAX + 1][FOLLOW_CELL_MAX + 1];
extern int16_t ang [FOLLOW_TASK_MAX + 1][FOLLOW_CELL_MAX + 1];
extern uint8_t path_n[FOLLOW_TASK_MAX + 1];

/* 诊断：上位机到底发进来什么（只读） follow_rx = 收到几个 '@' 帧头 follow_ok = 成功入表几帧 follow_bad = 解析到了但被判非法（格号 7/9/17/19、角度超
 * ±360、对数超限、任务号非法） follow_raw = 最近一帧的原文（含 '@' …
 * '%'），用来核对上位机发的格式 */
extern volatile uint32_t follow_rx, follow_ok, follow_bad;
extern char              follow_raw[64];

/* ⚠ 格号 7 / 9 / 17 / 19 **没有定义**（内圈 4 格不用，上位机不会发）： 收到的路线里只要出现这 4 个号，整帧判非法（直接丢弃），不会让车跑去 (0,0)。 */

/* ---------- 接收 ---------- */
void Follow_RxByte(uint8_t b);      /* ISR 里逐字节喂（USART3 的 RX 回调；只做整数） */
void Follow_Reset(void);            /* 清空所有路线（path_n/path 全 0） */

/* ---------- 出发方向 ---------- */
void    Follow_SideInit(void);      /* 上电调一次：按 follow_side 填 x[]/y[] */
void    Follow_SetSide(uint8_t right_start);   /* 改 follow_side 并立即重填 x[]/y[] */

/* ---------- 查数据 ---------- */
uint8_t  Follow_HasTask(uint8_t task_id);   /* 1 = 这个任务号收到过 */

/* 本地路线表（上位机不发时自己灌）：内容与 Follow.c 的 r1..r5 一致，灌完调 Follow_Walk(N) 就能依次走 */
void Follow_Set_R(void);   /* 右出发那套路线 */
void Follow_Set_L(void);   /* 左出发那套路线（任务 1 第一对 = 5 号格不转 → 15 号格转 -90）*/

/* ---------- 走一条路线 ---------- Follow_Walk(N) ↔ 任务号 N 的路线：逐对 (格号, 相对角) 下发 GO_To，到一对自动进下一对。
  * 非阻塞，在 case 里每 20ms 调一次即可： case A: Follow_Walk(2); if (Follow_WalkDone(2)) { Go_now = B; Go_sent = 0; }
  * break; 例：@1+21(90)+11(90)% ⇒ 调 Follow_Walk(1) 就是： GO_To(x[21], y[21], 90) → 到位后 GO_To(x[11], y[11], 90) → 走完。
  * 第一腿先朝当前车头直行、剩余 ≤20cm（GO_TURN_LATE_CM）才甩头 +90°；第二腿再甩 +90°（累计 180°）。 内部用的公开量，想改逻辑直接改 Follow.c 里的函数体： path_n[N] / path[N][i] /
  * ang[N][i] 第 i 对的格号与相对角（i 从 1 开始） x[格号] / y[格号] 该格的任务系坐标 cm GO_To(x[格号], y[格号], 相对角) 下发目标（相对角逆时针为正，0 = 不转）
  * GO_Arrived() 1 = 刚下发的目标已到位 ⚠ GO_To 的相对角是累加的，**每一对只能下发一次**（内部用 s_sent 保证）。 */
void Follow_Walk(uint8_t task);   /* 任务号 1~10；非法号/路线还没收到 → 什么都不做 */

uint8_t Follow_WalkDone(uint8_t task_id);   /* 1 = 这条路线走完（还没收到时返回 0） */
uint8_t Follow_WalkIdx(uint8_t task);   /* 诊断：当前走位指针 */
void    Follow_WalkReset(void);             /* 走位指针归零：之后再调 Follow_Walk 从第 1 对重来（不动接收到的数据） */

#endif /* __FOLLOW_H */
