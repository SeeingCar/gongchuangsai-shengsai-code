/* go_test.h —— 台架测试：世界坐标点到点 / 路径规划联调 */
#ifndef __GO_TEST_H
#define __GO_TEST_H

#include <stdint.h>

/* 世界坐标测试（阻塞，任务里调，s_run=1）：GO_To(60,0,+180) → GO_To(0,0,0)。 GO_To 是"绝对位置 + **相对**朝向"，与 GO() 同样吃分离式转弯（先直行、剩余 30cm 才甩头）。
 * 打印：W,步号,目标x,目标y,目标yaw,到达x,到达y,到达yaw,耗时ms,ok/TIMEOUT；末行 W,DONE,x,y,yaw。 */
void Go_Test_WorldRun(void);

/* 路径规划联调（阻塞在调用它的任务里，s_run=1 之后调）： ① 等上位机/雷达把路线发进来（以任务 1 有数据为准，最多等 30s，超时打 F,ERR,no-route）；
  * ② 依次把 1~10 号任务跑完：每个任务 while(!Follow_WalkDone(N)) Follow_Walk(N); 任务之间停 200ms； 上位机没发的任务号直接跳过； ③ 全部跑完 GO_Stop()。
  *  打印：F,任务号,耗时ms,ok/TIMEOUT；开始 F,BEGIN，结束 F,DONE。 ⚠ Follow_Walk 是**非阻塞**的：必须每 20ms 调一次直到 Done，不能"调一次然后延时"。 */
void Go_Test_FollowSeq(void);

/* 上位机路线测试（阻塞；不碰相机/手臂）： ① 等上位机发路线（最多 30s，超时 F,ERR,no-route）； ② 收到后把整张表打一遍 R,任务号,对数,格号,角,…； ③ 依次跑有数据的任务号，跑的时候每 1
 * 秒打一行 F,任务号,x,y,yaw,目标x,目标y； ④ 跑完 F,DONE。 */
void Go_Test_Route(void);

/* 先直行后横移（**非阻塞**，每 20ms 调一次，返回 1 = 还在走 / 0 = 已到位）：
 *   axis 1 = 先沿任务 X 直行、0 = 先沿任务 Y 直行；
 *   直行到离目标还剩 fork_cm，再把剩下这一段和另一轴一起补完。
 *   为什么要分段：X/Y 同时纠会留稳态误差，纯单轴段没有耦合。 */
uint8_t Go_Test_StraightThenSide(uint8_t axis, float tx, float ty, float fork_cm);
void    Go_Test_StraightThenSideReset(void);
uint8_t Go_Test_StraightThenSidePhase(void);   /* 诊断：0 = 没开始 / 1 = 直行段 / 2 = 补完段 */

/* 台架：**阻塞**走一遍「先直行后横移」到 (tx,ty)，跑完停住并每秒打最终位姿。
 *   axis 1 = 先沿任务 X 直行（右基准车头方向）、0 = 先沿任务 Y 直行。
 *   直接调用即可：Go_Test_StraightThenSideRun(1, 15.0f, 15.0f, GO_STRAIGHT_FORK_CM); */
void    Go_Test_StraightThenSideRun(uint8_t axis, float tx, float ty, float fork_cm);

/* 斜冲 + 直行收尾：斜走时所有轮都在滚（抓地好），最后一段纯直行退回目标（准）。
 *   (0,0)→(15,15) 实际走：斜冲到 (30,15) → 纯直行退回 (15,15)。
 *   非阻塞版：uint8_t Go_Test_DiagThenStraight(axis, tx, ty, over_cm)，每 20ms 调一次。
 *   over_cm = 直行轴多冲多少 cm：0 = 自动按"横移误差 × K"算；>0 指定多冲量；
 *             <0 = 不斜冲也不收尾，直接去 (tx,ty)（一段斜走，停在那儿）。
 *   斜冲点 = (tx + over, ty)，收尾退 over 回到 (tx,ty)。
 *   阻塞版：Go_Test_DiagThenStraightRun(axis, tx, ty) —— 跑完停住并每秒打位姿。 */
uint8_t Go_Test_DiagThenStraight(uint8_t axis, float tx, float ty, float over_cm);
void    Go_Test_DiagThenStraightReset(void);
uint8_t Go_Test_DiagThenStraightPhase(void);
void    Go_Test_DiagThenStraightRun(uint8_t axis, float tx, float ty);

/* 台架：从起点用「斜冲+直行收尾」走到左基准前置落点（go_test.c 里 GO_TS_PREGO_L_X/Y = 20,20），
 *   跑完停住并每秒打 D,STOP,<x>,<y>,<yaw>：拿卷尺量"是不是真的到了 20,20"。 */
void    Go_Test_PreGoLeftRun(void);

/* 台架：直给 GO_To(22, 11, 0)（左基准前置落点，go_test.c 里 GO_TS_GOTO_X/Y/ANG），不走斜冲+收尾。
 *   每 200ms 打 G,<x>,<y>,<yaw>,<arrived>；到位后每秒打 G,STOP,<x>,<y>,<yaw>。 */
void    Go_Test_GoToPreRun(void);

/* 台架：里程计标定 —— 沿车头方向直行 dist cm，停住后每秒打 INS 位姿。
 *   跑 Go_Test_OdoRun(1, 100) 量 X 方向、Go_Test_OdoRun(0, 100) 量 Y 方向；
 *   实物距离 d ⇒ 该方向的里程计尺度 = d / 100（<1 说明该方向打滑/多算）。
 *   顺便看 O,STOP 的 y 值：理想应为 0，若明显偏离说明车头基准角(yaw0)有偏差。 */
void    Go_Test_OdoRun(uint8_t axis, float dist);

#endif /* __GO_TEST_H */
