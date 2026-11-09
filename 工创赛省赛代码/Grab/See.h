/* See.h —— 摄像头 / 扫码串口接收（USART3，PD8=TX / PD9=RX），一个串口两种帧，靠包头分流： ① 相机帧 $色!X!Y%（1~6 物料，6 = 浅蓝不再归一；
  * 7~9 圆环位编号） ② 扫码帧 #AAA+BBB+CCC+DDD&（三位十进制按位取色号） ⚠ 相机转 90° 装 ⇒ 画面 +x（右）= 车头方向：调 cam_cx 走底盘，调 cam_cy 伸收 FB。
 * 比例尺两个：cam_px_to_cm(30.7px/cm，给底盘)，cam_px_to_cm_y(33px/cm，给 M2)。 */
#ifndef __SEE_H
#define __SEE_H

#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "grab_cfg.h"      /* TRACK_* 判据与步长参数 */

#define SEE_RX_BUF_SIZE   64      /* 单帧最大长度 */
#define SEE_FRAME_MAX     8       /* 帧队列深度：8×64 = 512B RAM */

extern volatile int cam_color;      /* 1~6 物料颜色（6 = 浅蓝，不再归一） */
extern volatile int cam_cx;         /* 像素 X，中心 0；+x = 车头方向 */
extern volatile int cam_cy;         /* 像素 Y，中心 0 */
#define SEE_FRESH_MS   200               /* 超过这么久没有新帧 = 数据过期，不能当"看见"用 */
extern volatile uint32_t cam_last_ms;    /* 最后成功解析物料帧的时刻 */
extern volatile int cam_has_obj;    /* 1 = 收到物料帧 */
extern volatile int cam_target_id;  /* 追踪目标色（1~6）：case 2 与 A1 判停都按它过滤；由 Grab 抓起一块后推进到下一块 */
extern volatile float cam_px_to_cm; /* 像素→厘米比例尺（cx 方向）：前馈换算用 */
extern volatile float cam_px_to_cm_y;/* cy 方向的像素→厘米比例尺：M2 前馈用 */
/* 爪子中心在画面的 cx 像素位置（对准目标 / A1 判停横向条件） */
extern volatile float cam_grip_cx_px;
/* cx 极性开关：0 = 不翻（+cx = 车头方向），1 = 翻。只对收到的 cx 取负；目标点 cam_grip_cx_px 是独立常量，不跟着翻 */
extern volatile int cam_cx_invert;
/* cy 极性开关：0/1。新相机是 180° 安装，两轴符号都反 */
extern volatile int cam_cy_invert;
extern volatile int cam_ring_no;    /* 7~9 圆环位编号 */
extern volatile int cam_ring_cx;
extern volatile int cam_ring_cy;
extern volatile int cam_has_ring;   /* 1 = 收到圆环帧 */

/* 每解析出一帧物料帧 +1；必须基于真实新帧判定，否则"连续 N 帧稳定"形同虚设 */
extern volatile uint32_t cam_frame_seq;

/* 任意颜色物料帧（第一阶段找任意物料定基准用）：不受 cam_target_id 过滤。 */
extern volatile int      cam_any_color;    /* 1~6（6 = 浅蓝，不再归一） */
extern volatile uint32_t cam_any_seq;

uint8_t See_IsNewFrame(uint32_t *last_seq);        /* last_seq = 上次帧号；有新帧返回 1 并更新 */

/* 圆环帧序号：与 cam_frame_seq 分开（圆环帧不更新 cam_cx/cy），用法同上 */
extern volatile uint32_t cam_ring_seq;
uint8_t See_IsNewRingFrame(uint32_t *last_seq);

/* ⚠ 死区必须 >= 容差，否则物料已进容差电机还在微调 ⇒ "连续 N 帧稳定"永不成立 ⇒ 极限环；数值见 grab_cfg.h */


/* 当前帧物料是不是目标色。只用 cam_has_obj 会看到别的颜色也停，抓错料 */
uint8_t See_IsTarget(int target);
/* 3 = 浅蓝 / 6 = 深蓝 相机分不稳：扫码时两者没同时出现就视为同一颜色（See_UpdateBlueMerge 决定）*/
uint8_t See_SameColor(int a, int b);
extern const uint8_t g_blue_merge_key;   /* 定义在 freertos.c 的配置区：3/6 是否允许等价 */
void    See_UpdateBlueMerge(void);   /* 按 qr_c0[0..2] 重算上面的等价开关（写 qr_c0 之后调）*/

extern volatile int qr_c0[3];         /* v0 拆位：3 个抓取物料的颜色 */
extern volatile int qr_c1[3];         /* v1 拆位 */
extern volatile int qr_c2[3];         /* v2 拆位：**第二遍** 3 个抓取物料颜色 */
extern volatile int qr_c3[3];         /* v3 拆位：**第二遍** 3 个环号（1/2/3 已 +6 成 7/8/9）*/
/* 第 1 轮的颜色/环号**绑定快照**：Grab_UseRound2() 会把 qr_c0/qr_c1 换成 C/D 段， 但第二轮第二次放置仍要按"第 1 轮哪个颜色属于哪个环"来放，所以单独留一份。 */
extern volatile int qr_c0a[3];

/* 对准源切换：0 = 8 号圆环帧（case 5/6）；1 = 指定颜色的物料帧（case 11）*/
void    See_AlignSetSel(uint8_t sel, uint8_t color);
int     See_AlignCx(void);
int     See_AlignCy(void);
uint8_t See_AlignNew(uint32_t *last_seq);   /* 1 = 这一拍有新的有效目标帧 */
uint8_t See_AlignSeen(void);   /* 1 = 目标此刻在画面里（150ms 内有过帧）*/
uint8_t  See_AlignSelGet(void);   /* 当前对准源：0 = 圆环帧，1 = 选定颜色物料帧 */
uint8_t  See_AlignColorGet(void); /* 选定的物料颜色 1..6 */
uint32_t See_AlignSeqGet(void);   /* 选定颜色通道的帧计数（张数一直在涨 = 正在收到该颜色的帧）*/
extern volatile int qr_c1a[3];
extern volatile int qr_has_data;      /* 1 = 已锁定第一帧合法扫码帧；之后 QR 帧不再覆盖数组 */

/* 诊断：**相机**扫到的码（不管 Set() 有没有灌过数据，这里都一直更新）， 用来在串口里确认"到底扫上码没有"。
 * qr_cam_n = 收到几帧合法 #AAA+BBB+CCC+DDD& qr_cam[0..2] = 前三段拆位（物料颜色 1~6，6 = 浅蓝不再归一） qr_cam[3..5] = 后三段拆位（环号，1/2/3 已 +6 成 7/8/9） */
extern volatile uint32_t qr_cam_n;
extern volatile int      qr_cam[6];

/* 扫码帧解析开关：1 = 解析 #...&（码直接来自相机），0 = 停用（二维码只由 Set() 给）。 打开后串口那行 Q 就是扫到的码。 */
#define SEE_QR_PARSE_EN   1

/* 每路最近一次的 cx/cy（物料用颜色号 1~6；圆环用 环号-7），只给遥测看 */

/* USART3 单字节接收缓冲，zdt_motor.c 的回调拿它重挂接收 */
extern volatile uint8_t see_rx_byte;

void See_Init(void);        /* osKernelStart 之前调一次：建信号量 + 启动 USART3 单字节接收 */

/* 上位机路线握手：发出发边 → 等路线（封装）*/
void    See_SendSide(uint8_t side);   /* 发 '#2*'（左，side=0）/ '#1*'（右，side=1）给上位机 */
uint8_t See_RouteReceived(void);      /* 1 = 8 段路线（任务 1..8）全到齐 */
uint8_t See_PreFlowByte(uint8_t b);   /* 1 = 该字节已被握手消费（不喂相机解析）*/
void    See_PreFlowDone(void);        /* 握手结束：恢复相机/扫码解析 */

void See_RxISR(uint8_t byte);   /* 中断里逐字节调：拼包，成帧就入队 */

uint16_t See_WaitFrame(uint32_t timeout);   /* 取一帧，返回帧长；0 = 超时（osWaitForever 一直等） */

void See_ParseFrame(void);                  /* 抠包壳 → 按 ! / + 拆字段 → 填上面的结果变量 */

void     See_RecentColorsSince(uint8_t *out, uint8_t n, uint8_t skip, uint8_t skip2, uint32_t since_ms);  /* 只要 since_ms 之后的帧；skip/skip2 = 要剔除的颜色（0 = 不剔除）*/
bool     See_DataFresh(uint32_t max_age_ms);  /* 相机数据是否新鲜（最后成功帧距今 < max_age）*/

#endif /* __SEE_H */
