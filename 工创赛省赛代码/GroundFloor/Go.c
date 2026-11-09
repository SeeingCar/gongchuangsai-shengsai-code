#include "Go.h"
#include "pid.h"             /* PID_Calc */
#include "ins.h"             /* INS_OutToWheels */
#include "chassis_cfg.h"     /* GO_TOL_XY_CM / GO_TOL_YAW_DEG */
#include "grab_cfg.h"     /* GO_TOL_XY_CM / GO_TOL_YAW_DEG */
#include "mf_util.h"         /* MF_DEG2RAD / mf_ang_err */
#include <math.h>            /* cosf / sinf / sqrtf / fabsf */

static ChassisCtl_t      s_go;      /* 底盘就一个 */
static const InsState_t *s_pose;    /* 位姿只读借用：GO_Init 绑定一次 */
/* 同一个位姿对象的**可写**指针：只给 GO_SetTaskPos（对准已知格心后纠正里程计）用。 指向的对象是 freertos.c 里的普通 static，不是 const 对象，去掉 const 安全。 Go 的其它地方一律仍走 s_pose 只读。 */
static InsState_t       *s_pose_rw;

/* 相差距离 = 目标 − 当前（任务系目标先转世界系）；返回是否进死区 */
static uint8_t go_arrived(void)
{
    float c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0 = sinf(s_go.yaw0 * MF_DEG2RAD);

    s_go.ex   = s_go.tar_x * c0 - s_go.tar_y * s0 - s_pose->x;
    s_go.ey   = s_go.tar_x * s0 + s_go.tar_y * c0 - s_pose->y;
    s_go.eyaw = mf_ang_err(s_go.yaw0 + s_go.tar_yaw, s_pose->yaw);

    return (sqrtf(s_go.ex * s_go.ex + s_go.ey * s_go.ey) <= s_go.tol_xy)
        && (fabsf(s_go.eyaw) <= s_go.tol_yaw);
}

/* 分离式转弯入场：还差角度 ≥ GO_TURN_LATE_DEG 且距离 > GO_TURN_LATE_CM 才锁头直行，锁的是下目标这一刻的车头（未加本次 dyaw）；两条都不满足就边转边走。 */
static uint8_t go_split_start(float x, float y, float angle)
{
#if (GO_TURN_LATE_EN == 0)
    (void)x; (void)y; (void)angle;
    return 0;                                                         /* 关掉：连位姿都不看 */
#else
    float c0   = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0   = sinf(s_go.yaw0 * MF_DEG2RAD);
    float dx   = (x * c0 - y * s0) - s_pose->x;          /* 任务系目标 → 世界系，再减当前位姿 */
    float dy   = (x * s0 + y * c0) - s_pose->y;
    float need = mf_ang_err(s_go.yaw0 + angle, s_pose->yaw);          /* 还差多少度 */

    if (fabsf(need) <= GO_TURN_LATE_DEG)             return 0;
    if (sqrtf(dx * dx + dy * dy) <= GO_TURN_LATE_CM) return 0;

    s_go.lock_yaw = s_pose->yaw;                  /* 锁死，直行段不再跟随目标角 */
    return 1;
#endif
}

void GO_Init(const InsState_t *pose)
{
    s_pose    = pose;
    s_pose_rw = (InsState_t *)pose;    /* 同一个对象；只有 GO_SetTaskPos 通过它写 */

    s_go.ex = s_go.ey = s_go.eyaw = 0.0f;
    s_go.tar_x = s_go.tar_y = s_go.tar_yaw = 0.0f;
    s_go.tar_seq = s_go.arrived_seq = 0;
    s_go.arrived = 0;
    s_go.tol_xy  = GO_TOL_XY_CM;
    s_go.tol_yaw = GO_TOL_YAW_DEG;
    s_go.yaw0    = 0.0f;
    s_go.held    = 0;
    s_go.split   = 0;
    s_go.lock_yaw = 0.0f;
}

uint8_t GO_Home(void)
{
    GO_OdomBaseReset();          /* 四轮差分基准对齐到当前角度 */
    s_go.yaw0 = s_pose->yaw;

    /* ⚠ 目标也要清：否则上一场的目标立刻变成巨大误差，车会自己开出去 */
    s_go.tar_x = s_go.tar_y = s_go.tar_yaw = 0.0f;
    s_go.split = 0;              /* 直行段跟着作废（基准航向都换了） */
    s_go.ff_vx = s_go.ff_vy = 0.0f;
    return 1;
}

/* 只把目标 XY 归零：**不动 yaw0，也不动 tar_yaw**。 用于起点摆位后重新定原点；航向基准只允许上电 GO_Home() 锁一次。 */
void GO_ZeroXY(void)
{
    GO_OdomBaseReset();
    s_go.tar_x = s_go.tar_y = 0.0f;
    s_go.tar_seq++;              /* 新目标，别让 GO_Arrived 认旧序号 */
    s_go.held   = 0;
    s_go.split  = 0;
    s_go.ff_vx  = s_go.ff_vy = 0.0f;
}

/* 下目标唯一入口：allow_split=1 才吃分离式转弯（现在只有 GO() 传 1）。同一目标重复下发不刷序号、不重判分离， 否则直行段里被重复喂同一目标，会把锁住的航向刷成当前车头（永远放行不了）。 */
static void go_target_set(float x, float y, float angle, uint8_t allow_split)
{
    if (x != s_go.tar_x || y != s_go.tar_y || angle != s_go.tar_yaw) {
        s_go.tar_seq++;
        s_go.split = (allow_split && go_split_start(x, y, angle)) ? 1 : 0;
    }
    s_go.tar_x   = x;
    s_go.tar_y   = y;
    s_go.tar_yaw = angle;
    s_go.held    = 0;      /* 下了新目标 = 解除就地停住 */
    s_go.ff_vx = s_go.ff_vy = 0.0f;
}

/* 任务系**绝对**位置 + **相对**朝向（dyaw = 在"当前目标朝向"上再加多少度，逆时针为正）。
  * allow_split=1：和 GO() 同一套"先直行、剩余 ≤ GO_TURN_LATE_CM 才甩头"的分离式转弯； dyaw = 0 ⇒ need≈0 ⇒ go_split_start 返回 0，
  * 行为和"只改位置、不转"完全一样。 ⚠ 相对角是累加的：同一条腿**只能下发一次**。每拍重复调会把朝向一路叠上去（90→180→270…）。 */
void GO_To(float x, float y, float dyaw)
{
    go_target_set(x, y, s_go.tar_yaw + dyaw, 1);
}

/* 跟踪：目标 X 只比"当前实际 X"超 step，不接着上一目标累加。 这样每帧重发也不会让目标一直往前跑；Y/朝向保持当前目标。 */
void GO_TrackX(float step)
{
    float c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0 = sinf(s_go.yaw0 * MF_DEG2RAD);
    float x_now = s_pose->x * c0 + s_pose->y * s0;

    go_target_set(x_now + step, s_go.tar_y, s_go.tar_yaw, 0);
}

/* 跟踪：位置目标同 GO_TrackX，另外把**任务系 X 方向**的速度前馈给 PID（绕过 step 上限和 GO_TOL 死区）。 ⚠ PID 那边吃的是世界系速度，这里必须按 yaw0 旋转；否则车头不是 0° 时前馈会打偏。 */
void GO_TrackXFF(float step, float ff_vx)
{
    float c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0 = sinf(s_go.yaw0 * MF_DEG2RAD);
    float x_now = s_pose->x * c0 + s_pose->y * s0;

    go_target_set(x_now + step, s_go.tar_y, s_go.tar_yaw, 0);
    s_go.ff_vx = ff_vx * c0;          /* 任务 X → 世界系：ex 方向 (c0,s0) */
    s_go.ff_vy = ff_vx * s0;
}

/* 跟踪：目标 Y 只比"当前实际 Y"超 step（X/朝向保持当前目标）。 左出发时相机横向对应的是任务系 Y（两套坐标差 90°），所以要有这一版。 */
void GO_TrackY(float step)
{
    float c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0 = sinf(s_go.yaw0 * MF_DEG2RAD);
    float y_now = -s_pose->x * s0 + s_pose->y * c0;     /* 世界 → 任务系 Y */

    go_target_set(s_go.tar_x, y_now + step, s_go.tar_yaw, 0);
}

/* 同 GO_TrackY，另加**任务系 Y 方向**的速度前馈（内部按 yaw0 转到世界系）。 */
void GO_TrackYFF(float step, float ff_vy)
{
    float c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0 = sinf(s_go.yaw0 * MF_DEG2RAD);
    float y_now = -s_pose->x * s0 + s_pose->y * c0;

    go_target_set(s_go.tar_x, y_now + step, s_go.tar_yaw, 0);
    s_go.ff_vx = -ff_vy * s0;         /* 任务 Y → 世界系：ey 方向 (−s0, c0) */
    s_go.ff_vy =  ff_vy * c0;
}

/* 车在**任务系 Y** 上的速度 cm/s。 */
float GO_GetVy(void)
{
    float c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0 = sinf(s_go.yaw0 * MF_DEG2RAD);

    return -s_pose->world_vx * s0 + s_pose->world_vy * c0;
}

/* 车在**任务系 X** 上的速度 cm/s（跟踪前馈要用：物料速度 = 车速 + 误差变化率）。 */
float GO_GetVx(void)
{
    float c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0 = sinf(s_go.yaw0 * MF_DEG2RAD);

    return s_pose->world_vx * c0 + s_pose->world_vy * s0;
}

/* 相对目标（加到当前目标上），立刻返回：GO(60,0,0) 沿 +X 走 60cm，GO(0,0,90) 原地转 90°；平移 + 大角度 = 分离式转弯（判定在 go_split_start）。 */
void GO(float dx, float dy, float dyaw)
{
    go_target_set(s_go.tar_x + dx, s_go.tar_y + dy, s_go.tar_yaw + dyaw, 1);
}


/* 就地停住：目标冻在**当前位姿**（任务系），控制周期强制出零速度；目标序号 +1。 ⚠ 不调 GO_OdomBaseReset()：那是里程计基准，只属于开跑归零，Go 层顺手调会污染位姿。 */
void GO_Stop(void)
{
    float c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    float s0 = sinf(s_go.yaw0 * MF_DEG2RAD);

    s_go.tar_x   = s_pose->x * c0 + s_pose->y * s0;
    s_go.tar_y   = -s_pose->x * s0 + s_pose->y * c0;
    /* 目标航向是**任务系绝对角** = pose.yaw − yaw0。 原来写成 mf_ang_err(yaw0, pose.yaw) = yaw0 − pose.yaw，符号是反的！ 控制器要的是 yaw0 +
     * tar_yaw == pose.yaw，所以这里必须是 pose.yaw −
     * yaw0。 车头在任务系里是 0° 时两者相等，所以一直没暴露； 一旦车转过（比如 Follow_Walk 转了 −90°），GO_Stop 就会把目标设成 +90°， 航向误差变成 180° ⇒ **车原地自转半圈**。这就是第二轮的"自转 180°"。 */
    s_go.tar_yaw = mf_ang_err(s_pose->yaw, s_go.yaw0);
    s_go.tar_seq++;            /* 与旧目标区分开：GO_Arrived() 按号认领 */
    s_go.held    = 1;
    s_go.ff_vx = s_go.ff_vy = 0.0f;
}

uint8_t GO_Arrived(void)
{
    return (s_go.arrived && s_go.arrived_seq == s_go.tar_seq) ? 1 : 0;
}

/* 读当前目标点（任务系 cm）：追踪时目标一直在挪，别让调用者自己记 */
void GO_GetTar(float *x, float *y)
{
    *x = s_go.tar_x;
    *y = s_go.tar_y;
}

/* 当前位姿在任务系 X 上的投影（+X = 开跑车头方向）；不是世界系 X。 */
float GO_GetXTask(void)
{
    float c0, s0;

    if (!s_pose) return 0.0f;
    c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    s0 = sinf(s_go.yaw0 * MF_DEG2RAD);
    return s_pose->x * c0 + s_pose->y * s0;
}

/* 当前位姿在任务系 Y 上的投影（+Y = 开跑时左侧）；不是世界系 Y。 与 GO_GetXTask 同一套旋转（任务系坐标，供遥测/诊断用）。 */
float GO_GetYTask(void)
{
    float c0, s0;

    if (!s_pose) return 0.0f;
    c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    s0 = sinf(s_go.yaw0 * MF_DEG2RAD);
    return -s_pose->x * s0 + s_pose->y * c0;
}


/* 当前车头相对**任务系**的方位角 °（0 = 开跑车头方向，逆时针为正）。 遥测/测试看"转了多少度"用它；绝对 IMU 角没有意义（随上电位置变）。 */
float GO_GetYawTar(void)
{
    return s_go.tar_yaw;          /* 目标航向（任务系 °，只读诊断用）*/
}

/* 把里程计在**任务系**的坐标强行纠正到给定值（用于"对准已知格心 ⇒ 修正累积误差"）。 axis：1 = 纠正任务系 X，2 = 纠正任务系 Y（另一轴不动）。 只平移位姿：不改朝向、不改已下发的目标（调用方要在之后重新 GO_Stop()/GO_To() 建立目标）。
 * ⚠ 调用顺序很重要：先纠正位姿、再 GO_Stop()，这样"冻在当前位姿"用的就是纠正后的坐标。 */
void GO_SetTaskPos(uint8_t axis, float v)
{
    float c0, s0, now, d;

    if (!s_pose_rw) return;
    c0 = cosf(s_go.yaw0 * MF_DEG2RAD);
    s0 = sinf(s_go.yaw0 * MF_DEG2RAD);

    if (axis == 1) {
        now = s_pose->x * c0 + s_pose->y * s0;      /* 与 GO_GetXTask 同一套公式 */
        d   = v - now;
        s_pose_rw->x += d * c0;                     /* 任务 +X 在世界系 = (c0, s0) */
        s_pose_rw->y += d * s0;
    } else {
        now = -s_pose->x * s0 + s_pose->y * c0;     /* 与 GO_GetYTask 同一套公式 */
        d   = v - now;
        s_pose_rw->x += -d * s0;                    /* 任务 +Y 在世界系 = (-s0, c0) */
        s_pose_rw->y +=  d * c0;
    }
}

float GO_GetYawTask(void)
{
    if (!s_pose) return 0.0f;
    return mf_ang_err(s_pose->yaw, s_go.yaw0);   /* 任务系绝对角 = pose.yaw − yaw0（原来符号反了）*/
}



/* 蠕动档：1 = 之后的平移目标按蠕动速度跑，0 = 常规；只影响平移限速与刹车包线，不影响航向和已下发的目标。 */
void GO_Crawl(uint8_t on)
{
    PID_Crawl(on);
}

void GO_SetVmax(float v)
{
    PID_SetVmax(v);
}

void GO_ControlCycle(float *wheel_rpm)
{
    PidOut_t out;
    uint32_t seq = s_go.tar_seq;      /* 先记号再算：判定和号必须是同一拍的目标 */
    float    eyaw_ctrl;               /* 喂给 pid 的航向误差：直行段给的是锁住的那一头 */
    int  i;

    /* 就地停住：目标已冻在当前位姿，这里**直接出零**、不算 pid —— 只靠"目标=当前位置"的话，GO_Stop 到这一拍之间车已动了一点，残差会让 pid 留在原档（蠕动档就是一直慢爬）。顺带 PID_Clear。 */
    if (s_go.held) {
        s_go.ex = s_go.ey = s_go.eyaw = 0.0f;
        s_go.arrived = 1;
        s_go.arrived_seq = seq;
        PID_Clear();
        for (i = 0; i < 4; i++) wheel_rpm[i] = 0.0f;
        return;
    }

    s_go.arrived = go_arrived();

    /* 分离式转弯：直行段喂 pid 的是锁住的航向（误差≈0 ⇒ 车头不动），剩余距离缩到 GO_TURN_LATE_CM 才放行真目标角；⚠ 到场判定一直用真目标角。 */
    eyaw_ctrl = s_go.eyaw;
#if (GO_TURN_LATE_EN != 0)
    if (s_go.split) {
        if (s_go.arrived || sqrtf(s_go.ex * s_go.ex + s_go.ey * s_go.ey) <= GO_TURN_LATE_CM) {
            s_go.split = 0;                               /* 放行：这一段开始甩头 */
        } else {
            eyaw_ctrl = mf_ang_err(s_go.lock_yaw, s_pose->yaw);
        }
    }
#endif

    if (s_go.arrived) {               /* 到位 → 相差距离给 0，不做动 */
        s_go.ex = s_go.ey = s_go.eyaw = 0.0f;
        eyaw_ctrl = 0.0f;
    }
    s_go.arrived_seq = seq;

    PID_SetTrackFF(s_go.ff_vx, s_go.ff_vy);
    out = PID_Calc(s_go.ex, s_go.ey, eyaw_ctrl);    /* pid：相差距离 → out */
    INS_OutToWheels(s_pose->yaw, out, wheel_rpm);   /* ins：out → 四轮转速 */
}

/* 改到位死区（cm）。跟踪专用：默认 0.5cm，而跟踪一步可能只有 0.8cm 以上， 若把死区调大（如 1.2cm）会被 go_arrived() 当成"已到位" → 控制直接出 0 → 车永远不动（实测卡在 cx=-40）。 */
void GO_SetTol(float xy)
{
    if (xy < 0.05f) xy = 0.05f;
    if (xy > 5.0f)  xy = 5.0f;
    s_go.tol_xy = xy;
}
