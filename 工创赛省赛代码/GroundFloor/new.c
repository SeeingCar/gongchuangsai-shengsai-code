/* new.c —— 新整体流程函数实现（替代 TaskFlow_Update() + GRAB_Update()）
 * 联合底盘 + 手臂状态机，完成：MoveCm → WaitArrive → Tracking → GrabJudge → PlaceOnServoM1 → Rotate90 → YMove → TrackTarget → PlaceDone
 */

#include <stdint.h>
#include <stdbool.h>
#include "new.c.h"
#include "chassis_cfg.h"     /* CTRL_DT_MS / MoveCm() */
#include "grab_cfg.h"        /* TRACK_* 参数 / cam_cx / cam_cy */
#include "M27.h"             /* M27_Arrived() / Grab6020_Arrived_Big() */
#include "See.h"             /* See_IsTarget() / cam_has_obj */


/* ========== 全局状态初始化（只读，freertos.c 或 StartTask 里写）========== */
volatile new_step_t g_new_step = NEW_STEP_MOVE_M2_IDLE;
volatile uint8_t    g_new_substep = 0;
volatile int        g_new_last_color = 0;


/* ========== API 实现 ========== */

void NewTaskFlow_Init(void)
{
    g_new_step       = NEW_STEP_MOVE_M2_IDLE;
    g_new_substep    = 0;
    g_new_last_color = 0;
}

void NewTaskFlow_Ready(void)
{
    /* 取初始位姿、等待数据就绪，类似原来的 TaskFlow_Ready() */
    Motion_Home();               /* 里程计归零（底盘） */
    cam_frame_seq = 0;           /* 清摄像头帧序列 */
}

void NewTaskFlow_Update(void)
{
    osDelay(20);                 /* 每 20ms 推进一次 */

    switch (g_new_step) {
        case NEW_STEP_MOVE_M2_IDLE:
            _NewStep_MoveM2Idle();
            break;
        case NEW_STEP_WAIT_ARRIVE:
            _NewStep_WaitArrive(&g_new_substep);
            break;
        case NEW_STEP_TRACKING:
            _NewStep_Tracking();
            break;
        case NEW_STEP_GRAB_JUDGE:
            _NewStep_GrabJudge();
            break;
        case NEW_STEP_PLACE_ON_SERVO_M1:
            _NewStep_PlaceOnServoM1();
            break;
        case NEW_STEP_ROTATE_90:
            _NewStep_Rotate90();
            break;
        case NEW_STEP_Y_LOW_SPEED_MOVE:
            _NewStep_YLowSpeedMove();
            break;
        case NEW_STEP_TRACKING_TARGET:
            _NewStep_TrackingTarget(cam_cx, cam_cy);
            break;
        case NEW_STEP_PLACE_DONE:
            _NewStep_PlaceDone();
            break;
    }

    /* 诊断用：串口打印当前状态 */
#if 0
    char buf[32];
    snprintf(buf, sizeof(buf), "new_step=%u,color=%d", (unsigned)g_new_step, g_new_last_color);
    USART0_WriteStr(buf);
#endif
}


/* ========== 内部状态机实现（底盘流程）========== */

/* case-1: MoveCm(90,0,3,900,100,0,900,100)，M2 原地不动，PD13 之前手臂不伸 */
static void _NewStep_MoveM2Idle(void)
{
    /* M2 原地不动：PD13 之前手臂不伸 */
    MoveCm(90.0f, 0.0f, 3, 900, 100, 0.0f, 900, 100);

    /* 移完后直接进入等待到达状态 */
    g_new_step = NEW_STEP_WAIT_ARRIVE;
    g_new_substep = 0;
}


/* case-0: 等待 M27_Arrived() && Grab6020_Arrived_Big()，最多 5s */
static void _NewStep_WaitArrive(uint32_t *p_wait_start)
{
    uint32_t t0 = HAL_GetTick();

    if (*p_wait_start == 0) {
        *p_wait_start = t0;
    }

    /* 等待条件：M27 已到达 && 6020 大臂已到位，且超时<5000ms */
    while ((!M27_Arrived() || !Grab6020_Arrived_Big()) && (HAL_GetTick() - t0) < 5000) {
        osDelay(20);
    }

    /* 等待完成后，开始追踪模式（不管目标颜色直接追） */
    g_new_step = NEW_STEP_TRACKING;
}


/* case-2: 追踪模式（不管目标颜色是什么就直接追） */
static void _NewStep_Tracking(void)
{
    /* 
     * 根据 cx/cy 进行追踪，这里需要调用底盘的 MoveCm 或类似的追踪逻辑
     * 假设底盘有函数 TrackToTarget(cam_cx, cam_cy)
     */
    
    if (!See_IsNewFrame(&cam_frame_seq)) {
        /* 没有新帧，保持当前姿态 */
        osDelay(20);
        return;
    }

    /* 不管目标颜色是什么，直接追 */
    TrackToTarget(cam_cx, cam_cy);

    /* 检查是否抓到料：cx<15 && cy<15 */
    if (cam_cx < 15.0f && cam_cy < 15.0f && See_IsNewFrame(&cam_frame_seq)) {
        g_new_last_color = cam_color;
        g_new_step = NEW_STEP_GRAB_JUDGE;
    }

    osDelay(20);
}


/* case-3: 抓取判定：cx<15 && cy<15 => 开始抓取 */
static void _NewStep_GrabJudge(void)
{
    /* 抓取判定窗口：连续 N 帧满足 cx<15 && cy<15 */
    
    if (!See_IsNewFrame(&cam_frame_seq)) {
        osDelay(20);
        return;
    }

    /* 如果仍然满足条件，执行抓取动作 */
    if (cam_cx < 15.0f && cam_cy < 15.0f) {
        Grab_Extend();           /* 手臂下降抓取 */
        osDelay(100);            /* 等待抓取到位 */
        Grab_Collect();          /* 吸取物料 */
        Grab_Retreat();          /* 手臂收回 */
        
        g_new_step = NEW_STEP_PLACE_ON_SERVO_M1;
    } else {
        /* 不满足条件，返回追踪模式 */
        g_new_step = NEW_STEP_TRACKING;
    }

    osDelay(20);
}


/* 放在"舵机物料 1"上面 */
static void _NewStep_PlaceOnServoM1(void)
{
    /* 
     * 移动底盘到舵机物料 1 的位置
     * 假设有一个函数 MoveToServoM1()
     */
    MoveToServoM1();

    /* 在舵机物料 1 上方放置物料 */
    Grab_PlaceOnServo();

    g_new_step = NEW_STEP_ROTATE_90;
}


/* 自转 90° */
static void _NewStep_Rotate90(void)
{
    /* M2 自转 90 度 */
    MoveCm(90.0f, 90.0f, 3, 0, 0, 90.0f, 0, 100);

    g_new_step = NEW_STEP_Y_LOW_SPEED_MOVE;
}


/* Y 方向低速平移 */
static void _NewStep_YLowSpeedMove(void)
{
    /* Y 方向低速平移，假设速度设为低速（例如 20cm/s）*/
    float y_speed = TRACK_Y_LOW_SPEED_CM_S;  /* 从 grab_cfg.h 获取低速参数 */

    MoveCm(90.0f, y_speed, 3, 
           TRACK_Y_MOVE_DIST_1, TRACK_Y_MOVE_DIST_2,
           90.0f, TRACK_Y_MOVE_DIST_3, TRACK_Y_STOP_TIME);

    g_new_step = NEW_STEP_TRACKING_TARGET;
}


/* 看见目标：根据 cx/cy 进行追踪 */
static void _NewStep_TrackingTarget(float cam_cx_in, float cam_cy_in)
{
    /* 根据当前画面中心的位置，计算需要移动的方向和距离 */
    
    if (!See_IsNewFrame(&cam_frame_seq)) {
        osDelay(20);
        return;
    }

    /* 
     * 执行放置动作（假设 Grab_Place() 是最终放置到圆环的动作）
     * 这里需要根据目标位置计算并移动到正确的位置
     */
    
    float target_x = cam_target_id > 0 ? TRACK_M2_PLACE_X : 0;
    float target_y = cam_target_id > 0 ? TRACK_M2_PLACE_Y : 0;

    /* 移动到放置位置 */
    MoveCm(target_x, target_y, 3, 
           TRACK_M2_GRAB_CM, TRACK_M2_GRAB_DIST,
           TRACK_M2_PRESURE_ANGLE, TRACK_M2_HOLD_TIME);

    /* 放置完成，进入最终状态 */
    g_new_step = NEW_STEP_PLACE_DONE;
}


/* 执行放置（最终把料放进圆环） */
static void _NewStep_PlaceDone(void)
{
    /* 
     * 这里应该：
     * 1. 确认物料已正确放入圆环
     * 2. 更新 grab_id_done 告诉手臂抓取完成
     * 3. 重置状态，准备下一次循环
     */
    
    if (Grab_Done()) {
        /* 如果手臂已经完成了三块料的抓取，停止流程 */
        tf_gate = 0;      /* 关闭底盘流程 */
        grab_gate = 0;    /* 关闭手臂流程 */
        
        /* 重置状态机到初始状态 */
        g_new_step = NEW_STEP_MOVE_M2_IDLE;
        g_new_substep = 0;
    }

    /* 这里可以添加额外的逻辑，例如：
     * - 更新二维码帧
     * - 切换下一轮（grab_round2 / grab_round3）
     */
}