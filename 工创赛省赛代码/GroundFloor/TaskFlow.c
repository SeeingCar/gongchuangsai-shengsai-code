#include "TaskFlow.h"
#include "Go.h"
#include "grab.h"           
#include "grab_cfg.h"      
#include "See.h"            
#include "M27.h"            
#include "Follow.h"         
#include "usart.h"          
#include "cmsis_os.h"       
#include <math.h>           

int        Go_now  = -1;        /* 当前步 = case 号（用 int：前面几步是负数 -1 / 0） */
static int Go_sent = 0;         /* 本 case 的指令是否已发过 */
/* 当前步的起始时刻：case 1 的交接超时 / 200ms 停留、case 3 等手臂抓完、case 12 盲走兜底共用。
 * 同一时刻只有一个 case 在跑，所以一份就够；0 = 本步还没起算（用它的那几处要按 0 判断）。 */
static uint32_t s_step_t = 0;

static float    s_go_grab_pos = 0.0f;  /* 第一阶段巡逻中心（**跟踪轴**坐标：右=X / 左=Y）*/
static int      see_target = 0;  /* 1 = 已看到目标，正在刹停等交接 */
static int      s_go_hf_n    = 0;  /* 停稳确认计数（连续 2 拍速度够小）*/

/* 大循环遍数：0 = 第 1 遍（二维码 A/B 段），1 = 第 2 遍（C/D 段）；case 7 出口据此决定走 8 / 9 / 10。 */
int         big_round    = 0;
/* 抓取流程：第一次就找目标色，两轴都对准才定基准；定基准那一次只要 M2 < TF_M2_GRAB_CM 就当场抓走。
 * 这样省掉"先追任意物料量尺、再回头找目标色"整整一轮：第一块料常是非目标色，白追一趟还会按它的位置记错基准。
 * s_base_phase：1 = 找目标色巡逻（唯一入口），2 = 目标跟踪；0 已废弃。 */
static int  s_base_phase   = 1;
/* 基准采样（最多看两次对准）：shot0 未对准过；shot1 第一次 M2≥阈值只记 x1；shot2 已定基准 （第二次 M2<阈值就用这次的 x，两次都够不着就取较小 x）。
 *   注：这里的 x / x1 都是**跟踪轴坐标**（右基准 = 任务 X，左基准 = 任务 Y），不是任务 X 专有。*/
static int  s_base_shot    = 0;
static int  s_base_shot_prev = 0;       /* 本次判定之前的 s_base_shot（③b 用它区分"是不是第一次"）*/
static float    s_base_pos     = 0.0f; /* 基准点（**跟踪轴**坐标，右=X / 左=Y；定下后不再改）*/
static float    s_search_center = 0.0f;  /* 巡逻中心（抓完时的**跟踪轴**坐标：右=X / 左=Y）*/
/* 抓完后的巡逻：先原地停 200ms 再定向走。
 * s_search_dir：+1 / −1 = 沿跟踪轴正向 / 反向（由上次抓取的 M2 决定）；s_lock_dwell：1 = 停留已结束，初值 1 让其它入口保持老行为。 */
static float    s_search_dir   = -1.0f;
static int  s_lock_dwell  = 1;
/* 巡逻中"看见非目标物料就重定中心"的锁：一条腿最多一次，反向清零。否则车走到窗口边缘才看见物料，下一拍就反向，物料又跑出画面。 */
static int  s_lock_moved = 0;    /* 1 = 本腿已经重定过中心 */
/* 巡逻段"看见任意物料"的去抖状态（原 tf_seen_material 的 static，内联后提到这里）：
 * 只认"连续 2 帧同色、帧间隔 <400ms"，单帧误码/串色不算。 */
static uint32_t s_mat_seq = 0;     /* 已处理到第几帧 */
static int      s_mat_col = 0;     /* 上一帧的颜色 */
static int      s_mat_cnt = 0;     /* 连续同色帧数 */
static uint32_t s_mat_t   = 0;     /* 上一帧时刻 */

/* case 2 跟踪（底盘只管 cam_cx）；限幅见 grab_cfg.h */
static uint32_t s_go_fseq = 0;         /* 已处理到第几帧 */
static uint32_t s_go_prev_tick = 0;    /* 上一帧相机 tick，速度前馈用 */
static float    s_go_prev_err = 0.0f;  /* 上一帧 cam_cx，KD 的 D 用 */
static float    s_go_prev_raw = 0.0f;  /* 上一帧**原始** cam_cx，前馈的像素速度用 */
static float    s_go_rate_f   = 0.0f;  /* 平滑后的像素速度 px/s */
static float    s_go_vobj     = 0.0f;  /* 物料速度估计 cm/s（非对称平滑后的前馈值）*/
static float    s_go_cx_f = 0.0f;      /* cam_cx 低通，滤掉6020角度晃动 */
static int  	s_go_first = 0;        /* 1 = 本段第一帧，不算 D/前馈 */
static float    s_go_dbg_vff  = 0.0f;  /* 上一帧下发的底盘前馈速度 cm/s */
static uint32_t s_go_mute     = 0;     /* 上一步没到位前的静默截止时刻（到位会提前放行）*/
static uint32_t s_go_frame_t  = 0;     /* 最近一次目标色新帧时刻（>250ms 判丢帧）*/
static int  Go_sub  = 0;         /* 子步骤号（case -1/4/7/10） */
/* case 2 的增益 / 死区 / 步长上限直接用 grab_cfg.h 的宏（TRACK_K_CH、TRACK_KD_CH、TRACK_FF_CH、
 * TRACK_TOL_PX、CH_TRACK_STEP_MAX_CM），这里不再存一份副本。 */

/* case 2 底盘追踪方向：-1 = 默认（实测方向），+1 = 反过来。串口命令 1 可在线改（见 TaskFlow_SetSign）。
 * 只作用于 case 2 步长，不改 case 5 对准与相机极性。 */
static float    s_t1_sgn = -1.0f;
/* case 5 底盘对准方向：第一轮 / 第二轮各一个，默认都是 -1；串口命令 3 / 5 可在线改。 */
static float    s_r5_sx_sgn = -1.0f;
static float    s_r9_sx_sgn = -1.0f;

uint8_t TaskFlow_BaseValid(void)          /* 1 = 基准已定（两次采样结束）*/
{
    if (s_base_shot >= 2) return 1;
    return 0;
}

float TaskFlow_SetSign(uint8_t i, float v)
{
    float sg;

    if (v >= 0.5f) sg = 1.0f;
    else           sg = -1.0f;

    if      (i == 1) s_t1_sgn    = sg;   /* case2 底盘   */
    else if (i == 2) s_r5_sx_sgn = sg;   /* case5 第一轮 */
    else              s_r9_sx_sgn = sg;   /* case5 第二轮 */

    if (sg > 0.0f) return 1.0f;
    return 0.0f;
}

/* case 5 找环状态 */
static int      s_go5_align  = 0;      /* 1 = 已看到 8 号环，正在底盘 2D 对准 */
static float        s_go5_search_center = 0.0f; /* 找环巡逻中心（跟踪轴坐标）*/
static int      s_go5_search_i = 0;    /* 0=+10, 1=-10 */
/* 圆环轮次：0 = 第一轮（case 5 用跟随轴），1 = 第二轮（用反轴）；两轮共用同一套 case 5/6/7/8，与手臂的 case 一一对应。 */
int             g_c12_ready  = 0;   /* 1 = 底盘已到 8 号环已知格心（case11_key=0 时给手臂放行）*/
/* 8 号环对准完成那一刻的位姿（已纠偏）：case11_key=0 时 case 12 直接开回这里 */
static float        s_ring8_px    = 0.0f;
static float        s_ring8_py    = 0.0f;
static int      s_ring8_has   = 0;     /* 1 = 已经记过一次 */
static int      s_ring_round  = 0;

/* 圆环对准状态：底盘只纠 cx（cam_ring_cx），cy 交给手臂 M2 */
static uint32_t     s_go6_fseq   = 0;
static int      s_go6_tol_n  = 0;      /* 连续多少帧两轴都在死区内 */
static int      s_lock_wait   = 0;      /* 1 = 等下一帧 */
static uint32_t     s_go6_prev_t  = 0;     /* 上一帧 ring 时刻 */

/* 进入/恢复"找目标色"巡逻：设中心 + 起步方向，先原地停 200ms。起步方向：上次抓取 M2 < TF_M2_GRAB_CM(9cm) ⇒ 先正，否则先负（从未抓过也算近 ⇒ 先正）。 */
static void To_Find_color(float center)
{
    s_search_center = center;
    if (grab_last_grab_cm < TF_M2_GRAB_CM) s_search_dir = +1.0f;
    else                                   s_search_dir = -1.0f;
    Go_sent   = 0;
    s_step_t    = HAL_GetTick();/* 停留 200ms 从这里起算 */
    s_lock_dwell   = 0;
    s_lock_moved  = 0;
}

/* 底盘"跟踪轴"跟着出发边走（两套坐标差 90°）：右出发 follow_side=1 → 相机横向 cx 对应任务系 X（车头前后）→ 动 X；
 * 左出发 follow_side=0 → cx 对应任务系 Y（横移）→ 动 Y；纵向 cy 一律交给 Grab 的 M2。 */
static void tf_track_ff(float step, float ff)
{
    if (follow_side) GO_TrackXFF(step, ff);
    else             GO_TrackYFF(step, ff);
}

static void tf_track_hold(void)
{
    if (follow_side) GO_TrackX(0.0f);
    else             GO_TrackY(0.0f);
}

/* 车在跟踪轴上的速度 cm/s（右出发 = 任务 X，左出发 = 任务 Y）。交接刹停判据、物料速度前馈、Grab 判基准都要用，所以对外公开。 */
float TaskFlow_CarV(void)
{
    if (follow_side) return GO_GetVx();
    return GO_GetVy();
}

static float tf_car_v(void) { return TaskFlow_CarV(); }

/* 显式指定轴：圆环对准用（case 5 走跟随轴、case 11 走反轴）*/
static void tf_go_pos_ax(float pos, uint8_t use_x)
{
    float tx, ty;

    GO_GetTar(&tx, &ty);
    if (use_x) GO_To(pos, ty, 0.0f);
    else       GO_To(tx, pos, 0.0f);
}

static void tf_car_rel_ax(float d, uint8_t use_x)
{
    if (use_x) GO(d, 0.0f, 0.0f);
    else       GO(0.0f, d, 0.0f);
}

/* 车在跟踪轴上的位置（右出发 = 任务 X，左出发 = 任务 Y）。巡逻中心、基准点、判抓取都只认这一个坐标。 */
static float tf_car_pos(void)
{
    if (follow_side) return GO_GetXTask();
    return GO_GetYTask();
}

/* 沿跟踪轴把目标挪到 pos，另一轴与朝向保持当前目标（右出发只动 X、左出发只动 Y）。 */
static void tf_go_pos(float pos)
{
    float tx, ty;

    GO_GetTar(&tx, &ty);
    if (follow_side) GO_To(pos, ty, 0.0f);
    else             GO_To(tx, pos, 0.0f);
}

/* 进 case 5 前的初始化：清对准状态 */
static void tf_ring_cycle_start(void)
{
    s_go5_align = 0;
    s_go5_search_center = tf_car_pos();
    s_go5_search_i = 0;
    s_go6_tol_n = 0;
    s_lock_wait  = 0;
    s_go6_fseq  = cam_ring_seq;
}

/* case 2 每块料都重进：帧号从当前起算；λ 固定，不跟目标运动学（否则会把盘的运动学进去）。 */
static void tf_track_prep(void)
{
    uint32_t now = HAL_GetTick();

    s_go_fseq     = cam_frame_seq;       /* 从这帧起算新帧 */
    s_go_frame_t  = now;                 /* 丢帧看门狗从这一刻起算，别刚进来就误判 */
    s_go_prev_tick = now;
    s_go_prev_err = (float)cam_cx;       /* 第一帧 D=0，别把已有偏差当突变 */
    s_go_prev_raw = (float)cam_cx;
    s_go_cx_f     = (float)cam_cx;
    s_go_rate_f   = 0.0f;                /* 新一段从静止建速度，别带上一段的残留 */
    s_go_vobj     = 0.0f;
    s_go_dbg_vff  = 0.0f;
    s_go_first    = 1;
    tf_track_hold();                     /* 先冻住扫描目标，从当前实际位置开始追 */
}

void TaskFlow_Init(void)
{
    /* 第一阶段的巡逻点在 case 0 到位后按实际 X 建；这里只清本流程状态。 */
    s_go_mute    = 0;
    s_ring_round = 0;
}


void TaskFlow_Ready(void)
{
    while (!Motion_Home()) {                  /* 返回 0 = 底层数据还没就绪 */
        osDelay(CTRL_DT_MS);          /* 必须让出 CPU：MotorTx 优先级更低 */
    }
    Go_now  = -1;                           /* 全流程从 case -1（任务 1 路线 + 扫码 + 去 15 号格）开始 */
    Go_sent = 0;
    Go_sub  = 0;                            /* case -1 从"还在走任务 1 路线"起步 */
    /* 起点/巡逻阶段用正常到位死区；进入 case 2 交接时再压到 0.05cm。 */
    GO_SetTol(GO_TOL_XY_CM);

    /* grab0 = 1（单独测抓取）：跳过 case -1 / case 0，直接进 case 1 巡逻抓料。
     * 下面这段初始化照抄 case 0 结尾那几句，否则巡逻中心和基准都是空的。*/
    if (g_grab0 != 0) {
        s_go_grab_pos = tf_car_pos();
        s_base_phase  = 1;                  /* 1 = 进入巡逻 */
        s_base_shot   = 0;
        s_base_pos    = 0.0f;
        To_Find_color(s_go_grab_pos);
        Go_now = 1;
    }

    /* base_en = 0（直接抓）：基准清零 + 视为"已定" ⇒ 判停从一开始就用抓取门 */
    if (g_base_en == 0) {
        s_base_pos  = 0.0f;
        s_base_shot = 2;
    }
}
/* 进 case 3：冻住底盘 + 恢复默认死区（跟踪用的 0.05 别带过去）*/
static void tf_enter_grab(float x)
{
    s_go_grab_pos = x;
    tf_tracking = 0;
    GO_SetTol(GO_TOL_XY_CM);
    GO_Stop();
    Go_sent = 0;
    s_step_t  = 0;                  /* case 3 用 0 表示"还没开始等"，进来先归零 */
    Go_now  = 3;
    /* grab_id_done 是电平标志，唯一消费者是 case 3（读到就清）。若上一轮抓手置了 1、底盘却已超时回巡逻走了，
     * 这个 1 会锁存下来：下次进 case 3 第一拍就当成"已经抓完"直接开走，手臂还在原地合爪。所以在这里清掉。 */
    grab_id_done = 0;
    grab_allow = 1;                 /* 手臂 A1：已对准并停稳 */
    ground_allow = 1;                /* 底盘：已停稳并允许抓取 */
}
/* 回 case 1：重新按目标色巡逻。基准只在追到目标色并两轴对准时才记（见 case 2），这里不管基准。 */
static void tf_resume_patrol(void)
{
    Go_sent = 0;
    ground_allow = 0;
    grab_allow  = 0;      /* 上一轮握手已结束，避免下一轮跟踪误用旧标志 */
    s_go_grab_pos  = tf_car_pos();
    s_base_phase   = 1;
    GO_SetTol(GO_TOL_XY_CM);      /* 回巡逻一律用常规死区，别把跟踪的 0.05 带过来 */
    To_Find_color(s_go_grab_pos);
    Go_now = 1;
}
/* 圆环对准（case 5/11 共用）后更新坐标里程计 */
/* 8 号环对准后纠正到的已知格心坐标 cm：左右基准各一套 X/Y（右基准的 Y 单独标成 107）*/
static float tf_ring8_fix_x(void)
{
    if (follow_side) return RING8_FIX_X_CM_R;
    return RING8_FIX_X_CM_L;
}

static float tf_ring8_fix_y(void)
{
    if (follow_side) return RING8_FIX_Y_CM_R;
    return RING8_FIX_Y_CM_L;
}

static void tf_ring8_fix(int use_x)
{
    float bx = GO_GetXTask();
    float by = GO_GetYTask();

    /* 纠的是实际对准的那根轴（use_x），不能按 swap 判：右出发 use_x=(swap?0:1)，左出发 use_x=(swap?1:0)。
     * 原来按 swap 判，左出发时会纠错轴（对的是 Y 却纠 X）。 */
    if (use_x) GO_SetTaskPos(1, tf_ring8_fix_x());
    else       GO_SetTaskPos(2, tf_ring8_fix_y());

    /* 坐标一改，之前记下的环 8 物理点要跟着平移，否则它指向的就变了 */
    if (s_ring8_has) {
        s_ring8_px += GO_GetXTask() - bx;
        s_ring8_py += GO_GetYTask() - by;
    }
}
//对准8号环并更新里程计
static void tf_ring_step(uint8_t swap)
{
    static int s_last_swap = 0xFF;
    int use_x;
	//右基准
    if (follow_side) {
        if (swap == 0) use_x = 1; //x
        else           use_x = 0;}//y 
	//左基准
	else {
        if (swap == 0) use_x = 0; //y
        else           use_x = 1;}//x
    /* 换轴进来第一次调用时，将相关变量进行重置 */
    if (swap != s_last_swap) {
        s_last_swap         = swap;						//记住这一轮用哪根轴
        if (use_x) s_go5_search_center = GO_GetXTask();	//根据不同的基准取巡逻中心x或y
        else       s_go5_search_center = GO_GetYTask();	
        s_go5_search_i      = 0;						//巡逻走到哪个端点（0 = +10cm，1 = −10cm）
        s_go5_align         = 0;						//0 = 还在巡逻找环，1 = 已看到环、正在对准
        Go_sent             = 0;           				//"这一步的指令已发过"
        s_go6_tol_n         = 0;  						//连续几帧两轴都在死区内
        s_lock_wait         = 0;						//"上一小步还没走完，别叠新目标"
        s_go6_fseq          = cam_ring_seq;				//已处理到哪一帧圆环
        s_go6_prev_t        = HAL_GetTick();   			//最后一次看到环的时刻
        GO_SetTol(GO_TOL_XY_CM);                   		//移动死区0.5cm
    }
	//没看见环的情况巡逻找环
    if (!s_go5_align) {
		//"巡逻找环" 切换到 "已看到环、开始对准" 的交接
        if (See_AlignSeen()) {				//8 号环在最近 150ms 内出现过
            GO_Stop();						//冻住当前目标、速度归零
            s_go5_align = 1;				//切到"对准"状态
            s_go6_tol_n = 0;				//清"连续到位帧数"
            s_lock_wait  = 0;				//允许发第一小步
            s_go6_fseq  = cam_ring_seq;		//把当前帧标记为"已处理"
            s_go6_prev_t = HAL_GetTick();	//记录当前时间
            GO_SetTol(GO_TOL_XY_RING_CM);	//死区 0.5 → 0.2cm
            return;
        }
        /* 等手臂把 5-0（6020→90°、M2→5cm、UD→0）摆完再巡逻 */
        if ((Grab_now == 5) && (grab_sub == 0)) {
            GO_Stop();                                    
            Go_sent = 0;
            return;
        }
		//±10cm 巡逻找 8 
        if (!Go_sent) {                                
            GO_SetTol(GO_TOL_XY_CM);
            GO_Crawl(1);
            float back;

            if (s_go5_search_i == 0) back = 10.0f;
            else                     back = -10.0f;
            tf_go_pos_ax(s_go5_search_center + back, use_x);
            Go_sent = 1;
        } else if (GO_Arrived()) {
            if (s_go5_search_i == 0) s_go5_search_i = 1;
            else                     s_go5_search_i = 0;
            Go_sent = 0;
        }
        return;
    }
    /* 已看到 8 号环：底盘对准 cx（走 use_x 指定的轴），cy 交给 Grab 的 M2 */
    if (See_AlignNew(&s_go6_fseq) && See_AlignSeen()) {
        int ex  = See_AlignCx() - (int)RING_CX_TGT_PX_8;  /* 目标点 -11px，原始误差 */
        int ey  = See_AlignCy() - (int)RING_CY_TGT_PX_8;  /* 目标点 -21px，原始误差 */
        int aex = (int)fabsf((float)ex);				/* x轴误差绝对值 */
        int aey = (int)fabsf((float)ey);				/* y轴误差绝对值 */

        /* 对准判据：同一帧 cx、cy 都在各自 8px 死区内，连续 5 帧才算完成，任一帧掉出死区就清零重数。*/
        if ((aex <= RING_K6_TOL_PX) && (aey <= RING_K6_TOL_PX)) {
            s_go6_tol_n++;
            if (s_go6_tol_n >= RING_K6_FRAMES) {          /* 两轴5帧都到位 → 进 case 6 */
                /* 对准完成，车此刻就在 8 号环已知格心上，更新里程计 */                
                tf_ring8_fix(use_x);
                /* 只在"第 1 遍第 2 次 case 5（swap=1）"这一刻记录：用户确认这才是 8 号环的正确位姿（Y 已被纠成 105、X 是当时的里程计值）。
                 * 第 2 遍的 case 5 会纠 X，不能拿它当基准。X≈190.4 / Y≈105.0 是这一次记录的现场实测。 */
                if ((swap == 1) && (big_round == 0)) {
                    s_ring8_px  = GO_GetXTask();
                    s_ring8_py  = GO_GetYTask();
                    s_ring8_has = 1;
                }
                GO_Stop();
                ring_ok     = 1;
                s_go5_align = 0;
                Go_sent   = 0;
                /* case5→6 标定；case11→12 第二次放置 */
                if (Go_now == 11) Go_now = 12;
                else              Go_now = 6;
                return;
            }
        } 
		//不符合标准，计数清0
		else {s_go6_tol_n = 0;}
		//看见并追踪
        if (!s_lock_wait) {
            //根据误差定kp的值
			float kp_x;
            if (fabsf((float)See_AlignCx()) > 60.0f) kp_x = K5_K_FAR;
            else                                     kp_x = K5_K_NEAR;
            /* 符号按"出发边 + 轮次"选：左基准下 cx 驱动的轴与右基准相反，所以两轮各用一套（右：−1/+1；左：−1/−1）。 */
            float sg;

            if (swap == 0) {
                if (follow_side) sg = K5_SX_SIGN_R;
                else             sg = K5_SX_SIGN_L;
                sg = sg * s_r5_sx_sgn;
            } else {
                if (follow_side) sg = K9_SX_SIGN_R;
                else             sg = K9_SX_SIGN_L;
                sg = sg * s_r9_sx_sgn;
            }
            float sx   = sg * kp_x * (float)ex;

            if (sx >  CH_TRACK2_STEP_MAX_CM) sx =  CH_TRACK2_STEP_MAX_CM;
            if (sx < -CH_TRACK2_STEP_MAX_CM) sx = -CH_TRACK2_STEP_MAX_CM;
            GO_Crawl(1);
            GO_SetTol(GO_TOL_XY_RING_CM);
            tf_car_rel_ax(-sx, use_x);
            s_lock_wait = 1;
			} 
			//打开移动锁，允许下一步的运动
			else if (GO_Arrived()) {s_lock_wait = 0;}
			//记录下当前看见8号时的时间
			s_go6_prev_t = HAL_GetTick();
	} 
		/* 环丢了 */
		else{                                              
			uint32_t age = HAL_GetTick() - s_go6_prev_t;
			if (age > 300) GO_Stop();
			if (age > CH_LOST_RETURN_MS) {
				s_go5_align  = 0;
				s_lock_wait  = 0;
				GO_Stop();
				}
			}
}

/* 巡逻摆幅：还没抓到料时用宽的 TF_PATROL_SWING_CM；抓过一块之后材料少了，收窄到 TF_PATROL_SWING_AFTER_CM。 */
static float tf_patrol_swing(void)
{
    if (grab_id >= 1) return TF_PATROL_SWING_AFTER_CM;
    return TF_PATROL_SWING_CM;
}

/* 每 20ms 推进一步 */
void TaskFlow_Update(void)
{
    /* tf_gate 只由 StartTask 置 1；本函数里置 1 = 死锁 */
    if (!tf_gate) return;
	
    switch (Go_now) {
	/* 出发段：任务 1 路线 → 扫到码就去 15 号格；没扫到就在原地来回扫 */	
    case -1:  
		/*走任务1的路段*/
        if (Go_sub == 0) {                     /* 0 = 还在走任务 1 路线 */
            Follow_Walk(1);
            if (!Follow_WalkDone(1)) break;     /* 路线还没走完 */
			GO_Crawl(1);						//进入蠕动档
            GO_Stop();                          /* 停住：GO 是相对当前目标的，从这里开始加 */						
            Go_sub  = 1;
        }
		/*如果没有扫码，执行巡逻找码逻辑*/
        if (!qr_has_data) {
            if (!Go_sent) {
                switch (Go_sub) {
                case 1:
                    if (follow_side) GO( 0.0f,  12.0f,0.0f); 	/*右边，左移10*/
                    else             GO( -12.0f, 0.0f, 0.0f);	/*左边，左移10*/
                    Go_sent = 1;
                    break;
                case 2:
                    if (follow_side) GO(0.0f, -12.0f, 0.0f);	/*右边，右移10*/
                    else             GO(12.0f, 0.0f, 0.0f); 	/*左边，右移10*/
                    Go_sent = 1;
                    break;
                case 3:
                    if (follow_side) GO(0.0f, -12.0f, 0.0f);	/*右边，右移10*/
                    else             GO(12.0f, 0.0f, 0.0f);		/*左边，右移10*/
                    Go_sent = 1;
                    break;
                case 4:
                    if (follow_side) GO( 0.0f,  12.0f,0.0f);	/*右边，左移10*/
                    else             GO(-12.0f, 0.0f, 0.0f);	/*左边，左移10*/
                    Go_sent = 1;
                    break;
                }
            } else if (GO_Arrived()) {
                Go_sub++;                     /* 下一步 */
                if (Go_sub > 4) Go_sub = 1; /* 第 4 步之后回到第 1 步 */
                Go_sent = 0;
            }
            break;
        }
        /* 扫上了：去 15 号格（原 case -1 的动作），到位才进 case 0 */
        GO_To(x[15], y[15], 0.0f);    /* dyaw = 0 ⇒ 重复下发同一目标不刷序号、不累加角度，可以每拍发 */
        if (GO_Arrived()) {
            Go_sent = 0;
            Go_now  = 0;
        }
        break;
	/* 第二段：跑任务 2 的路线；走完把阀门交给手臂（tf_gate=0、grab_gate=1）*/
    case 0:  
		GO_Crawl(0);						//退出蠕动档
        Follow_Walk(2);
        if (!Follow_WalkDone(2)) break;
		/* 将后续需要的变量初始化 */
        grab_gate = 1;tf_gate   = 0;                
        s_go_grab_pos  = tf_car_pos();
		s_base_phase   = 1;					/* 置1为进入巡逻 */
        s_base_shot    = (g_base_en == 0) ? 2 : 0;   /* 基准重新采样；base_en=0 时直接视为"已定"（第 1 块就用紧门）*/
        s_base_pos     = 0.0f;
        To_Find_color(s_go_grab_pos);
        Go_now  = 1;
        break;
    /* 第三段，巡逻，看见目标后进入追踪 */
    case 1: 
        if (see_target) {
			/*如果目标丢了或超时还没有停稳就重新巡逻*/
            if (!See_IsTarget(cam_target_id) ||((HAL_GetTick() - s_step_t) > 2000)) {
                see_target    = 0;			/* 退出交接状态 */
                s_go_hf_n     = 0;			/* 清停稳确认计数 */
                s_step_t      = HAL_GetTick();  /* 交接取消：回去巡逻要重新起算停留 200ms */
                Go_sent     = 0;          /* 让巡逻/搜索重新发一个点 */
                GO_Stop();
                break;
            }
			/*交接成功的出口，车确实停稳了，可以开始跟踪*/	
            if (fabsf(tf_car_v()) < 2.0f) {
                s_go_hf_n++;
                if (s_go_hf_n >= 2) {
                    GO_Crawl(1);             /* 追踪保持蠕动档：12cm/s、刹车 40cm/s² */
                    GO_SetTol(GO_TOL_XY_FINE_CM);        /* 到位死区变低，防止蠕动被死区卡死 */
                    tf_track_prep();		 /*	跟踪初始化 */
                    s_base_phase = 2;        /* 真正进跟踪才置 2；取消交接时保持 1，才能回去巡逻 */
                    see_target   = 0;        /* 退出交接、换 case2 */
                    s_go_hf_n    = 0;        /* 退出交接、换 case2 */
                    Go_now     = 2;        /* 退出交接、换 case2 */
                }
            } else {
                s_go_hf_n = 0;
            }
            break;
        }
        /* 没看见目标，巡逻 */
        if (s_base_phase == 1) { 
			/* 巡逻过程中看见目标物料 */
            if (See_IsTarget(cam_target_id)) {
                GO_Stop();                       
                see_target  = 1;                /* 进入看见目标 */
                s_go_hf_n   = 0;                /* 对准次数 */
                s_step_t    = HAL_GetTick();    /* 时间基准 */
                break;
            }
            /* ① 先原地停 200ms：这期间上面的 See_IsTarget 一旦成立就直接进追踪 */
            if (!s_lock_dwell) {
                GO_Stop();
                if ((HAL_GetTick() - s_step_t) >= 200) {
                    s_lock_dwell = 1;
                    Go_sent = 0;
                }
                break;
            }
            /* 巡逻中看见非目标物料、且已离中心 ≥TF_PATROL_RECENTER_MIN_CM ⇒ 把窗口挪到看见它的位置（一条腿最多一次）；离得还近时不算。 */
            if (!s_lock_moved) {
                int mat = 0;
                if (cam_any_seq != s_mat_seq) {                  /* 有新物料帧才算一次 */
                    int c = cam_any_color;
                    uint32_t now = HAL_GetTick();
                    s_mat_seq = cam_any_seq;
                    /* 同色连续才计数；换色或距上一帧超过 400ms 就重新从 1 数（否则单帧就会被当成"确认看见"）*/
                    if ((c != s_mat_col) || ((now - s_mat_t) > 400)) {
                        s_mat_cnt = 1;
                    } else if (s_mat_cnt < 255) {
                        s_mat_cnt++;
                    }
                    s_mat_col = c;
                    s_mat_t   = now;

                    if (s_mat_cnt >= 2) mat = c;          /* 连续 2 帧同色才算真看见 */
                }

                if ((mat != 0) && !See_SameColor(mat, cam_target_id) &&
                    (fabsf(tf_car_pos() - s_search_center) >= TF_PATROL_RECENTER_MIN_CM)) {
                    s_lock_moved  = 1;
                    s_search_center = tf_car_pos();       /* 看见它的那一刻的跟踪轴坐标 */
                    Go_sent   = 0;                        /* 立刻按新中心重发巡逻点 */
                }
            }

            /* 双向巡逻：中心 ±TF_PATROL_SWING_CM 两点来回，先走哪边由 s_search_dir 定（见 To_Find_color），到位就反向。 */
            if (!Go_sent) {
                GO_Crawl(1);
                tf_go_pos(s_search_center + s_search_dir * tf_patrol_swing());
                Go_sent = 1;
            } else if (GO_Arrived()) {
                s_search_dir   = -s_search_dir;
                Go_sent  = 0;                 	/* 下一拍发另一个点 */
                s_lock_moved = 0;               /* 新的一条腿：允许再按看见的物料重定一次 */
            }
            break;
        }

        break;
	/* 跟踪 cam_cx（只动跟踪轴）；停不停由 Grab 的 grab_allow 统一判 */
    case 2:   
        tf_tracking = 1;                     /* 告诉手臂：底盘已经在跟踪，可以判停了 */

        /* 基准判定：同一块料在不同 x 上有的抓得到有的抓不到，先定一个基准。
         *   ③ M2 < TF_M2_GRAB_CM  ⇒ 够得着：定基准 + 当场抓（原规则）；
         *   ④ M2 ≥ TF_M2_GRAB_CM ⇒ 够不着：记第一次的位置，之后「x 比基准大出 TF_BASE_GRAB_DX」才抓，否则把基准刷新成更小的 x。
         * 上面所有的 x 都是 tf_car_pos() 的返回值 = **跟踪轴坐标**（右基准=任务X，左基准=任务Y），
         * 左右规则完全一致：当前 (x 或 y) 比基准大出 TF_BASE_GRAB_DX 就抓。*/
        if (grab_allow) {                  /* A1：目标不变 + 两轴到位，Grab 已判停 */
            float x = tf_car_pos();        /* 跟踪轴坐标（右=X / 左=Y）*/
            float m = cm_2;
            int pass = 0;

            grab_allow   = 0;
            ground_allow = 0;

            s_base_shot_prev = s_base_shot;    /* 先存一份：下面会覆盖 s_base_shot */

            if (g_base_en == 0) {
                /* 直接抓：基准恒 0（安全线失效）+ shot=2 ⇒ 判停用抓取门，判停一成立就抓 */
                s_base_pos  = 0.0f;
                s_base_shot = 2;
                pass        = 1;
            } else if ((grab_id + 1 >= GRAB_ROUNDS)) {
                pass = 1;                      /* 最后一块永远免安全线 */
            } else if (m < TF_M2_GRAB_CM) {    /* ③ 够得着：与 Update 版等价（只在还没有基准时写基准）*/
                if (s_base_shot_prev < 2) {    /* shot0 / shot1：定基准 */
                    s_base_pos  = x;
                    s_base_shot = 2;
                }
                /* ③b：**第一次**判停就扑过头（m < TF_M2_CY_OFF_CM）—— 这一次判停是靠"免判 cy"
                 * 成立的，位置不可信 ⇒ 只记基准不抓；下一次判停基准已有效、cy 必须真进死区才能成立，
                 * 即"免判只服务于量基准，抓取时不免判"。其余情况与原版一致：直接抓。*/
                if ((s_base_shot_prev != 0) || (m >= TF_M2_CY_OFF_CM)) {
                    pass = 1;
                }
            } else {                           /* ④ 够不着（M2 ≥ 10cm）：与 Update 版逐条等价 */
                if (s_base_shot == 0) {        /* shot0：只记位置，不定基准 */
                    s_base_pos  = x;
                    s_base_shot = 1;
                } else if (s_base_shot == 1) { /* shot1：取两次里较小的位置当基准，然后判一次 */
                    if (x < s_base_pos) s_base_pos = x;
                    s_base_shot = 2;
                    if (x > (s_base_pos + TF_BASE_GRAB_DX)) pass = 1;
                } else {                       /* shot2：基准已冻结，只判不放宽 */
                    if (x > (s_base_pos + TF_BASE_GRAB_DX)) pass = 1;
                }
            }

            if (pass) {
                s_go_grab_pos = x;
                tf_enter_grab(x);
                break;
            }

            /* 没通过：继续跟踪，等下一次对齐（grab_allow 由 Grab 重新置 1）*/
        }

        {
            int  new_frame;
            int  is_target;
            uint32_t fseq_save;

            fseq_save = s_go_fseq;         /* 先存一份：被节拍门挡住时把这一帧还回去 */
            new_frame = See_IsNewFrame(&s_go_fseq);
            is_target = See_IsTarget(cam_target_id);

            /* 先记"这一拍看到目标色新帧了"——必须在丢失判定之前，否则丢失一触发就再也刷不了时间戳，车永久停住（之前 cx=17 却一动不动就是这个）。 */
            if (new_frame && is_target) s_go_frame_t = HAL_GetTick();

            /* 断帧三级（和手臂一致）：0~250ms 等帧；250~500ms 保留上次前馈的一半、车以半速跟（别急刹）；>500ms 停车；>0.7s 回 case 1 重新巡逻。
             * 刹车必须保留，否则丢帧时车会按旧前馈一直走；颜色不对的帧也要走到这里，否则画面里只剩别的颜色时前馈会一直推着车跑。 */
            if ((HAL_GetTick() - s_go_frame_t) > 250) {
                uint32_t age = HAL_GetTick() - s_go_frame_t;

                s_go_first = 1;
                s_go_rate_f = 0.0f;   /* 恢复时不要带丢帧前的像素速度 */
                if (age <= 500) {
                    tf_track_ff(0.0f, s_go_dbg_vff * 0.5f);   /* 目标不动，半速前馈继续走 */
                } else {
                    GO_Stop();
                    if (age > CH_LOST_RETURN_MS) {            /* 真丢了：回巡逻 */
                        tf_tracking = 0;                      /* 底盘已不在跟踪，别让手臂继续据此判停 */
                        GO_SetTol(GO_TOL_XY_CM);
                        tf_resume_patrol();
                    }
                }
                break;
            }

            if (!is_target) break;           /* 画面里是别的颜色：不发新指令，等目标回来 */

            /* 50ms 一条（相机 20Hz）：前馈负责速度，位置只做小修正。 */
            if ((int32_t)(HAL_GetTick() - s_go_mute) < 0) {
                s_go_fseq = fseq_save;   /* 把这一帧还回去（同 Grab.c）：否则被门吞掉后再也读不到 */
                break;
            }

            if (new_frame) {
                int      cx_raw = cam_cx;

                float    cx;
                float    d;
                float    d_ff;              /* 原始像素变化（前馈用，不用滤波后的，少 2 帧滞后）*/
                float    step = 0.0f;
                float    vff = 0.0f;
                int  first = s_go_first;
                uint32_t now = HAL_GetTick();
                uint32_t dt_ms = now - s_go_prev_tick;

                /* 只对底盘用的 cx 做低通；0.5/0.5 ⇒ 只滞后 1 帧（原来 0.8/0.2 滞后 2 帧，环路延迟太大）。 */
                if (first) {
                    s_go_cx_f = (float)cx_raw;
                } else {
                    s_go_cx_f = 0.5f * s_go_cx_f + 0.5f * (float)cx_raw;
                }
                cx = s_go_cx_f;

                d = cx - s_go_prev_err;
                if (first) {                 /* 第一帧只走 P，不把整段偏差当速度突变 */
                    d = 0.0f;
                    s_go_first = 0;
                }

                /* 前馈用的“像素速度”：取**原始** cam_cx 的差分，再用 0.5 一阶平滑。 用滤波后的 cx 会多滞后，物料一停就多冲。 */
                d_ff = (float)cx_raw - s_go_prev_raw;
                if (first) d_ff = 0.0f;
                s_go_prev_raw = (float)cx_raw;
                if (dt_ms > 5 && dt_ms < 250) {
                    float rate = d_ff * 1000.0f / (float)dt_ms;
                    /* α 越小越平滑（0.5 = 默认，0.15 ≈ 0.4s 时间常数）；串口 23 已删 */
                    s_go_rate_f = 0.5f * s_go_rate_f + 0.5f * rate;   /* 一阶低通，α = 0.5 */
                } else {
                    s_go_rate_f = 0.0f;   /* dt 无效：像素速度按 0 处理（原行为） */
                }

                {
                    float d_n   = d;

                    if (dt_ms > 0) d_n = d * 50.0f / (float)dt_ms;  /* D 按 50ms 帧隔归一 */
                    if (dt_ms > 250) d_n = 0.0f;                    /* 间隔太长，D 不可信 */
                    d = d_n;

                    /* 物料速度前馈：v_obj = 车速 − 像素速度×比例尺。不能直接用误差变化率当速度——里面混着车自己的速度，会给成正反馈。
                     * 再做非对称平滑：变大/同向慢跟（滤像素噪声），变小/反向快跟（停车不过冲）。 */
                    {
                        float v_car = tf_car_v();

                        /* 前馈按 cam_px_to_cm 这个标定值换算；如果以后发现里程计标定偏了，就在这里乘一个补偿系数。
                         * dt 无效时 s_go_rate_f=0 ⇒ v_meas=车速（前馈把车速当物料速度），这是当前行为，别以为测到了物料速度。 */
                        float v_meas = v_car - s_go_rate_f * cam_px_to_cm;
                        float dv     = v_meas - s_go_vobj;

                        if (first) {
                            s_go_vobj = 0.0f;               /* 刚捕获：从 0 开始建速度 */
                        } else if ((v_meas * s_go_vobj) < 0.0f ||
                                   fabsf(v_meas) < fabsf(s_go_vobj)) {
                            s_go_vobj += 0.7f * dv;         /* 变小/反向：1~2 帧收掉，停车不过冲 */
                        } else {
                            s_go_vobj += 0.35f * dv;        /* 变大：τ≈140ms，比原来的 0.2 快一倍，减小滞后 */
                        }
                    }
                    /* 前馈也要跟着方向符号翻：位置项已经乘了 s_t1_sgn，前馈不翻就会和它对着干 → 目标附近小幅来回过冲。 */
                    vff = s_go_vobj * TRACK_FF_CH * s_t1_sgn;

                    /* 前馈随 M2 伸出量衰减：<=9cm 全量，9→19cm 线性降到 0.5 倍，>=19cm 保持 0.5。相机装在手臂上，臂伸出后 px/cm 变大
                     * ⇒ 像素速度被放大 ⇒ 前馈过强。只缩放前馈，不动位置项 K 和死区；参数见 grab_cfg.h 的 CH_FF_ARM_*。 */
                    {
                        float arm_cm = cm_2;
                        float mul    = CH_FF_ARM_START_MUL;

                        if (arm_cm >= CH_FF_ARM_END_CM) {
                            mul = CH_FF_ARM_END_MUL;
                        } else if (arm_cm > CH_FF_ARM_START_CM) {
                            mul = CH_FF_ARM_START_MUL
                                + (CH_FF_ARM_END_MUL - CH_FF_ARM_START_MUL)
                                  * (arm_cm - CH_FF_ARM_START_CM)
                                  / (CH_FF_ARM_END_CM - CH_FF_ARM_START_CM);
                        }
                        vff *= mul;
                    }

                    if (vff >  CH_TRACK_VFF_MAX_CMPS) vff =  CH_TRACK_VFF_MAX_CMPS;
                    if (vff < -CH_TRACK_VFF_MAX_CMPS) vff = -CH_TRACK_VFF_MAX_CMPS;
                    /* 物料静止时前馈应≈0；把测量残差按 0 处理，别让噪声推着车慢慢漂。 0.5cm/s 是很小的门限：手慢慢平移物料（2~3cm/s）时前馈仍然生效。 */
                    if (vff > -CH_VFF_DEAD_CMPS && vff < CH_VFF_DEAD_CMPS) vff = 0.0f;
                }
                s_go_prev_tick = now;
                s_go_prev_err  = cx;         /* 死区内也要更新，下一帧的 D 才连续 */

                {   /* 爪子中心在画面里：误差按 cam_grip_cx_px 这个偏移算 */
                    float e = cx - cam_grip_cx_px;    /* cx=cam_grip_cx_px 时误差为 0（原串口 16，已删）*/

                    /* 寻基准阶段用 TF_FIRST_TOL_PX（宽死区），记下基准后回 TRACK_TOL_PX（常规）*/
                    float tol_ch;

                    if (TaskFlow_BaseValid()) tol_ch = (float)TRACK_TOL_PX;
                    else                      tol_ch = (float)TF_FIRST_TOL_PX;

                    if (e > tol_ch || e < -tol_ch) {
                        step = s_t1_sgn * (-TRACK_K_CH * e - TRACK_KD_CH * d);   /* 方向由 s_t1_sgn（串口 1-1 / 1-0）定 */
                        /* 上限按控制帧间隔归一：控制 dt 实测 20~120ms，固定 4cm/帧在 20ms 时等于 200cm/s，必然甩飞；以 60ms 为基准缩放，不小于 0.5cm。 */
                        {
                            float sm = CH_TRACK_STEP_MAX_CM * (float)dt_ms / 60.0f;

                            if (sm < 0.5f)      sm = 0.5f;
                            if (sm > CH_TRACK_STEP_MAX_CM) sm = CH_TRACK_STEP_MAX_CM;
                            if (step >  sm) step =  sm;
                            if (step < -sm) step = -sm;
                        }
                    } else {
                        /* 进死区 ⇒ 前馈无条件清零：否则 vff 骑在阈值上会把车反复推出死区，形成来回震荡。*/
                        vff = 0.0f;
                    }
                }
                s_go_dbg_vff  = vff;
                tf_track_ff(step, vff);      /* 位置目标 + 速度前馈（X 或 Y 由出发边定）*/
                s_go_mute = HAL_GetTick() + TRACK_MUTE_MS;
            }
        }
        break;

    case 3:   /* 手臂抓取期间：底盘原地不动，发指令会顶掉对中位置 */
        if (!grab_id_done) {
            /* 三条退出条件，任一成立就回巡逻：① 手臂已不在跟踪/抓取（它可能自己退回扫描）；② 手臂清了 grab_allow；③ 6s 兜底。 */
            int gs = Grab_now;
            if ((gs != 2) && (gs != 3)) {
                s_step_t = 0;
                tf_resume_patrol();
                break;
            }
            if (!grab_allow) {
                s_step_t = 0;
                tf_resume_patrol();
                break;
            }
            if (s_step_t == 0) s_step_t = HAL_GetTick();
            else if ((HAL_GetTick() - s_step_t) > 6000) {
                s_step_t = 0;
                tf_resume_patrol();
            }
            break;
        }
        s_step_t = 0;
        grab_id_done = 0;
        if (grab_id < GRAB_ROUNDS) {
            tf_resume_patrol();              /* 以上一次实际 X 为中心 ±10 找下一块 */
        } else {
            Go_sub  = 1;
            Go_sent = 0;
            if (g_grab0 != 0) {
                Go_now = 10;               /* 单独测抓取：三块抓完就停（case 10 = GO_Stop）*/
            } else {
                Go_now  = 4;               /* 三块都抓完 → 进 case 4 */
            }
        }
        break;

    case 4:   /* 三块抓完 → 去 23 号格 → 跑任务 3 的路线 → 进 case 5 */
        switch (Go_sub) {
        case 1:   /* 4-1 去 23 号格，**同时**等手臂回到收工位；两者都满足才进 4-2 */
            if (!Go_sent) {
                GO_Crawl(0);
                GO_To(x[23], y[23], 0.0f);   /* 绝对坐标，朝向不动 */
                Go_sent = 1;
            } else if (GO_Arrived() && Grab_Done()) {     
                Go_sent = 0;
                Go_sub  = 2;
            }
            break;
        case 2:   /* 4-2 跑任务 3 的路线；走完直接进 case 5 */
            Follow_Walk(3);
            if (!Follow_WalkDone(3)) break;

            /* 交棒 case 5：手臂按扫码 4~6 位（qr_c1）排 7/8/9 的放料顺序，底盘只管对准 8 号环 */
            tf_ring_cycle_start();
            GO_Crawl(1);                     /* case 5 只巡逻 ±10cm，用蠕动档 */
            grab_round2  = 1;                /* 第二轮放行：手臂进它的 case 5 */
            grab_gate    = 1;                /* 放开手臂 */
            ring_ok = 0;                     /* 对准标志从头来 */
            Go_sent  = 0;Go_now   = 5;
            break;
        }
        break;
	/* 找 8 号环：第一轮用跟随轴（右X/左Y），第二轮用反轴（右Y/左X）；两轮共用这一个 case，和手臂的 case 5 一一对应。 */
    case 5:   
        tf_ring_step(s_ring_round);
        break;
    /* 放料，底盘不动 */
    case 6:         
        GO_Stop();
        Go_sent = 0;
        Go_now  = 7;
        break;
	/* 放料：底盘不动，Grab 循环三次把物料放到 7/8/9；手臂进收工位后一起走 */
    case 7:   
        GO_Stop();
        if (Grab_now >= 8) {        /* 手臂放完三块、进收工位（8/9/10）*/
            Go_sub  = 1;
            Go_sent = 0;
            if (big_round != 0) {
                Go_now = 10;         /* 第 2 遍结束：停 */
            } else if (s_ring_round == 0) {
                Go_now = 8;          /* 本例内第一次放完：3 号格 + 任务4 → 换轴找环 */
            } else {
                Go_now = 9;          /* 第 1 遍结束：11 号格 + 任务5 → 启动第 2 遍 */
            }
        }
        break;
	/* case 8 的主要目的是从粗加工区走到暂存区 */
    case 8:   /* 收尾：去 3 号格 → 跑任务 4 的路线 */
        switch (Go_sub) {

        case 1:   /* 8-1 去 3 号格（绝对坐标，朝向不动） */
            if (!Go_sent) {
				GO_SetTol(GO_TOL_XY_CM);         /* 回到大死区 */
                GO_Crawl(0);                     /* 退出蠕动档：大距离移动要正常速度 */
                GO_To(x[3], y[3], 0.0f);
                Go_sent = 1;
            } else if (GO_Arrived()) {
                Go_sent = 0;
                Go_sub  = 2;
            }
            break;

        case 2:   /* 8-2 跑任务 4 的路线；走完分两路：第一轮结束回 case 5 再找一轮 / 第 2 遍直接 Go_now = 10 停住（11/12 已不执行）*/
            Follow_Walk(4);
            if (!Follow_WalkDone(4)) break;

            if (big_round != 0) {
                /* 第 2 遍：8 号环之后不再重复"找环 + 放料"，改走 case 11（对准 8 号环里那块料）→ case 12（9/8/7 放料）。 */
                GO_Crawl(1);
                ring_ok     = 0;
                grab_round2 = 1;             /* 放行手臂：它自己会去 11 或 12 */
                Go_sent   = 0;
                Go_sub    = 0;
                g_c12_ready = 0;
                Go_now = 10;                 /* 第 2 遍：第二轮放料不做（不进 11/12）→ 直接收工停住 */
            } else if (s_ring_round == 0) {
                /* 第一轮结束 → 第二轮：回 case 5（用反轴），和手臂回到 case 5 同步 */
                s_ring_round = 1;
                tf_ring_cycle_start();
                GO_Crawl(1);
                grab_round2 = 1;             /* 放行手臂第二轮 */
                grab_gate   = 1;
                ring_ok     = 0;
                Go_sent   = 0;
                Go_sub    = 0;
                Go_now    = 5;
            } else {
                /* 第二轮也跑完了：停在原地不动（要循环/回原点在这里加）*/
                Go_sent = 0;
                Go_sub  = 3;
            }
            break;

        default:  /* 走完了：每一拍照样冻住目标，防止被人推/漂移 */
            GO_Stop();
            break;
        }
        break;
	/* 第 1 遍结束的转场：11 号格 → 任务5 → 清零 → 两台进 case 1（第 2 遍）*/
    case 9:   
        switch (Go_sub) {
        case 1:   /* 9-1 去 11 号格 */
            if (!Go_sent) {
                GO_Crawl(0);
                GO_To(x[11], y[11], 0.0f);      /* 绝对坐标，朝向不动 */
                Go_sent = 1;
            } else if (GO_Arrived()) {
                Go_sent = 0;
                Go_sub  = 2;
            }
            break;
        case 2:   /* 9-2 跑任务 5（21 号格转 90° → 23 号格不转）*/
            Follow_Walk(5);
            if (!Follow_WalkDone(5)) break;

            /* 第 2 遍启动：换二维码 + 清零 + 两台进 case 1 */
            big_round = 1;
            Grab_UseRound2();                   /* 手臂侧：qr_c2/qr_c3 → qr_c0/qr_c1 + 清第二遍标志 */
            Follow_WalkReset();                 /* 走位指针归零（路线数据保留）*/
            s_ring_round   = 0;                 /* 第 2 遍第一次找环回到跟随轴 */
            ring_ok        = 0;
            s_go5_align    = 0;
            s_go5_search_i = 0;
            s_go6_tol_n    = 0;
            s_lock_wait     = 0;
            grab_round2    = 0;
            grab_round3    = 1;                 /* 放行手臂：GB case 9 → 回 case 0 摆姿态 */
            tf_tracking    = 0;
            s_step_t      = 0;
            see_target   = 0;
            s_go_hf_n      = 0;
            Go_sub       = 0;
            /* 第 2 遍：基准重新采样 —— 不沿用第 1 遍抓料时定下的基准；
             * base_en=0 时直接视为"已定"，让第 2 遍第 1 块也用紧门 */
            s_base_shot = (g_base_en == 0) ? 2 : 0;
            s_base_pos  = 0.0f;
            tf_resume_patrol();                 /* → case 1 巡逻找料 */
            break;
        default:
            Go_sub  = 1;
            Go_sent = 0;
            break;
        }
        break;
   /* case10是第 2 遍也放完了：底盘停住，不再发任何目标 */
    case 10:  
        GO_Stop();
        break;
    /* case 11 的主要功能是第二遍第二次的对准 */
    case 11:  
        if (Grab_Ring8Color() != 0) See_AlignSetSel(1, Grab_Ring8Color());
        else                        See_AlignSetSel(0, Grab_Ring8Color());
        tf_ring_step(1);                          /* 第 2 遍里的第二次对准 → 用反轴 */
        break;
	/* case 12 的主要功能是第二轮第二次的放置 */
    case 12:  /* 第二轮第二次放置：底盘停住（等手臂按 9→8→7 放完）*/
        if ((g_case11_key == 0) && (Go_sub != 2)) {
            /* 跳过 case 11 ⇒ 没有图像对准，直接盲走到 8 号环的已知格心 */
            if (!Go_sent) {
                GO_Crawl(0);
                if (s_ring8_has) {
                    GO_To(s_ring8_px, s_ring8_py, 0.0f);   /* 开回"8 号环对准那一刻"的位姿 */
                } else {
                    GO_To(tf_ring8_fix_x(), tf_ring8_fix_y(), 0.0f);   /* 没记过才用约定值兜底 */
                }
                s_step_t    = HAL_GetTick();
                g_c12_ready = 0;
                Go_sent   = 1;
            } else if ((GO_Arrived()) ||
                       ((HAL_GetTick() - s_step_t) >= 15000)) {   /* 15s 兜底：打滑/到不了也放行，别让手臂死等 */
                GO_Stop();
                g_c12_ready = 1;           /* 到位（或超时）：放行手臂开始放料 */
                Go_sent   = 0;
                Go_sub    = 2;
            }
            break;
        }
        g_c12_ready = 1;                   /* 正常流程：底盘就停着，手臂已在 case 12 */
        GO_Stop();
        if (Grab_now >= 10) {          /* 手臂放完三块、进 case 10 */
            Go_sub  = 0;
            Go_sent = 0;
            Go_now  = 10;
        }
        break;
    }
}
