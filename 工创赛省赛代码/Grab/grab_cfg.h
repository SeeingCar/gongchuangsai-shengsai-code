#ifndef __GRAB_CFG_H
#define __GRAB_CFG_H

/* 判停去抖：连续 N=3 个真实新帧都在死区内才算到点 */
#define STOP_CONFIRM_FRAMES   3     /* 判停连续帧数：2→3（多等约 60ms，换抗单帧误判）*/
/* 底盘移动死区 */
#define GO_TOL_XY_CM          0.5f          /*长距离死区*/
#define GO_TOL_YAW_DEG        0.5f			/*长距离死区*/
#define GO_TOL_XY_FINE_CM     0.05f			/* 跟踪/对准专用细死区 */
#define GO_TOL_XY_RING_CM     0.1f			/* 8号环对准底盘移动死区 */

//超时保护
	/* 总超时保护
	使用点 1：底盘 case 5 / case 11（圆环对准）
	使用点 2：底盘 case 2（追物料）
	使用点 3 / 4：手臂 case 5-2 / 11-2（圆环 M2 对准）*/
#define CH_LOST_RETURN_MS      700      	
	/* 用于grab的case2，200ms 内保持上一次的速度指令继续跑，避免手臂一顿一顿地抖*/ 
#define GB_LOST_HOLD_MS        200
	/* 用在grab的case2，丢了 700ms 还没回来，判定"这块料看丢了"回 case 1 重新扫。 */
#define GB_LOST_RETURN_MS      700      	

/* case 1 巡逻 */
#define GB_PATROL_M2_NEAR_CM    3.0f     	/* 手臂巡逻伸缩下限 cm（到 4cm就往外伸）*/
#define GB_PATROL_M2_FAR_CM    14.0f     	/* 手臂巡逻伸缩上限 cm（到 14cm就往回收）*/
#define GB_SCAN_CMPS           3.4f      	/* 巡逻时M2的速度 */
#define TF_PATROL_SWING_CM     10.0f		/* 底盘巡逻范围 */
#define TF_PATROL_SWING_AFTER_CM  5.0f		/* 抓过一块之后（grab_id >= 1）的底盘巡逻范围 */
#define TF_PATROL_RECENTER_MIN_CM  7.0f		/* 离初始位置多远可以换巡逻中心 */
/* case 2 追踪 ：位置步长 + 速度前馈 */
	//底盘跟踪增益
#define TRACK_K_CH             0.015f   	/* 底盘P cm/px（1cm 误差 → 0.46cm 步长）*/
#define TRACK_KD_CH            0.010f   	/* 底盘D：与 K 同步，保持 Kd/K≈0.67 */
#define TRACK_FF_CH            0.20f    	/* 底盘前馈增益：0=关，1=全额；中间值需实测 */
#define TRACK_TOL_PX           15       	/* 底盘跟踪死区 px（约 0.5cm）；只给底盘用 */
	//底盘前馈限幅与死区
#define CH_TRACK_VFF_MAX_CMPS  45.0f    	/* 底盘前馈速度上限 cm/s */	
#define CH_VFF_DEAD_CMPS       0.5f     	/* 底盘前馈死区 cm/s */
	//时序与失效保护
#define TRACK_MUTE_MS          50        	/* 底盘静默门：同一帧不重复算（相机 20Hz） */
#define GB_TRACK_MUTE_MS       45        	/* M2：45ms 节拍 */
	//手臂追踪增益范围参数
#define CH_FF_ARM_START_CM     9.0f      	/* 开始衰减的伸出量 cm */
#define CH_FF_ARM_END_CM       19.0f      	/* 衰减到最低的伸出量 cm */
#define CH_FF_ARM_START_MUL    1.00f     	/* 起点倍数 */
#define CH_FF_ARM_END_MUL      0.50f     	/* 终点倍数 */
	//步长上限
#define CH_TRACK_STEP_MAX_CM   4.0f      	/* 每帧步长上限 cm（按 dt/60 归一）：刹车包线 √(2·60·4)=22cm/s */
	//判基准
#define TF_FIRST_TOL_PX       30			/* 寻基准阶段的宽死区（还没记下基准时用） */
#define TF_BASE_GRAB_DX       2.0f   		/* 基准定下后：**跟踪轴**坐标比基准大这么多 cm 就可以抓（右=X / 左=Y）*/
#define TF_BASE_V_CMPS        2.5f   		/* 判基准阶段车必须几乎停住：|跟踪轴速度| <= 它（cm/s），Grab 判停用 */
#define TF_M2_GRAB_CM         10.0f   		/* 可抓阈值 cm */
#define TF_M2_CY_OFF_CM        2.0f   		/* 基准未定且 M2 小于它（车扑过头）⇒ 判停不判 cy（只在量基准阶段生效）*/


	//M2追踪限位
#define FB_MIN_CM              0.8f     	/* 追踪软下限 0.8cm */
#define FB_EXT_MAX_CM          19.0f      	/* 追踪软上限 19cm  */
#define FB_HOLD_CM             (-1.0f)     	/* MoveCm 的 fb_cm 传 -1 = M2 原地不动（不受 FB_MIN_CM 钳位）*/
#define CX_TARGET_PX          (-11.0f)  	/* 爪子中心在画面的 cx 位置 */

/* case3 抓取 */
#define GRAB_ROUNDS            3        	/* 一共抓几个物料（= qr_c0[] 里几个颜色） */
#define M6020_PLACE_DEG        197.0f    	/* 放料角：臂转到车上转盘上方（实测 196°）*/
#define FB_SAFE_CM             4.9f     	/* 子步 2 收回位（和起始位 3cm 同值不同义）*/
#define GRAB_SETTLE_TICKS      1        	/* 到位后稳定拍数（2→1：省 40ms）*/
#define GRAB_STOP_SETTLE_TICKS 1        	/* 停抓：UD 到位后稳 1 拍就闭爪 */
#define A1_CX_TOL_PX           18   		/* 可抓取是允许的横向偏差 */
#define GB_CHK_SKIP_FIRST_EN  1				/* 抓取检查，（case 3-4：抬 UD 后记录最近 6 帧颜色）的开关 */
	//抓取y轴死区扩大，M2小于2死区为20，M2大于17死区为25
#define GB_TOL_NEAR_M2CM       2.0f
#define GB_TOL_NEAR_CM        20.0f     	/* M2 < GB_TOL_NEAR_M2CM 时的死区 px */
#define GB_TOL_FAR_M2CM       17.0f
#define GB_TOL_FAR_CM         25.0f     	/* M2 >= GB_TOL_FAR_M2CM 时的死区 px */

/* 抓取时 UD 下降的高度 cm：数值在 freertos.c 配置区（#define high），这里只做跨文件声明。*/
extern const float g_high;

/* case5 对准8号环 */
	//底盘x/y轴的kp
#define K5_K_FAR               0.031f   	/* case5 X：|cx|> 60px */
#define K5_K_NEAR              0.040f   	/* case5 X：|cx|<=60px */
	//M2速度环
#define RING_M2_KV            2.1f       	/* 位置增益 1/s（2.4→2.1，略微减小）*/
#define RING_M2_VMAX          3.0f       	/* 指令上限 cm/s（圆环对中要慢）*/
#define RING_M2_ENV_K         0.7f       	/* 刹车包线系数：|v| ≤ K·stop_speed(|e|) */
#define RING_M2_MUTE_MS        45       	/* 速度指令最短间隔（相机 20Hz）*/
#define RING_M2_LOST_HOLD_MS  200       	/* 环短暂丢失：保持上次速度多久 */
#define RING_M2_STUCK_MS     1000       	/* 顶在限位且误差还大：多久判死 */
#define GB_RING_M2_SIGN     (+1.0f)      	/* 方向：与 case2 同向；若 cy 大时 M2 反向跑就改 -1 */
//圆环对准时的两轴限幅
#define CH_TRACK2_STEP_MAX_CM  4.0f      
	//对准判据
#define RING_K6_TOL_PX          6      	 	/* case 5/6 圆环对准死区 px（cx、cy 都要进这个门）*/
#define K6_ALIGN_FRAMES         7       	/* case 6：连续 N 帧 cx/cy 都在死区内才记录标定值 */
#define RING_K6_FRAMES          5         	/* case 5/6 连续 N 帧**两轴**都在 8px 内才算到 */
	//图像目标点
#define RING_CX_TGT_PX_7      (-10.0f)   	/* 7 号环对准目标：图像 x（px）*/
#define RING_CX_TGT_PX_8      (-11.0f)   	/* 8 号环对准目标：图像 x（px）*/
#define RING_CX_TGT_PX_9      (-13.0f)   	/* 9 号环对准目标：图像 x（px）*/
#define RING_CY_TGT_PX_7      (-20.0f)   	/* 7 号环对准目标：图像 y（px）*/
#define RING_CY_TGT_PX_8      (-21.0f)   	/* 8 号环对准目标：图像 y（px）*/
#define RING_CY_TGT_PX_9      (-17.0f)   	/* 9 号环对准目标：图像 y（px）*/
	//方向符号 ×4
#define K5_SX_SIGN_R            (-1.0f)  	/* 右基准 第一轮 */
#define K5_SX_SIGN_L          	(-1.0f)  	/* 左基准 第一轮 */
#define K9_SX_SIGN_R            (+1.0f)   	/* 右基准 第二轮 */
#define K9_SX_SIGN_L          	(-1.0f)   	/* 左基准 第二轮 */
	//M2初始伸长长度
#define K6_RETRACT_M2_CM        7.5f
#define FB_SCAN5_HI_CM         9.0f      	/* 巡逻M2最远伸到这儿 */
#define FB_SCAN5_LO_CM         3.0f      	/* 巡逻M2最近收到这儿 */
	//对准后重定基准
#define RING8_FIX_X_CM_R      105.0f   		/* 右基准：8 号环对准后纠正的 X 坐标 cm（case 5 对准）*/
#define RING8_FIX_Y_CM_R      102.0f   		/* 右基准：8 号环对准后纠正的 Y 坐标 cm（case 11 对准）*/
#define RING8_FIX_X_CM_L      104.0f   		/* 左基准：8 号环对准后纠正的 X 坐标 cm（case 11 对准）*/
#define RING8_FIX_Y_CM_L      106.0f   		/* 左基准：8 号环对准后纠正的 Y 坐标 cm（case 5 对准）*/

/* case6的PD对准 */
#define K6_D7_ANG_DEG           (-1.3f)  	/* 7 号角度修正（加在 90+θ 上）*/
#define K6_D7_M2_CM             (-0.4f)  	/* 7 号 M2 修正（加在模型长度上）*/
#define K6_D9_ANG_DEG           ( 0.0f)  	/* 9 号角度修正（加在 90-θ 上）*/
#define K6_D9_M2_CM             ( 0.0f)  	/* 9 号 M2 修正 */
#define K6_M2_OFF_CM            13.2f   	/* 长度偏置 c */
#define K6_M2_LAT_CM            1.15f   	/* 横向偏置 s */
#define K6_RING_ARM_OFFSET_CM   12.0f     	/* 固定偏置 */
#define K6_RING_OPP_CM          15.0f     	/* 直角三角形对边（两环间距）*/
                                      
#define K6_PD_ANG_KP           0.035f    	/* px -> 度 */
#define K6_PD_ANG_KD           0.008f
#define K6_PD_M2_KP            0.020f    	/* px -> cm */
#define K6_PD_M2_KD            0.004f
#define K6_PD_ANG_STEP_MAX     0.8f      	/* 每次最多转 0.8° */
#define K6_PD_M2_STEP_MAX      0.4f      	/* 每次最多伸 0.4cm */
#define K6_PD_PERIOD_MS        150      	/* 150ms 调一次 */



/* ---- M2 跟踪（Grab case 2，速度模式）---- 7 环节：vobj 估计 → 位置项 v = v_ff − Kv·e → 刹车包线 → 限位包线 → 实测超速刹车 → 死区 → 饱和/斜率。 只有一个位置增益，无 D 项 / 预测 / 捕获模式。
 * ⚠ ENV_K ≤ 1 是"刹车距离 ≤ |e|"不过冲的解析保证。 */
#define GB_TRACK_KV            2.4f     	/* [7]  位置增益 1/s（延迟 240ms 时 2~3 稳妥）*/
#define GB_TRACK_FF            0.0f     	/* [8]  物料速度前馈增益 0~1.5（[30] 同义）
											* ⚠ 默认 0：vobj = 臂速 − 像素速度，只有在“手臂伸 1cm、画面里物料
											*   也正好动 1cm”（图像增益 g=1）时才等于物料速度。g<1 时残差 =
											*   (1−g)·臂速，前馈再把它加回指令 ⇒ **手臂在放大自己的运动**，
											*   物料停下时更是纯自激（画面里唯一在动的就是手臂自己）⇒
											*   一路撞到端点再撞回来。要开前馈必须先标定 g 把 (1−g) 消掉。*/
#define GB_TRACK_TOL_PX        15       	/* [9]  死区 px（≈0.45cm）；|e| 小于它一律给 0 */
#define GB_TRACK_DECEL         40.0f    	/* [11] 刹车减速度 cm/s²（保守值；③/⑤ 都用它）*/
#define GB_K_LIM               2.5f     	/* [12] 限位包线斜率 cm/s 每 cm（方向相关；原“回退速度”编号）*/
#define GB_BRK_VFF_CMPS        1.0f     	/* ⑤ 的判据切换门 cm/s：|v_ff| 小于它 ⇒ 认为物料基本不动，
											*   臂速≈相对速度，用绝对判据（双方向都判超速）；
											*   否则 ⇒ 物料在动，只判"朝 |e| 变大方向跑"，避免把
											*   正确追赶误判成超速（仿真实测：fabsf 版会 2.3 次/秒误切、
											*   判停拖到 4.68s）。1.0 与 1.5 指标一致，1.0 更不易在门口抖。*/
#define GB_TRACK_ACC           0        	/* [13] 速度帧 acc 档 0~255：0 = 驱动器无加减速曲线（保持原值；
											*      想试加减速曲线可单独发 13-150，仿真显示无收益）*/
#define GB_TRACK_ENV_K         0.7f     	/* [24] 刹车包线折扣：允许修正 = k×stop_speed(|e|)
											*      （1.0 = 理论边界，0.7 = 留 30% 余量）*/
#define GB_TRACK_TAU           0.24f    	/* [25] 执行器总延迟 s（0.18→0.24 保守）：刹车包线用 */
#define GB_STOP_V_CMPS         2.5f     	/* [26] 判停速度门 cm/s：判“跟不跟得上物料”，不判静止 */
#define GB_FF_MAX_CMPS         8.0f     	/* [27] 前馈上限 cm/s（物料不可能比这更快）*/
#define GB_VMAX_CMPS          17.0f     	/* 总速度上限 cm/s（= 120rpm 折算，替代错误的 20.19）*/
/* 判据④：不可信帧（帧间隔很短而定位跳很大 ⇒ 突发帧/串色，整帧丢弃）*/
#define GB_JUMP_DT_MS          30
#define GB_JUMP_PX             60.0f
/* 判据③：够不着。按**时间**判，不按帧数（cy 一摆就把帧计数清零，实测从未触发）*/
#define GB_STUCK_MS            600
/* 物料速度估计的“噪声门” cm/s：|v_meas| 小于它就当真 0（挡在滤波之前）。 1.0 → 1.5：静止物料的像素抖动折成 1~1.2cm/s，门必须高过它，否则前馈自己抖。 */
#define GB_TRACK_VOBJ_GATE_CMPS  1.5f
/* vobj 滤波时间常数：按 dt 计算 alpha，使目标帧率变化时滤波速度一致 */
#define GB_VOBJ_TAU_UP_S       0.224f
#define GB_VOBJ_TAU_DN_S       0.174f

/* case11 圆环对准：图像 x → **底盘 X 前后**（TaskFlow 管）； 图像 y → **M2 速度模式**（Grab 管）。与 case2 抓料的职责划分一致。
 * 环是**静止**的，所以照 case2 的骨架做，但去掉物料速度估计/前馈（那部分对静止目标恒为 0）， 其余保护一个不少：看门狗三档、45ms 静默门、刹车包线、限位包线、实测超速刹车、死区、斜率。 */
#define RING_SEL_GATE_PX        100       	/* 选定颜色去重门：只认与 8 号环相差 ≤60px 的那块（车上可能还有同色料）*/

/* 6020 调参地图  */

/* ---- 1. 外环：角度环 ----------------------------------------------- */
/* 位置刚度 = ANG_KP × SPD_KP（V/°）。要“硬”就加 ANG_KP —— 位置反馈是干净的， 不会像 rpm 那样把尖峰放大成振动；SPD_KP 才是抖动的来源，别动它。 走车时臂被车体带着晃 ±2°，就是刚度不够：这里再抬一档到 ~840 V/°。 */
#define M6020_ANG_KP          24.0f     	/* [2] 3cm 端 P（15→24）*/
#define M6020_ANG_KP_MAX      20.0f     	/* [9] 18cm 端 P（12→20）*/
#define M6020_ANG_KD          1.5f      	/* [6] 3cm 端 D（1.0→1.5，压走车时的摆）*/
#define M6020_ANG_KD_MAX      2.2f      	/* [10] 18cm 端 D（1.5→2.2）*/

/* ---- 2. 轨迹整形（固定，不调）------------------------------------------ */
#define M6020_ACC_MAX         2700       	/* 刹车规划 °/s²；原 3000 的 0.9 倍 */
#define M6020_RATE_MAX        540.0f     	/* 最高速 °/s（默认；原 600 的 0.9 倍）*/
#define M6020_RATE_STEP       6.75f      	/* 每 5ms 加速度限幅；原 7.5 的 0.9 倍 */

/* ---- 3. 内环：速度环（只调这五个）------------------------------------ */
#define M6020_SPD_KP          36.0f      	/* [3] 3cm 端 P（30→36：刚度再抬一档，RATE_LP 已降到 0.35，噪声余量比之前大）*/
#define M6020_SPD_KP_MAX      42.0f      	/* [4] 18cm 端 P（35→42）*/
#define M6020_SPD_KD          1.0f       	/* [12] 3cm 端 D：对实测 rate 微分 */
#define M6020_SPD_KD_MAX      1.0f       	/* [13] 18cm 端 D */
#define M6020_SPD_KI_HOLD     4.0f       	/* [14] 3cm 端 |e|=0 时的 I */
#define M6020_HOLD_E_BAND     8.0f       	/* [15] 3cm 端拐点 */
#define M6020_SPD_KI_HOLD_MAX 3.0f       	/* [16] 18cm 端 |e|=0 时的 I（2→3：走车时更抗住单向负载）*/
#define M6020_HOLD_E_BAND_MAX 6.0f       	/* [17] 18cm 端拐点 */
#define M6020_SPD_ILIM        1800.0f    	/* [1] 积分项输出上限 */

/* ---- 4. 最内：电压给定限幅（固定）------------------------------------ */
#define M6020_CMD_MAX         20000.0f   	/* 电压给定限幅 ±20000/25000 */
#define M6020_CMD_SLEW        4000.0f    	/* 每 5ms 电压变化上限；4000 = 800k/s */

/* ---- 5. 反馈与到位（固定）-------------------------------------------- */
#define M6020_RATE_LP         0.35f      	/* 编码器 rpm 低通；5ms 下 τ≈11ms */
#define M6020_TOL_DEG         0.5f       	/* 严格到位角度容差 °（只有 case 6 标定用）*/
#define M6020_TOL_DEG_BIG     1.0f       	/* 宽松到位角度容差 °（其余步骤都用它）*/
#define M6020_TOL_DPS         10.0f      	/* 到位角速度容差 °/s（两个角度判据共用）*/

/* 舵机 pwm.c TIM4 两路 50Hz：夹爪 CH1、转盘 CH2；脉宽 0°→500µs / 270°→2500µs 舵机固定，写死在 pwm.c。 下列角度全部待实测/标定。 */
#define GRIP_OPEN_DEG       190.0f   		/* 夹爪张开（case 3 第 3 步）*/
#define GRIP_CLOSE_DEG      114.0f     		/* 夹爪夹紧（case 3 第 1 步）*/

#define CAROUSEL_ANG_INIT   214.0f       	/* 转盘：初始角度 */
#define CAROUSEL_ANG0       174.0f       	/* 转盘：第 1 块料（Servo2_SetAngle）*/
#define CAROUSEL_ANG1       122.0f       	/* 转盘：第 2 块料 */
#define CAROUSEL_ANG2       68.0f       	/* 转盘：第 3 块料 */
#define CAROUSEL_ANG_AVOID  60.0f       	/* 转盘：二维码第 4/5 位为 3（+6 后 =9）时，放第 1/2 块用的避让角 */
#define CAROUSEL_ANG_PARK   178.0f       	/* 转盘：收工位（4-1 / 8-1 用）*/

#endif /* __GRAB_CFG_H */
