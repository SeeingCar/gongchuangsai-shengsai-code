/* chassis_cfg.h —— 底盘机械 / 运动学 / 控制参数（全工程唯一改参处）。 单位：位置 cm、速度 cm/s、角度 °、角速度 °/s、轮速 rpm；新车必测：轮半径、WHEEL_DIR、ROT_RPM_PER_DPS。
 * 轮序 wheel[0..3] = 右后 / 左前 / 右前 / 左后（★ 历史约定，不是右前/左前/左后/右后，别想当然）。 */
#ifndef __CHASSIS_CFG_H
#define __CHASSIS_CFG_H

/* 坐标系：任务系 原点 = 开跑位置、+X = 开跑车头方向、+Y = 左侧（GO / GO_To 用它）；车体系 x 朝前、y 朝左、yaw 逆时针为正。 世界系 = 以 IMU 为轴的固定系，里程计 x/y 与 pid 的速度都在这个系里。 */
#define WHEEL_RADIUS_CM       3.75f   /* 轮半径 cm */
#define ROT_RPM_PER_DPS       0.97f   /* 角速度(°/s) → 轮 rpm（车心到轮的等效半径比） */
#define RPM_PER_CMPS          (60.0f / (6.2831853f * WHEEL_RADIUS_CM))   /* cm/s → rpm */

/* WHEEL_DIRn = 前进时 Δ角度的符号（dir=0 时角度总是增加），逐轮点动实测： 轮0(轴1 右后) -1、轮1(轴6 左前) +1、轮2(轴3 右前) -1、轮3(轴4 左后) +1；正解里 wheel_rpm > 0 必须 = 该轮朝前进方向转。
 * ⚠ 这组只管反馈（MotorTx 里乘一下）；命令侧 dir 换算在 MotorA/MotorB，两条路独立、必须各改一处。 */
#define WHEEL_DIR0            -1.0f
#define WHEEL_DIR1            +1.0f
#define WHEEL_DIR2            -1.0f
#define WHEEL_DIR3            +1.0f

#define CTRL_DT_MS            20                      /* 控制周期 ms（全工程唯一出处，别再写 20 或 0.02；MotorTx 与 StartTask 都用它） */
#define CTRL_DT_S             (0.001f * CTRL_DT_MS)   /* 同一个周期的秒值 */

#define CHASSIS_V_MAX_CMPS    		50.0f   /* 纯直行/混合方向最高平移速度 cm/s */
#define CHASSIS_A_ACC_CMPS2   		300.0f  /* 纯直行/混合方向起步加速 cm/s²（刹车仍按 A_BRAKE，别一起抬）*/
#define CHASSIS_V_STRAFE_MAX_CMPS  	50.0f  /* 纯横移最高平移速度 cm/s（只按任务系 Y 方向走时）*/
#define CHASSIS_A_STRAFE_CMPS2     	300.0f  /* 纯横移起步加速 cm/s²（横移轮子抓地差，压低）*/
#define CHASSIS_ACC_RAMP_EN   		1       /* 1 = 保留加速斜坡；0 = 调试用，直接跟速度指令 */
#define CHASSIS_A_BRAKE_CMPS2 		150.0f   /* 实测刹车能力 cm/s²：刹车包线按它算这一拍敢跑多快 */
#define CHASSIS_W_MAX_DPS     		100.0f  /* 最大旋转速度 °/s */
#define GO_WHEEL_RPM_MAX      		200.0f  /* 逆解后单轮命令转速上限 rpm（四轮等比缩） */

/* 蠕动档：只有 GO_Crawl(1) 之后的平移目标才吃这两个值，走大距离的段位不受影响。 ⚠ a 太小 ⇒ 刹车距离 ≫ 死区 ⇒ 滑过目标点（pid.c 刹车包线 √(2·a·距离)；实测 8cm/s² 时 9cm/s 滑 5cm）。 */
#define CHASSIS_V_CRAWL_CMPS     	12.0f  /* 蠕动平移速度上限 cm/s（待实测调整） */
#define CHASSIS_A_CRAWL_CMPS2    	40.0f  /* 蠕动减速能力 cm/s²：12cm/s 时刹车距离 1.8cm ≈ 死区量级 */

/* 分离式转弯（GO() 和 GO_To() 都吃；GO_TrackX/GO_TrackY 不吃）：平移 + 大角度 ⇒ 先直行、到点前再甩头。 直行段把航向锁在"下目标那一刻的车头"，剩余距离 ≤
 * GO_TURN_LATE_CM 才放行真目标角；GO_TURN_LATE_DEG 以下的小角度不分。
 * ★ 想"走到更近才开始转" ⇒ 把 GO_TURN_LATE_CM 调**小**（越小越接近"先到位、再原地转"）。 ⚠ 到场判定 go_arrived 始终用真目标角，与这里无关。 */
#define GO_TURN_LATE_EN       1        /* 0 = 关：回到 GO() 原来边转边走的混合图形 */
#define GO_TURN_LATE_CM       10.0f    /* 剩余距离 ≤ 它 ⇒ 放行转向 cm */
#define GO_TURN_LATE_DEG      40.0f    /* 还差的角度 ≤ 它 ⇒ 不分离（小角度混着转更顺）° */

/* pid 三个环只用 kp（纯 P）：位置环减速由刹车包线管，到位车速 ≈ kp × GO_TOL_XY_CM = 2.5cm/s，滑行 ≈ v²/(2·a_brake) ≈ 0.05cm。 */
#define GO_XY_KP              5.0f
#define GO_YAW_KP             4.0f
#define GO_YAW_STEP_RPM       20.0f   /* 航向角加速度上限：每拍角速度命令最多变这么多轮 rpm（约 515°/s²） */

#endif /* __CHASSIS_CFG_H */
