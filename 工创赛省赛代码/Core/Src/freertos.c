/* USER CODE BEGIN Header */
/* freertos.c —— FreeRTOS 任务层：创建 9 个任务，负责底盘 20ms 控制拍、遥测与总线门控。 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "zdt_motor.h"
#include "hwt601.h"
#include "ins.h"
#include "Go.h"
#include "pid.h"
#include "TaskFlow.h"                      /* 顶层任务流程 */
#include "chassis_cfg.h"
#include "grab.h"                          /* 抓取层 */
#include "6020.h"                          /* GM6020 驱动 */
#include "grab_cfg.h"                      /* M6020_CMD_MAX：判输出打满 */
#include "M27.h"                           /* 伸缩 FB / 升降 UD */
#include <stdio.h>
#include <stdlib.h>                        /* atof：串口调参 */
#include "See.h"
#include "pwm.h"
#include "Follow.h"                        /* 路径规划：Follow_SideInit + Follow_RxByte */
#include "../../Test/Go_Test/go_test.h"    /* 台架：Go_Test_Route（GO_TEST_FOLLOWSEQ） */
extern UART_HandleTypeDef huart1;   /* 调试打印口 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 速度帧的加减速档直接写 0：不做曲线加减速，直接按设定速度跑（0~255，越大起步越猛）；单轮上限 GO_WHEEL_RPM_MAX 见 chassis_cfg.h。 */

/* 出发边：0 = 左出发（跟踪轴 = 任务 Y），1 = 右出发（跟踪轴 = 任务 X）。改错 ⇒ 整条路线镜像走反。 */
#define follow_side_key     1
const uint8_t g_follow_side_key = (uint8_t)follow_side_key;

/* 1 = 正常走 case 6（对 7/9 做 PD 对准并记录实测值）；0 = 跳过 7/9 的对准。 */
#define case6_key           1
const uint8_t g_case6_key = (uint8_t)case6_key;

/* 是否在第二次的第二轮进行对准 */
#define case11_key          0
const uint8_t g_case11_key = (uint8_t)case11_key;

/* 3(浅蓝)/6(深蓝) 相机分不稳：1 = 扫码时两者没同时出现就视为同一颜色；0 = 永远严格区分。 */
#define blue_merge_key      1
const uint8_t g_blue_merge_key = (uint8_t)blue_merge_key;

/* 放料盘高度 cm（实测值，初始 8）。抓取时 UD 的目标高度 = 5.15 - (high - 8)：
 *   盘高 8 ⇒ 5.15（当前基准值）；盘高 9 ⇒ 4.15。Grab.c case 3-0 那两句 MoveCm 的 ud_cm 用。*/
#define high                8.25f
const float   g_high = (float)high;

/* 5s 周期行的总闸：1 = 每 5s 打一遍三块料的抓取判定 6 帧颜色（例 …|000000|000000|000000）；三块都抓完（s_log_done）后自动停，不打扰抓取阶段的日志。 */
#define TELEM_PERIODIC_EN   1
/* 详细诊断行总开关（2026-10-31 改）：0 = 串口**只打两行** Q,<已锁定>,<料1>,<料2>,<料3>,<环1>,<环2>,<环3> C,7算6020,7算M2,7实测6020,7实测M2,
 * 8算…,8实测…, 9算…,9实测… 1 = 恢复原来的 Z/K/S/P 四行 +
 * 每 5s 的标定/颜色汇总行。 */
#define TELEM_VERBOSE_EN    0

/* 台架开关：1 = 上电发 0-1 之后**只跑** Go_Test_FollowSeq()（Follow_Set 里灌的三条模拟路线）， 跑完原地停住；TaskFlow / 手臂一律不参与。测完改回 0 就是正常比赛流程。
 * 打开时请把车架空或留足场地：三条路线是 1/3/13/23 号格之间的真实跑位。 */
#define GO_TEST_FOLLOWSEQ   0

/* 单独测试抓取：1 = 跳过前段（case -1 的路线/扫码/去 15 号格、case 0 的任务2），
 * 直接进 case 1「巡逻 → 跟踪 → 抓取」，三块抓完就停下（底盘进 case 10 GO_Stop、手臂摆收工位待命）。
 * ⚠ 颜色目标要自己灌：把下面正常分支里那行注释掉的 Set(...) 取消注释并填三块颜色 + 三个环号，
 *   否则 qr_c0[0] = 0 ⇒ 手臂永远扫不到料（case 0 会打印 cam_target_id = 0）。
 * ⚠ 只在 GO_TEST_FOLLOWSEQ = 0（正常分支）里生效。 0 = 正常全流程。*/
#define grab0               0
const uint8_t g_grab0 = (uint8_t)grab0;

/* 接线体检：1 = 每秒打 T,ANG / T,AGE / T,LIVE 三行，专门查断线、没应答、CAN/串口不通。
 *   T,ANG  = 6 个步进电机的多圈角度 + 6020 关节角 + 陀螺仪 yaw
 *   T,AGE  = 6 个步进电机"距上次应答多少 ms"（一直涨 = 那根线/地址有问题）
 *   T,LIVE = 6020 累计反馈帧数（不涨 = CAN 线/ID）+ 陀螺仪收到过好帧(0/1)
 * 调试用，正常比赛保持 0。*/
#define test                0

/* 基准判定总开关：1 = 完整判定（M2<10：定基准即抓；M2≥10：记历次最小值，退回来 2cm 才抓）；
 * 0 = 直接抓（基准恒 0、进度置"已定" ⇒ 判停成立就抓）。改动需重编译。*/
#define base_en             0
const uint8_t g_base_en = (uint8_t)base_en;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 4 轮目标转速 rpm[4]：MotorTx 写、MotorA/B 读；正 = 该轮朝前进方向转（WHEEL_DIR 校正后的符号）。 轮序 0..3 = 右后(轴1,总线A) / 左前(轴6,总线B) / 右前(轴3,总线B) / 左后(轴4,总线A)。 */
static volatile float s_tgt_rpm[4] = {0.0f};


/* 里程计状态（ins 模块）；位姿全工程只有这一份，Go 借它的指针（GO_Init(&s_ins)）。 */
static InsState_t s_ins;

/* 四轮编码器差分基准（放文件级是为了让 Motion_Home() 能清零；只清电机侧会出假尖峰）。 */
static float s_prev_angle[4] = {0, 0, 0, 0};   /* 上周期角度，差分算 rpm */
static float s_rpm_lp[4]     = {0, 0, 0, 0};   /* rpm 一阶低通（消差分噪声） */

/* wheel[] 下标 → 实际电机轴号（实测接线）：0=轴1右后 / 1=轴6左前 / 2=轴3右前 / 3=轴4左后。 ⚠ 底盘没有轴 2、5、7（轴 2 = 手臂前后、轴 7 = 手臂上下）：取错轴会让 wheel[1] 恒 0，vx/vy 全错。 */
static const uint8_t s_wheel_axis[4] = { 1, 6, 3, 4 };

/* ★ 临时（验证收发用）：1 = 收齐 8 段路线后就停住不跑流程；验证完改 0 就恢复正常。
 * ⚠ 放在 USER CODE 区里：CubeMX 重新生成只保留 USER CODE BEGIN/END 之间的内容。*/
static volatile int s_route_halt = 1;

/* USER CODE END Variables */
/* Definitions for TaskMotorTx */
osThreadId_t TaskMotorTxHandle;
const osThreadAttr_t TaskMotorTx_attributes = {
  .name = "TaskMotorTx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskMotorRx */
osThreadId_t TaskMotorRxHandle;
const osThreadAttr_t TaskMotorRx_attributes = {
  .name = "TaskMotorRx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for Task */
osThreadId_t TaskHandle;
const osThreadAttr_t Task_attributes = {
  .name = "Task",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for SeeTask */
osThreadId_t SeeTaskHandle;
const osThreadAttr_t SeeTask_attributes = {
  .name = "SeeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for Hwt601Task */
osThreadId_t Hwt601TaskHandle;
const osThreadAttr_t Hwt601Task_attributes = {
  .name = "Hwt601Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskMotorA */
osThreadId_t TaskMotorAHandle;
const osThreadAttr_t TaskMotorA_attributes = {
  .name = "TaskMotorA",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskMotorB */
osThreadId_t TaskMotorBHandle;
const osThreadAttr_t TaskMotorB_attributes = {
  .name = "TaskMotorB",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskMotorC */
osThreadId_t TaskMotorCHandle;
const osThreadAttr_t TaskMotorC_attributes = {
  .name = "TaskMotorC",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task6020 */
osThreadId_t Task6020Handle;
const osThreadAttr_t Task6020_attributes = {
  .name = "Task6020",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void Motion_PreInit(void);

/* USART1 发送：不用 HAL_UART_Transmit，避免它占住 huart1 的锁，导致 RX 回调里
 *   HAL_UART_Receive_IT 返回 BUSY、接收停掉。只轮询 TXE/TC，不碰中断配置。
 * 定义放在 USER CODE 区里（原来在区外，CubeMX 重新生成时被清掉了）。*/
static void uart1_send(const char *s, uint16_t n)
{
  uint16_t i;
  uint32_t t0 = HAL_GetTick();

  for (i = 0; i < n; i++) {
    while ((huart1.Instance->SR & USART_SR_TXE) == 0) {
      if ((HAL_GetTick() - t0) > 20) return;
    }
    huart1.Instance->DR = (uint8_t)s[i];
  }
  while ((huart1.Instance->SR & USART_SR_TC) == 0) {
    if ((HAL_GetTick() - t0) > 20) return;
  }
}

/* USER CODE END FunctionPrototypes */

void MotorTx(void *argument);
void MotorRx(void *argument);
void StartTask(void *argument);
void See(void *argument);
void Hwt601Poll(void *argument);
void MotorA(void *argument);
void MotorB(void *argument);
void MotorC(void *argument);
void Motor6020(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Motion_PreInit();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskMotorTx */
  TaskMotorTxHandle = osThreadNew(MotorTx, NULL, &TaskMotorTx_attributes);

  /* creation of TaskMotorRx */
  TaskMotorRxHandle = osThreadNew(MotorRx, NULL, &TaskMotorRx_attributes);

  /* creation of Task */
  TaskHandle = osThreadNew(StartTask, NULL, &Task_attributes);

  /* creation of SeeTask */
  SeeTaskHandle = osThreadNew(See, NULL, &SeeTask_attributes);

  /* creation of Hwt601Task */
  Hwt601TaskHandle = osThreadNew(Hwt601Poll, NULL, &Hwt601Task_attributes);

  /* creation of TaskMotorA */
  TaskMotorAHandle = osThreadNew(MotorA, NULL, &TaskMotorA_attributes);

  /* creation of TaskMotorB */
  TaskMotorBHandle = osThreadNew(MotorB, NULL, &TaskMotorB_attributes);

  /* creation of TaskMotorC */
  TaskMotorCHandle = osThreadNew(MotorC, NULL, &TaskMotorC_attributes);

  /* creation of Task6020 */
  Task6020Handle = osThreadNew(Motor6020, NULL, &Task6020_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MotorTx */
/* USER CODE END Header_MotorTx */
void MotorTx(void *argument)
{
  /* USER CODE BEGIN MotorTx */
  /* 底盘控制任务（20ms）：读反馈 → INS 推算 → Go 控制 → 写 s_tgt_rpm[4]；上层 StartTask 通过 GO 投递目标。 本任务不做串口发送：一行 85B 阻塞 ≈7.4ms，占 20ms 周期的 37%，打印全交给 MotorRx。 */
  const TickType_t period_tick = pdMS_TO_TICKS(20);
  TickType_t last_wake = xTaskGetTickCount();
  const float dt = CTRL_DT_S;      /* 控制周期，唯一出处见 chassis_cfg.h */
  /* 兜底上限（正常永远碰不到）：超过 2× 满速的单拍转角只可能是毛刺或驱动器复位。 真正的"上电首帧突变"已由下面的"每轴等真帧再建基准"从源头解决，这里只挡意外。 */
  const float max_step = GO_WHEEL_RPM_MAX * 6.0f * dt * 2.0f;

  static uint8_t s_base_ok[4] = {0};   /* 每轴：差分基准是否已用**真角度**建立 */
  InsInput_t in;
  float rpm_raw[4];                /* 本拍差分 rpm（低通前），用于算平滑后的反馈 */

  /* 装反的轮取反，见 chassis_cfg.h */
  static const float wdir[4] = { WHEEL_DIR0, WHEEL_DIR1, WHEEL_DIR2, WHEEL_DIR3 };

  for(;;)
  {
    int i;

    /* 角度差分 → rpm（一阶低通，α 越小越平滑） ⚠ 基准必须用**真角度**建：多圈角在第一次应答前恒为 0，拿它对 0 求差会造出"0→真值"的巨跳 （几千度 ⇒ 十万 rpm 级 ⇒ 米级假位移）。
     * 所以每轴单独等 ZDT_HasAngle()，第一次拿到真值的 那一拍只建基准、出 0，下一拍才开始差分。 某轴始终不应答 ⇒ 只有那一路恒 0（退化），其余三轮照常、车继续跑。 */
    for (i = 0; i < 4; i++) {
      int ax    = s_wheel_axis[i];
      float   a_now = zdt_angle[ax];

      if (!s_base_ok[i]) {
        if (ZDT_HasAngle(ax)) {
          s_prev_angle[i] = a_now;
          s_base_ok[i]    = 1;
        }
        rpm_raw[i] = 0.0f;
      } else {
        float deg = a_now - s_prev_angle[i];
        if (deg > max_step || deg < -max_step) deg = 0.0f;  /* 毛刺/驱动器复位：弃用本拍，基准照常刷新 */
        rpm_raw[i] = wdir[i] * (deg / dt) / 6.0f;   /* °/s /6 = rpm（未低通） */
        s_prev_angle[i] = a_now;
      }
      s_rpm_lp[i] = 0.35f * rpm_raw[i] + 0.65f * s_rpm_lp[i];
      in.wheel_rpm[i] = s_rpm_lp[i];
    }

    in.imu_yaw   = HWT601_GetYaw();

    INS_Update(&s_ins, &in);

    /* GO：装载投递的目标 + 出四轮转速（状态写都在本任务，单写者） 0-1 之前不跑 GO：否则 Motion_Home 锁的 yaw0 与当前 IMU 有差时，车会在启动前自己转。 */
    float rpm_out[4];
    GO_ControlCycle(rpm_out);

    for (i = 0; i < 4; i++) {
      s_tgt_rpm[i]  = rpm_out[i];
    }

    vTaskDelayUntil(&last_wake, period_tick);
  }
  /* USER CODE END MotorTx */
}

/* USER CODE BEGIN Header_MotorRx */
/* USER CODE END Header_MotorRx */
void MotorRx(void *argument)
{
  /* USER CODE BEGIN MotorRx */
  const TickType_t period_tick = pdMS_TO_TICKS(CTRL_DT_MS);
  TickType_t last_wake = xTaskGetTickCount();
  static uint32_t last_chk_t = 0;     /* 抓取判定 6 帧打印：每 5s */
  static int  s_log_done = 0;     /* 1 = 周期行已停发（锁存）*/
  static int  s_log_summary = 0;  /* 1 = 停发前补打一行 5s 总结 */
  for(;;)
  {
    char b[256];
    char cmd[24];
    int  n;

    /* 1) 串口命令 编号-数值（如 1-3）：换行或 50ms 静默结束，回 OK/ERR；编号含义见下面 case。
     * 其余调参项已删，用宏默认值。 */
    if (Grab6020_CmdPoll(cmd, sizeof(cmd))) {
      int idx = 0;
      int p = 0;
      float val;

      while (cmd[p] >= '0' && cmd[p] <= '9') {
        if (idx < 1000) idx = idx * 10 + (cmd[p] - '0');   /* 饱和，避免长数字溢出 */
        p++;
      }
      /* 必须至少有一位前导数字：否则 "-1" 会被解析成"0 号命令 val=1" ⇒ 行驶中被一条杂散行开车/停车 */
      if (p > 0 && idx <= 5 && cmd[p] == '-' &&
          (cmd[p + 1] == '.' || (cmd[p + 1] >= '0' && cmd[p + 1] <= '9'))) {
        val = (float)atof(&cmd[p + 1]);

        switch (idx) {

        case 1:  val = TaskFlow_SetSign(1, val); break;   /* case2 底盘 X */
        case 2:  val = Grab_SetSign(1, val);     break;   /* case2 的 M2  */
        case 3:  val = TaskFlow_SetSign(2, val); break;   /* case5 第一轮底盘 */
        case 4:  val = Grab_SetSign(2, val);     break;   /* case5 的 M2（两轮）*/
        case 5:  val = TaskFlow_SetSign(3, val); break;   /* case5 第二轮底盘 */
        default: break;
        }
        n = snprintf(b, sizeof(b), "OK %d=%.3f\r\n", idx, (double)val);
      } else {
        n = snprintf(b, sizeof(b), "ERR\r\n");
      }
      if (n > (int)sizeof(b) - 1) n = (int)sizeof(b) - 1;
      if (n > 0) uart1_send(b, (uint16_t)n);
    }

    if (grab_log_done != 0) {
      if (!s_log_done) { s_log_done = 1; s_log_summary = 1; }
    } else {
      s_log_done = 0;
    }


    /* 每 1000ms 一行：C,… 标定对照（原来的 Q 行已随 Z 行一起删掉，代码里不再打印）*/
    {
      static uint32_t last_qr_t = 0;

      if ((TELEM_VERBOSE_EN != 0) && ((HAL_GetTick() - last_qr_t) >= 1000)) {          /* Q/C 两行的打印周期（ms）*/
        last_qr_t = HAL_GetTick();

        /* ② 标定对照 C：每环两列 ——「算」= 按 θ/M2₈ 现算的目标，「实测」= 标定 PD 完成时记下的值 ⇒ 两列相减就是几何公式与实际偏了多少；case 6 没跑过时这些列都是 0。 */
        {
          double th  = (double)grab_k6_th;          /* θ = atan(15 / (c+M2₈)) */

          n = snprintf(b, sizeof(b),
                       "C,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                       90.0 + th + (double)K6_D7_ANG_DEG, (double)Grab_K6M2Of(7),   /* 7 号算（含偏置）*/
                       (double)Grab_CalAng(0), (double)Grab_CalM5(0),      /* 7 号实测 */
                       90.0,                   (double)Grab_CalM5(1),      /* 8 号算 */
                       (double)Grab_CalAng(1), (double)Grab_CalM5(1),      /* 8 号实测 */
                       90.0 - th + (double)K6_D9_ANG_DEG, (double)Grab_K6M2Of(9),   /* 9 号算（含偏置）*/
                       (double)Grab_CalAng(2), (double)Grab_CalM5(2));     /* 9 号实测 */
          if (n > 0 && n < (int)sizeof(b)) uart1_send(b, (uint16_t)n);
        }

        /* 详细诊断三行（K/S/P）：TELEM_VERBOSE_EN=0 时一律不打。 */
        if (TELEM_VERBOSE_EN) {

          /* K,θ,斜边,M2₈记录值,M2当前值 —— 判断"算得对不对 / 反馈有没有上来"： M2₈=0 且 M2 当前=0 ⇒ 编码器/总线没回；M2₈=0 但当前有值 ⇒ 记录那一刻是 0（时序）。 */
          n = snprintf(b, sizeof(b), "K,%.1f,%.2f,%.2f,%.2f\r\n",
                       (double)grab_k6_th, (double)grab_k6_m2,
                       (double)Grab_CalM5(1), (double)cm_2);
          if (n > 0 && n < (int)sizeof(b)) uart1_send(b, (uint16_t)n);

          /* S,目标色,当前看见的颜色,cx,cy,环号,环cx,环cy */
          n = snprintf(b, sizeof(b), "S,%d,%d,%d,%d,%d,%d,%d\r\n",
                       (int)cam_target_id, (int)cam_color,
                       (int)cam_cx, (int)cam_cy,
                       (int)cam_ring_no, (int)cam_ring_cx, (int)cam_ring_cy);
          if (n > 0 && n < (int)sizeof(b)) uart1_send(b, (uint16_t)n);

          /* P 行：每秒打**一段路线**（任务 1→2→…→8 循环）
           *   P,<任务号>,<对数>,<格1>,<角1>,<格2>,<角2>…
           *   例：P,1,2,5,0,15,-90  = 任务 1有 2 对：5 号格不转 → 15 号格转 -90°*/
          {
            static int s_rt_i = 1;          /* 下一秒打哪一段 */
            int nt = path_n[s_rt_i];
            int k;
            int     q;

            q = snprintf(b, sizeof(b), "P,%u,%u", (unsigned)s_rt_i, (unsigned)nt);
            for (k = 1; (k <= nt) && (k <= FOLLOW_CELL_MAX); k++) {
              if (q >= (int)sizeof(b) - 16) break;
              q += snprintf(b + q, sizeof(b) - (size_t)q, ",%u,%d",
                            (unsigned)path[s_rt_i][k], (int)ang[s_rt_i][k]);
            }
            if ((q > 0) && (q < (int)sizeof(b) - 2)) {
              b[q++] = '\r'; b[q++] = '\n';
              uart1_send(b, (uint16_t)q);
            }
            s_rt_i = (uint8_t)((s_rt_i >= 8) ? 1 : (s_rt_i + 1));
          }

        }
      }
    }

    /* test = 1：每秒一行"接线体检" X,… —— 6 个步进电机的角度 + 6020 关节角 + 陀螺仪 yaw。
     * （前缀用 X 而不是 T：T 已被台架的 Go_Test_StraightThenSideRun 占用）
     * 轴顺序：轴1(底盘右后) 轴6(左前) 轴3(右前) 轴4(左后) 轴2(手臂伸缩) 轴7(手臂升降)。
     * 用手掰某根轴 / 掰 6020：数值跟着变 = 反馈正常；一直不动 = 那根的线、ID 或供电有问题。*/
    if (test != 0) {
      static uint32_t last_test_t = 0;

      if ((HAL_GetTick() - last_test_t) >= 1000) {
        last_test_t = HAL_GetTick();
        n = snprintf(b, sizeof(b), "X,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                     (double)zdt_angle[1], (double)zdt_angle[6],
                     (double)zdt_angle[3], (double)zdt_angle[4],
                     (double)zdt_angle[2], (double)zdt_angle[7],
                     (double)Grab6020_GetAngle(), (double)HWT601_GetYaw());
        if (n > 0 && n < (int)sizeof(b)) uart1_send(b, (uint16_t)n);
      }
    }

    if (TELEM_PERIODIC_EN &&
        (s_log_summary || ((HAL_GetTick() - last_chk_t) >= 5000))) {
      char    s[24];
      int m, i, p = 0;

      last_chk_t = HAL_GetTick();
      s_log_summary = 0;
      for (m = 0; m < GRAB_ROUNDS; m++) {
        for (i = 0; i < 6; i++) {
          int v = Grab_ChkFrame(m, i);
          s[p++] = (v <= 9) ? (char)('0' + v) : '?';
        }
        if (m + 1 < GRAB_ROUNDS) s[p++] = '|';
      }
      s[p] = '\0';
      /* 每 5s 只打**抓取判定颜色帧**：V,<第1块6帧>|<第2块6帧>|<第3块6帧> 例 V,111111|000000|000000 = 第 1 块抬爪后 6 帧都是色 1（判成功）， 第 2/3 块还没抓
       * ⇒ 全 0。标定/实测对照已挪到每秒的 C 行，这里不再重复。 */
      n = snprintf(b, sizeof(b), "V,%s\r\n", s);
      if (n > 0) uart1_send(b, (uint16_t)n);
    }

    vTaskDelayUntil(&last_wake, period_tick);
  }
  /* USER CODE END MotorRx */
}

/* USER CODE BEGIN Header_StartTask */
/* USER CODE END Header_StartTask */
void StartTask(void *argument)
{
  /* USER CODE BEGIN StartTask */

	/* 上电回零 */
	Gripper_Close();
	osDelay(1000);
	M27_Home();
	osDelay(5000);
	/* 恢复初始位姿 */
	MoveCm(200.0f, 0.0f, -1, 900, 100, 0.0f, 900, 100);   /* M2 原地不动：PD13 之前手臂不伸 */
	uint32_t t0 = HAL_GetTick();
	while ((!M27_Arrived() || !Grab6020_Arrived_Big()) && (HAL_GetTick() - t0) < 5000) 
	{osDelay(20);}  
	/* 一键启动：PD13 */
	while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) 
	{osDelay(CTRL_DT_MS);}

	
//  {
//    uint32_t t_led = HAL_GetTick();

//    while (s_route_halt != 0) {                 /* 跑完不进入正常全流程，停在这里 */
//      /* 停在这儿的时候闪灯（500ms 翻转一次）：一眼看出程序还活着，只是被这个闸门停住了。 */
//      if ((HAL_GetTick() - t_led) >= 500) {
//        t_led = HAL_GetTick();
//        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//      }
//      osDelay(CTRL_DT_MS);
//    }
//    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   /* 真放行了就把灯灭掉 */
//  }

  TaskFlow_Init();      /* 归零后重新清一遍流程内部状态 */
  TaskFlow_Ready();		/* 死等 Motion_Home 成功（归零 + 锁航向 + 对齐四轮基准）→ 拨到 case -1 */  
  /* 上位机路线握手：
   *   ① 按 follow_side_key 发 3 个字节给上位机：0 → '#2*'（左），1 → '#1*'（右）；
   *   ② 进 while 等上位机把路线发进来；**8 段（任务 1..8）全到齐**才往下走。*/
  See_SendSide(follow_side_key);
  while (!See_RouteReceived()) {
    osDelay(CTRL_DT_MS);
  }
  See_PreFlowDone();                             /* 路线到手：恢复相机/扫码解析 */
	
#if (GO_TEST_FOLLOWSEQ != 0)
  /* ⚠ 这一段是台架专用：跑完就死循环停住，下面的正常流程不会执行（用 #else 隔开避免 "unreachable" 警告）*/
  /* ===== 台架：只跑**上位机发来的路线**（不碰相机、不碰手臂）===== ① 等上位机把 @任务+格号(角)+…% 发进来（最多 30s） ② 收到后把表打一遍（R,…），确认解析 ③ 依次跑有数据的任务号，跑的时候每 1 秒打一行 F,…，跑完 F,DONE */
	Go_Test_GoToPreRun();
	for (;;) {
		GO_Stop();                     /* 目标冻在当前位姿：控制周期出零 */
		osDelay(200);
	}
#else   /* GO_TEST_FOLLOWSEQ == 0：正常比赛流程 */
  /* 先只放底盘跑 case -1（路线 + 二维码 + 去 15 号格）；它跑完会 grab_gate=1 放手臂 */
  if (g_grab0 != 0) {
      /* 单独测抓取：底盘先不放行（tf_gate = 0），等手臂 case 0 摆好位由它置 tf_gate = 1；
       * 手臂立刻放行。之后底盘直接从 case 1 巡逻开始跑。*/
      tf_gate   = 0;
      grab_gate = 1;
  } else {
      tf_gate   = 1;
      grab_gate = 0;
  }
  Grab_Start();

  /* Set(2, 3, 1, 1, 3, 2, 3, 2, 1, 3, 2, 1); */

  
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
      TaskFlow_Update();          /* 底盘：case -1 路线/二维码/去 15 号格 → 0 → 1 找料 → 2 跟踪 → 3 抓取 → 4 放置 */
      GRAB_Update();              /* 手臂：case 0 摆位 → 1 扫描 → 2 跟踪 → 3 抓取；三块后停 40 */
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_DT_MS));
    }
  
#endif  /* GO_TEST_FOLLOWSEQ */
  /* USER CODE END StartTask */
}

/* USER CODE BEGIN Header_See */
/* * @brief Function implementing the SeeTask thread. @param argument: Not used @retval None */
/* USER CODE END Header_See */
void See(void *argument)
{
  /* USER CODE BEGIN See */
  (void)argument;

  for(;;)
  {
    See_WaitFrame(osWaitForever);   /* 等一帧（队列空就一直阻塞） */
    See_ParseFrame();               /* 解析它，填 cam_* / qr_* */
  }
  /* USER CODE END See */
}

/* USER CODE BEGIN Header_Hwt601Poll */
/* USER CODE END Header_Hwt601Poll */
void Hwt601Poll(void *argument)
{
  /* USER CODE BEGIN Hwt601Poll */
  /* HWT601 周期轮询（Modbus 读数），周期 = HWT601_POLL_PERIOD_MS；这是 IMU 唯一数据来源，注掉它 Yaw 就是死的（与 MotorA/B 同为 10ms、同级 40 抢 CPU）。 */
  const TickType_t period_tick = pdMS_TO_TICKS(HWT601_POLL_PERIOD_MS);
  TickType_t last_wake = xTaskGetTickCount();
  for(;;)
  {
    HWT601_Poll();

    vTaskDelayUntil(&last_wake, period_tick);
  }
  /* USER CODE END Hwt601Poll */
}

/* USER CODE BEGIN Header_MotorA */
/* USER CODE END Header_MotorA */
void MotorA(void *argument)
{
  /* USER CODE BEGIN MotorA */
  /* 总线 A = USART2：1 号(右后)、4 号(左后)。轴 2 不在这条总线上，读到的是手臂前后电机。 */
  const TickType_t period_tick = pdMS_TO_TICKS(10);
  TickType_t last_wake = xTaskGetTickCount();

  for(;;)
  {
    ZDT_Poll();

    ZDT_ReadPos(1);
    osDelay(3);
    ZDT_ReadPos(4);
    osDelay(3);

    ZDT_UpdateAngle(1);
    ZDT_UpdateAngle(4);

    /* 这里**不再**判 ZDT_AngleFresh：丢帧时 zdt_angle 保持旧值 ⇒ Δθ=0 ⇒ 轮速自然衰减到 0， 速度命令照常下发，车不会急停。总线真掉线只会让这一路轮速停在旧值，由人从遥测判断。
     * （原先是连续丢 4 帧就发 Stop 帧 + 把 valid 拉低 ⇒ 比赛中一次总线抖动四轮同时归零。） */

    /* 1 号 = wheel[0] 右后：前进 = dir 1（实测 dir1 时角度减小）*/
    {
      float   rpm = s_tgt_rpm[0];
      int dir = (rpm > 0.0f) ? 1 : 0;
      float   spf = (rpm < 0.0f) ? -rpm : rpm;
      uint16_t sp = (spf > GO_WHEEL_RPM_MAX) ? (uint16_t)GO_WHEEL_RPM_MAX : (uint16_t)(spf + 0.5f);  /* 四舍五入：截断会把 <1rpm 的末段推力吃成 0，0.05cm 死区就永远到不了 */
      ZDT_MMCL_AddSpeed(ZDT_BUS_A, 1, dir, sp, 0);
    }
    /* 4 号 = wheel[3] 左后：前进 = dir 0（实测 dir0 时角度增大）*/
    {
      float   rpm = s_tgt_rpm[3];
      int dir = (rpm > 0.0f) ? 0 : 1;
      float   spf = (rpm < 0.0f) ? -rpm : rpm;
      uint16_t sp = (spf > GO_WHEEL_RPM_MAX) ? (uint16_t)GO_WHEEL_RPM_MAX : (uint16_t)(spf + 0.5f);  /* 四舍五入，同上 */
      ZDT_MMCL_AddSpeed(ZDT_BUS_A, 4, dir, sp, 0);
    }
    ZDT_MMCL_Send(ZDT_BUS_A);

    vTaskDelayUntil(&last_wake, period_tick);
  }
  /* USER CODE END MotorA */
}

/* USER CODE BEGIN Header_MotorB */
/* USER CODE END Header_MotorB */
void MotorB(void *argument)
{
  /* USER CODE BEGIN MotorB */
  /* 总线 B = UART4：3 号(右前)、6 号(左前)。6 号是底盘左前轮，不是手臂升降 UD。 */
  const TickType_t period_tick = pdMS_TO_TICKS(10);
  TickType_t last_wake = xTaskGetTickCount();

  for(;;)
  {
    ZDT_Poll();

    ZDT_ReadPos(3);
    osDelay(2);
    ZDT_ReadPos(6);
    osDelay(2);

    ZDT_UpdateAngle(3);
    ZDT_UpdateAngle(6);

    /* 同上：不判新鲜度，丢帧只让这一路轮速衰减，命令照发、不停车。 */

    /* 3 号 = wheel[2] 右前：前进 = dir 1（实测 dir1 时角度减小）*/
    {
      float   rpm = s_tgt_rpm[2];
      int dir = (rpm > 0.0f) ? 1 : 0;
      float   spf = (rpm < 0.0f) ? -rpm : rpm;
      uint16_t sp = (spf > GO_WHEEL_RPM_MAX) ? (uint16_t)GO_WHEEL_RPM_MAX : (uint16_t)(spf + 0.5f);  /* 四舍五入：截断会把 <1rpm 的末段推力吃成 0，0.05cm 死区就永远到不了 */
      ZDT_MMCL_AddSpeed(ZDT_BUS_B, 3, dir, sp, 0);
    }
    /* 6 号 = wheel[1] 左前：前进 = dir 0（实测 dir0 时角度增大）*/
    {
      float   rpm = s_tgt_rpm[1];
      int dir = (rpm > 0.0f) ? 0 : 1;
      float   spf = (rpm < 0.0f) ? -rpm : rpm;
      uint16_t sp = (spf > GO_WHEEL_RPM_MAX) ? (uint16_t)GO_WHEEL_RPM_MAX : (uint16_t)(spf + 0.5f);  /* 四舍五入，同上 */
      ZDT_MMCL_AddSpeed(ZDT_BUS_B, 6, dir, sp, 0);
    }
    ZDT_MMCL_Send(ZDT_BUS_B);

    vTaskDelayUntil(&last_wake, period_tick);
  }
  /* USER CODE END MotorB */
}

/* USER CODE BEGIN Header_MotorC */
/* USER CODE END Header_MotorC */
void MotorC(void *argument)
{
  /* USER CODE BEGIN MotorC */
  /* 总线 C = UART5：手臂伸缩 = 电机2、升降 = 电机7。节拍参考 MotorA：读 2 → 3ms → 读 7 → 3ms， 再刷新角度/换算 cm 并写帧（2 → 5ms → 7），留余量避免和第 7 号的应答在半双工总线上撞车。 */
  const TickType_t period_tick = pdMS_TO_TICKS(20);
  TickType_t last_wake = xTaskGetTickCount();

  for(;;)
  {
    ZDT_Poll();

    ZDT_ReadPos(2);
    osDelay(3);
    ZDT_ReadPos(7);
    osDelay(3);                          /* 等 7 号应答上来再换算角度 */

    ZDT_UpdateAngle(2);
    ZDT_UpdateAngle(7);

    M27_Update();                        /* 刷新 cm + 到位判定 */
    M27_Send();                          /* 有新目标才发：电机2 帧 → 10ms → 电机7 帧 */

    vTaskDelayUntil(&last_wake, period_tick);
  }
  /* USER CODE END MotorC */
}

/* USER CODE BEGIN Header_Motor6020 */
/* * @brief Function implementing the Task6020 thread. @param argument: Not used @retval None */
/* USER CODE END Header_Motor6020 */
void Motor6020(void *argument)
{
  /* USER CODE BEGIN Motor6020 */
  /* 6020 双环 5ms 一拍：反馈 1kHz，这里每 5 帧取一帧。 改这一拍必须同步改 grab_cfg.h 的折算：积分/斜率/步长都是“每拍”量。 */
  const TickType_t period_tick = pdMS_TO_TICKS(5);
  TickType_t last_wake = xTaskGetTickCount();

  for(;;)
  {
    Grab6020_Update();

    vTaskDelayUntil(&last_wake, period_tick);
  }
  /* USER CODE END Motor6020 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* 底盘归零 + 锁基准（开跑前一次）：位置归零在 INS_ZeroPos、锁航向在 GO_Home()、四轮差分基准由 GO_OdomBaseReset() 对齐。 返回 1=成功、0=IMU 一帧都还没来过。
 * 全工程唯一问"数据来没来"的地方，只在起步这一刻问一次； 不放在控制回路里，否则就成了"掉线即停车"。 */
uint8_t Motion_Home(void)
{
    if (!HWT601_HasFrame()) return 0;         /* 一次都没收到过：不能拿哨兵 yaw=0 去锁航向基准 */
    INS_ZeroPos(&s_ins);                     /* 里程计位置归零 */
    return GO_Home();                        /* 锁基准 + 对齐四轮差分基准 */
}

/* 第二次归零：只清 XY，航向基准不动。起点摆位后调用。 */
uint8_t Motion_ZeroXY(void)
{
    if (!HWT601_HasFrame()) return 0;
    INS_ZeroPos(&s_ins);                     /* 只清位置/速度，不改 yaw */
    GO_ZeroXY();                             /* 只清目标 XY，不动 yaw0/tar_yaw */
    return 1;
}

/* 由 GO_Home() 回调：把四轮差分基准"对齐"到当前角度（清 0 会造出一拍假速度尖峰，被 INS 积分成假位移），须与 INS_ZeroPos 同一时刻完成。 */
void GO_OdomBaseReset(void)
{
    int i;

    for (i = 0; i < 4; i++) {
        s_prev_angle[i] = zdt_angle[s_wheel_axis[i]];
        s_rpm_lp[i]     = 0.0f;
    }
}

/* CAN1 RX0 中断分发（唯一 CAN 接收回调）：中断里只做搬运，解析在 6020.c（整数拷贝，不做浮点）；CAN1 上再挂设备按 StdId 分路。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Grab6020_CanRx(hcan);
}

/* 调度器启动前的底盘初始化（main.c 调用）：须在 osKernelInitialize 之后、osKernelStart 之前，让调度器一跑起来就能动；内容 INS/Go/PID 清零 + GRAB(CAN/舵机) + M27 + See(USART3)。 */
void Motion_PreInit(void)
{
    INS_Init(&s_ins);
    GO_Init(&s_ins);         /* 绑定位姿 */
    GO_Stop();               /* 上电先冻住：第一次下发目标前四轮不出力 */
    PID_Init();

    GRAB_Init();             /* CAN1 滤波器必须在 HAL_CAN_Start 之前配好 */
    /* 伸缩/升降：只清状态；0cm 基准等 MotorC 首次读到 0x36 时才取（那时才有角度） */
    M27_Init();
    /* 摄像头/扫码：建信号量 + 启动 USART3（PD8/PD9）单字节接收（越早越好，别丢头几帧） */
    See_Init();
    /* 路径规划：按左/右出发把 52 个点宏拆进 x[]/y[]（不调它 x/y 全是 0）
     * 出发边由上面的 follow_side_key 决定（Follow_SetSide = 设值 + 重拆表）*/
    Follow_SetSide((uint8_t)follow_side_key);
}

/* USER CODE END Application */

