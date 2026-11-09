# 工创赛省赛代码(ST-LINK / STM32F407 固件)

2027 年第十届中国大学生工程实践与创新能力大赛「智能搬运」赛项的省赛固件工程。

- **MCU**:STM32F407ZGTx(LQFP144)
- **框架**:STM32Cube HAL + FreeRTOS(CMSIS-RTOS v2),控制节拍 `CTRL_DT_MS = 20` ms
- **工程**:Keil MDK(`MDK-ARM/hard.uvprojx`)、IAR EWARM(`EWARM/Project.eww`)、CubeMX(`hard.ioc`)
- **机构**:四轮底盘(ZDT 闭环步进 ×4)+ 四自由度手臂(GM6020 整臂旋转、伸缩/升降步进、夹爪与转盘舵机)

## 目录结构

| 路径 | 内容 |
| --- | --- |
| `Core/` | CubeMX 生成的外设初始化:时钟、GPIO、CAN、UART、TIM、中断、FreeRTOS 任务创建 |
| `GroundFloor/` | **底盘**:任务流程状态机、路线走位、位置环、里程计、PID、IMU、步进电机驱动 |
| `Grab/` | **手臂**:抓取状态机、相机/二维码解析、GM6020 驱动、伸缩与升降、PWM 舵机 |
| `Drivers/` | ST HAL 与 CMSIS |
| `Middlewares/` | FreeRTOS 源码与 CMSIS-RTOS v2 适配层 |
| `Test/Go_Test/` | 台架走位/标定测试(`GO_TEST_FOLLOWSEQ`) |
| `MDK-ARM/` | Keil 工程、启动文件、调试配置 |
| `EWARM/` | IAR 工程与链接脚本 |
| `refs/` | 外设手册(GM6020 使用说明) |
| `hard.ioc` / `.mxproject` | STM32CubeMX 工程配置 |
| `规则.md` | 开发红线与验收约定(**改动代码前先读**) |
| `交接_给小模型_必读.md` | 929 行完整交接手册:文件地图、参数、流程、硬件接线、通信协议、经验教训 |
| `小模型提示词.md` | 交接用的提示词模板 |

## 关键模块

| 文件 | 职责 |
| --- | --- |
| `Core/Src/freertos.c` | 总调度:9 个 FreeRTOS 任务、20 ms 控制拍、串口遥测、电机收发、所有功能开关 |
| `GroundFloor/TaskFlow.c` | 底盘状态机 `TaskFlow_Update()`,按 `Go_now` 推进比赛流程 |
| `GroundFloor/Follow.c` | 按格走位 + 前置段,`CELL_L*` / `CELL_R*` 为 25 个格点坐标 |
| `GroundFloor/Go.c` | 底盘位置环与到位判据 |
| `GroundFloor/ins.c` | 四轮 rpm ↔ 车体速度 ↔ 世界系积分 |
| `Grab/Grab.c` | 手臂状态机 `GRAB_Update()` |
| `Grab/See.c` | 相机物料帧、二维码帧解析与颜色历史判定 |
| `Grab/6020.c` / `Grab/M27.c` / `GroundFloor/zdt_motor.c` | GM6020、伸缩/升降、底盘步进驱动 |

## 硬件接口

| 外设 | 用途 | 参数 |
| --- | --- | --- |
| USART1 (PA9/PA10) | 调试/遥测输出 + 串口调参 | 115200 8N1 |
| USART2 | ZDT 步进总线 A:轴 1(右后)、4(左后) | 115200 8N1 |
| USART3 (PD8/PD9) | 相机 + 扫码 + 上位机 | 115200 8N1 |
| UART4 | ZDT 步进总线 B:轴 3(右前)、6(左前) | 115200 8N1 |
| UART5 | ZDT 步进总线 C:轴 2(伸缩 FB)、7(升降 UD) | 115200 8N1 |
| USART6 (PC6/PC7) | HWT601 陀螺仪(Modbus,地址 0x50) | 115200 8N1 |
| CAN1 (PB8/PB9) | GM6020 整臂旋转(ID=1) | 1 Mbps,发 `0x1FF`,收 `0x205` |
| TIM4 CH1/CH2 | 夹爪舵机 / 转盘舵机 | 50 Hz |

## 编译

1. Keil MDK 打开 `MDK-ARM/hard.uvprojx`,目标 `hard`,编译下载即可;或 IAR 打开 `EWARM/Project.eww`。
2. 源码无 Makefile,依赖 Keil/IAR 工程;本仓库不含编译产物(`.gitignore` 已排除)。
3. 交接文档中提到的 `_agent/build_verify.sh`(调用 armcc 的 19 文件编译自检)属于原机器上的过程存档,未纳入本仓库。

## 通信协议(摘要)

- 上位机 → 板(USART3):路线帧 `@任务+格号(角)+格号(角)…%`,例 `@1+5(0)+15(-90)%`
- 相机 → 板(USART3):物料帧 `$颜色!cx!cy%`(颜色 1~6);二维码帧 `#AAA+BBB+CCC+DDD&`(12 位:两遍各 3 色 + 3 环)
- 板 → 上位机:`#2*`(左出发)/ `#1*`(右出发)握手
- 串口调参(USART1):`编号-数值`(1~5),回显 `OK 编号=值`
- 遥测(USART1):`K` 角度、`S` 状态、`P` 路线表轮播、`V` 抓取判定(5 s 一次)

> 详细格式与含义见 `规则.md` 与 `交接_给小模型_必读.md` 的 §11、§17。
