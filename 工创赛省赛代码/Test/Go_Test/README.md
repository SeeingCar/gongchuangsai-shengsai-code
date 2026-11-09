# Go_Test —— 四轮正转,定 `WHEEL_DIR` 的符号

## 这是干什么的

上电后**不进 FreeRTOS**(StartTask / TaskFlow / Grab 全都不启动),只做一件事:

> 让 M1~M4 四个轮子都按 `dir=0`(项目里的"正 rpm")转 2 秒、停 3 秒,循环。

然后对照**串口数据**和**现实里车走的方向**,判断符号对不对:

| 串口里四个 `angle` | 现实里车 | 结论 |
|---|---|---|
| 一起往**正**方向涨 | 往**前**走 | 符号对 → `chassis_cfg.h` 的 `WHEEL_DIR0..3` 保持 `+1.0f` |
| 一起往**正**方向涨 | 往**后**走 | 反了 → 把 `WHEEL_DIR0..3` 全改成 `-1.0f` |
| 有的涨有的跌 | 原地打转/斜着走 | 电机接线或轮序不对,先查线 |

`dir=0` 这个约定来自正式固件 `Core/Src/freertos.c` 的 `MotorA`:

```c
uint8_t dir = (rpm < 0.0f) ? 1 : 0;      /* 正 rpm → dir=0 */
ZDT_MMCL_AddSpeed(ZDT_BUS_A, addr, dir, sp, MOTOR_ACCEL);
```

## 怎么开 / 怎么关

只改一个地方 —— `Test/Go_Test/go_test.h`:

```c
#define GOTEST_ON   1     /* 1 = 上电跑这个测试;0 = 正常固件 */
```

`main.c` 里只加了两行(已在工程里做好):顶部 `#include ".../go_test.h"`,以及
`ZDT_MotorInit(); HWT601_Init();` 之后调用 `Go_Test_Run();`。

`GOTEST_ON=0` 时 `Go_Test_Run()` 是个**空函数**,立刻返回,后面
`osKernelInitialize()` → `MX_FREERTOS_Init()` → `osKernelStart()` 照常启动,固件行为和以前完全一样。

> 也就是说"把 StartTask 注释掉"这件事,是靠 `GOTEST_ON` 完成的;**不需要真的去注释 `freertos.c`**。
> 如果你更想手动来:`main.c` 里把 `osKernelInitialize(); MX_FREERTOS_Init(); osKernelStart();`
> 三行注释掉、换成 `Go_Test_Run();` 也行,效果一样。

## Keil 工程

`MDK-ARM/hard.uvprojx` 里已经加了一个 `Test/Go_Test` 分组(`go_test.c` + `go_test.h`)。
**如果 Keil 刚才开着工程**,它保存时会用内存里的版本覆盖磁盘 —— 那就手动加一下:

1. Keil 左侧 Project 窗口 → 右键某个分组 → `Add Group...` → 命名 `Test/Go_Test`
2. 右键新分组 → `Add Existing Files to Group...` → 选到 `..\Test\Go_Test\go_test.c` → Add
3. 编译

## 上电前请注意

- 第一次测:把车**架空**(轮子离地)或至少**手扶电闸**,因为它是持续"走 2 秒 / 停 3 秒"循环的。
- 串口接 **USART1(115200)**,就是平时看遥测那个口。

## 串口会看到

```
=== Go_Test:四轮正转 dir=0 ===
轮序 M1右前 M2左前 M3左后 M4右后
看串口:四个 angle 应一起往正方向涨。看现实:车往前还是往后?
使能后    angle M1..M4 =      0.0      0.0      0.0      0.0
--- 正转 60rpm ---
走        angle M1..M4 =    180.0    180.0    180.0    180.0
走        angle M1..M4 =    360.0    360.0    360.0    360.0
--- 停 ---
停        angle M1..M4 =    360.0    360.0    360.0    360.0
```

## 测完回报什么

1. 串口这几行(尤其 `走` 那几行的四个 angle 是涨还是跌、四个是不是一致);
2. 现实里车头是**往前**还是**往后**;
3. 顺手看一下:四个轮子是不是都真的在转(有没有哪个不转/反转)。

拿到这两条我就能告诉你 `WHEEL_DIR0..3` 该填什么(以及要不要顺便定 `WHEEL_RADIUS_CM`)。
