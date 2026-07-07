# action 模块说明

本文用于快速理解 `Core/Src/action.c` 和 `Core/Inc/action.h` 的作用、对外接口和在 `GC_task.c` 中的调用方式。

## 1. 模块定位

`action` 模块是比赛动作编排层。

它不直接实现底层电机通信、PID 算法或舵机协议，而是把已有基础动作组合成完整比赛流程：

- 舵机预设动作：来自 `huaner_servo.h`
- Z 轴升降/回零：来自 `Motor_Move.h`
- 视觉找物块：来自 `ServoMotorControl.h`
- 底盘定位移动：来自 `GC_Chassis_Control.h`
- 动作等待：来自 `delay.h`

一句话理解：

```text
二维码任务 -> 视觉找物块 -> 抓到车上槽位 -> 从车上取出 -> 按颜色放地面 -> 再从地面回收到车上
```

## 2. 任务值含义

`action.c` 当前只处理 3 个颜色任务：

```c
#define ACTION_QRCODE_TASK_COUNT 3U
#define ACTION_PLACE_SIDE_OFFSET_MM 152.0f
```

任务值必须是 `1~3`，非法值会被跳过。

地面放置位置映射：

| task | 含义 | 地面 X 目标位置 |
|---|---|---|
| `1` | 红色 / 左侧 | `-152mm` |
| `2` | 绿色 / 中间 | `0mm` |
| `3` | 蓝色 / 右侧 | `+152mm` |

注意：`task1/task2/task3` 表示颜色任务顺序，不直接表示车上槽位。车上槽位由数组下标决定：

```text
第 1 个 task -> 车上槽 1
第 2 个 task -> 车上槽 2
第 3 个 task -> 车上槽 3
```

## 3. 推荐在 GC_task 中调用的接口

这些是 `action.h` 中最适合流程状态机调用的高级接口。

### Action_RunQRCodeTasks

```c
void Action_RunQRCodeTasks(uint8_t task1, uint8_t task2, uint8_t task3);
```

作用：按二维码给出的 3 个颜色任务，依次视觉找物块并抓到车上 1/2/3 槽。

内部流程：

```text
for each task:
  1. 判断 task 是否为 1~3
  2. ServoMotorVision_Calibrate(task) 视觉找对应颜色
  3. 调用 zhuafang1/2/3() 放到对应车槽
```

典型用途：物料盘抓取阶段。

### Action_PlaceLoadedQRCodeTasks1

```c
void Action_PlaceLoadedQRCodeTasks1(uint8_t task1, uint8_t task2, uint8_t task3);
```

作用：把车上 1/2/3 槽的物块取出来，按颜色放到地面左/中/右位置。

内部流程：

```text
for each task:
  1. 从车上第 i 个槽取物块
  2. 根据 task 计算地面 X 位置
  3. FieldPos_Block() 横向移动到目标位置
  4. 使用第一套高度放到地面
```

第一套放置高度中，Z 轴下降较深：

```c
ZAxis_MoveRelative(-13000, 200, 4000);
```

典型用途：第一轮地面放置。

### Action_PlaceLoadedQRCodeTasks2

```c
void Action_PlaceLoadedQRCodeTasks2(uint8_t task1, uint8_t task2, uint8_t task3);
```

作用：和 `Action_PlaceLoadedQRCodeTasks1()` 类似，也是把车上物块放到地面指定颜色位置。

区别：使用第二套放置高度，Z 轴下降较浅：

```c
ZAxis_MoveRelative(-7000, 200, 4000);
```

典型用途：第二轮放置或叠放阶段。

### Action_RecoverGroundQRCodeTasks

```c
void Action_RecoverGroundQRCodeTasks(uint8_t task1, uint8_t task2, uint8_t task3);
```

作用：按二维码顺序，从地面左/中/右位置把物块抓回车上 1/2/3 槽。

内部特点：

- 先用 `Action_GetLastPlaceX()` 推算当前底盘停在最后一次放置位置。
- 再按任务顺序移动到对应地面位置。
- 第一个回收动作走 `Action_GrabRecoverFirstToCarSlot()`。
- 后续回收动作走 `zhuafang8/9/10()`。

典型用途：地面物块回收到车上。

## 4. 暴露的底层动作脚本

这些函数也在 `action.h` 中暴露，但更适合单动作调试，不建议在正式流程里到处直接调用。

| 接口 | 作用 |
|---|---|
| `zhuafang1()` | 地面抓取并放到车上槽 1 |
| `zhuafang2()` | 地面抓取并放到车上槽 2 |
| `zhuafang3()` | 地面抓取并放到车上槽 3，末尾使用 `kanche2()` |
| `zhuafang4()` | 旧版较长组合动作，主流程基本未直接使用 |
| `zhuafang5()` | 槽 2 相关短动作，偏调试 |
| `zhuafang6()` | 槽 3 相关短动作，偏调试 |
| `zhuafang7()` | 视觉/底盘校准前准备动作：Z 轴回零、下降、看地面、松爪、servo13 回零 |
| `zhuafang8()` | 从地面抓回并放到车上槽 1 |
| `zhuafang9()` | 从地面抓回并放到车上槽 2 |
| `zhuafang10()` | 从地面抓回并放到车上槽 3，末尾使用 `kanche2()` |

## 5. 在 GC_task 中的调用方式

`Action_*` 接口都是阻塞式动作。调用后会执行完整机械动作，动作没做完不会返回。

因此在 `GC_task.c` 的 `case` 中调用时，必须调用完立刻切换到下一个状态，避免下一次 `GC_Task_Run()` 重复执行同一动作。

示例：

```c
#include "action.h"

case GC_wuliaopan1:
    Action_RunQRCodeTasks(gc_qr1, gc_qr2, gc_qr3);
    GC_Task_EnterState(GC_kaojinA1);
    break;
```

不推荐这样写：

```c
case GC_wuliaopan1:
    Action_RunQRCodeTasks(gc_qr1, gc_qr2, gc_qr3);
    break;
```

因为状态没有切走，下一次进入 `GC_Task_Run()` 还会再次执行抓取动作。

## 6. 二维码值传入 GC_task 的建议方式

`main.c` 当前可以通过 `QRcode_WaitTaskGroupsAndStop()` 获取 6 个二维码任务。

建议在 `GC_task.c` 中保存这些值：

```c
static uint8_t gc_qr1, gc_qr2, gc_qr3;
static uint8_t gc_qr4, gc_qr5, gc_qr6;

void GC_Task_SetQRCodeTasks(uint8_t t1, uint8_t t2, uint8_t t3,
                             uint8_t t4, uint8_t t5, uint8_t t6)
{
    gc_qr1 = t1;
    gc_qr2 = t2;
    gc_qr3 = t3;
    gc_qr4 = t4;
    gc_qr5 = t5;
    gc_qr6 = t6;
}
```

并在 `GC_task.h` 中声明：

```c
void GC_Task_SetQRCodeTasks(uint8_t t1, uint8_t t2, uint8_t t3,
                             uint8_t t4, uint8_t t5, uint8_t t6);
```

`main.c` 扫码完成后调用：

```c
GC_Task_SetQRCodeTasks((uint8_t)main_qrcode_task1,
                       (uint8_t)main_qrcode_task2,
                       (uint8_t)main_qrcode_task3,
                       (uint8_t)main_qrcode_task4,
                       (uint8_t)main_qrcode_task5,
                       (uint8_t)main_qrcode_task6);
```

之后 `GC_task.c` 中就可以直接使用：

```c
Action_RunQRCodeTasks(gc_qr1, gc_qr2, gc_qr3);
Action_PlaceLoadedQRCodeTasks1(gc_qr1, gc_qr2, gc_qr3);
Action_RecoverGroundQRCodeTasks(gc_qr1, gc_qr2, gc_qr3);

Action_RunQRCodeTasks(gc_qr4, gc_qr5, gc_qr6);
Action_PlaceLoadedQRCodeTasks2(gc_qr4, gc_qr5, gc_qr6);
```

## 7. 可参考的两轮流程

当前 `main.c` 中已有类似流程，可以迁移到 `GC_task.c`：

```text
第一轮 task1~3:
  Action_RunQRCodeTasks
  VisionPrecision_Calibrate
  Action_PlaceLoadedQRCodeTasks1
  Action_RecoverGroundQRCodeTasks
  Action_PlaceLoadedQRCodeTasks1

第二轮 task4~6:
  Action_RunQRCodeTasks
  VisionPrecision_Calibrate
  Action_PlaceLoadedQRCodeTasks1
  Action_RecoverGroundQRCodeTasks
  VisionPrecision_Calibrate
  Action_PlaceLoadedQRCodeTasks2
```

## 8. 接手时优先检查的参数

`action` 模块的大部分问题不是语法错误，而是动作参数和实际机械结构不匹配。

优先检查：

- `ACTION_PLACE_SIDE_OFFSET_MM = 152.0f` 是否对应实际左/中/右间距。
- `ZAxis_MoveRelative()` 的下降/上升脉冲是否安全。
- `servo13_Home(backOffMs)` 的回退时间是否合适。
- `fangche1/2/3()`、`kanche()`、`kanche2()`、`kandi()` 的舵机位置是否和机械臂实际位置一致。
- `Action_PlaceLoadedQRCodeTasks1()` 和 `Action_PlaceLoadedQRCodeTasks2()` 的高度差是否对应第一轮/第二轮需求。

## 9. 总结

正式流程中优先调用：

```c
Action_RunQRCodeTasks(...)
Action_PlaceLoadedQRCodeTasks1(...)
Action_PlaceLoadedQRCodeTasks2(...)
Action_RecoverGroundQRCodeTasks(...)
```

单独调试机械动作时再调用：

```c
zhuafang1() ~ zhuafang10()
```

在 `GC_task.c` 中，每次调用一个 `Action_*` 后都应立即 `GC_Task_EnterState(next_state)`，避免重复执行同一个阻塞动作。
