# Material Cart

物料搬运小车项目资料仓库，包含嵌入式控制程序、机械结构模型和 PCB 制板文件。

本项目面向竞赛机器人/物料搬运车场景，主要功能包括麦克纳姆轮底盘运动控制、Z 轴升降、舵机/机械臂抓取、视觉定位、二维码任务解析和按颜色/位置完成抓取放置流程。

## 仓库内容

```text
material-cart/
├── gc-z7-code/     STM32F407 控制程序与 Keil 工程
├── model/          SolidWorks 机械装配体与零件模型
├── 制板文件/       PCB Gerber 压缩包
└── README.md       项目说明
```

## 嵌入式工程

控制程序位于 `gc-z7-code/`，使用 STM32CubeMX 生成，目标芯片为 `STM32F407VETx`，工具链为 Keil MDK-ARM。

常用入口：

| 文件/目录 | 说明 |
|---|---|
| `gc-z7-code/MDK-ARM/gc.uvprojx` | Keil uVision 工程文件 |
| `gc-z7-code/gc.ioc` | STM32CubeMX 配置文件 |
| `gc-z7-code/Core/Inc/` | 用户头文件 |
| `gc-z7-code/Core/Src/` | 用户源文件 |
| `gc-z7-code/Drivers/` | STM32 HAL 与 CMSIS 驱动 |
| `gc-z7-code/MDK-ARM/gc/` | Keil 编译输出目录 |

### 打开与编译

1. 安装 Keil MDK-ARM。
2. 打开 `gc-z7-code/MDK-ARM/gc.uvprojx`。
3. 选择目标工程后编译。
4. 通过 ST-Link / SWD 下载到 STM32F407 控制板。

当前仓库未配置命令行构建脚本，建议直接使用 Keil uVision 编译和烧录。

### 主要模块

| 模块 | 作用 |
|---|---|
| `Motor_Move.c/.h` | 底盘运动、麦克纳姆轮解算、Z 轴升降控制 |
| `GC_Chassis_Control.c/.h` | 底盘 PID、定距直行、原地转向、视觉定位控制 |
| `PID.c/.h` | 通用 PID 算法 |
| `zhangdatou.c/.h` | 步进电机驱动通信 |
| `HWT101.c/.h` | HWT101 IMU 姿态角读取 |
| `serial.c/.h` | 视觉模块串口通信 |
| `LobotServoController.c/.h` | 幻尔串口舵机控制 |
| `huaner_servo.c/.h` | 舵机预设动作 |
| `ServoMotorControl.c/.h` | 视觉辅助的舵机精定位 |
| `QRcode.c/.h` | 二维码任务读取 |
| `action.c/.h` | 比赛动作流程编排 |

## 机械模型

机械模型位于 `model/`，主要包含 SolidWorks 装配体和零件文件：

| 类型 | 扩展名 | 说明 |
|---|---|---|
| 装配体 | `.SLDASM` | 小车整体、局部机构装配 |
| 零件 | `.SLDPRT` | 结构件、支架、轮组、机械臂等 |
| 通用模型 | `.stp` | 可跨 CAD 软件导入的模型文件 |

建议使用 SolidWorks 打开根装配体，例如 `model/1.SLDASM` 或 `model/xyz版.SLDASM`。

## 制板文件

PCB 制板文件位于 `制板文件/`，目前包含多个 Gerber 压缩包：

| 文件 | 说明 |
|---|---|
| `Gerber_主板.zip` | 主控板制板文件 |
| `Gerber_openmv转接板.zip` | OpenMV 转接板制板文件 |
| `Gerber_塔吊转接板.zip` | 塔吊/机构转接板制板文件 |
| `Gerber_幻尔转接板.zip` | 幻尔舵机/模块转接板制板文件 |
| `Gerber_烧录转接板.zip` | 烧录转接板制板文件 |

下单制板前建议先用 Gerber 查看器检查层定义、孔位、板框和丝印。

## 硬件与通信简表

| 外设/接口 | 用途 |
|---|---|
| USART1 | 步进电机驱动通信 |
| USART2 | HWT101 IMU |
| UART5 | 视觉模块 / 蓝牙调试通信 |
| USART6 | 幻尔串口舵机 |
| TIM2 | 1 ms 定时中断 |
| TIM3 | PWM 舵机输出 |
| PE7 | 限位开关输入 |

## 使用建议

- 修改 STM32 外设配置时，优先编辑 `gc-z7-code/gc.ioc` 并通过 STM32CubeMX 重新生成。
- 自定义代码应尽量放在 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 区域内，避免重新生成时被覆盖。
- 机械模型包含较多中文文件名，跨系统传输时注意编码和路径兼容性。
- 当前仓库保留了部分 Keil 编译产物，便于复现原始工程状态；后续如果只维护源码，可以再添加 `.gitignore` 精简仓库。

## 许可

仓库当前未声明开源许可证。未经项目成员确认前，请勿默认用于商业发布或二次分发。
