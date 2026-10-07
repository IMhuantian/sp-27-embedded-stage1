# cboard

RoboMaster 开发板 C 型（STM32F407IGHx）上的嵌入式工程。

队内第三讲给的工程模板和 `sp_middleware` 中间件是基础，我在上面写了应用层的几个任务，
把嵌入式方向第一阶段要求的功能做完了：

- 蜂鸣器提示音
- RGB 流水灯（也能用来判断程序有没有堵塞）
- IMU 数据采集 + Mahony 姿态解算；三轴姿态角（roll / pitch / yaw）经串口打印到 SerialPlot
  （USART1 / 921600 / 帧头 `AA BB`）
- DT7 遥控器通讯
- 两台 GM6020 的姿态与电机联动

## 目录结构

```
.
├── applications/        # 应用层任务
│   ├── led_task.cpp        流水灯
│   ├── buzzer_task.cpp     上电提示音
│   ├── imu_task.cpp        BMI088 采集 + 姿态解算
│   ├── uart_task.cpp       DT7 遥控器接收
│   ├── plotter_task.cpp    串口打印
│   └── can_task.cpp        CAN 收发 + 姿态与电机联动
├── Src/ Inc/            # CubeMX 生成的 HAL 初始化（freertos.c 在这里）
├── Drivers/ Middlewares/
├── cmake/               # 工具链与 CubeMX 子工程
├── sp_middleware/       # 电控中间件（submodule）
├── cboard.ioc           # CubeMX 工程文件，外设和 FreeRTOS 配置都在这里改
├── CMakeLists.txt       # 顶层构建脚本
├── CMakePresets.json
├── openocd.cfg          # 烧录器和芯片型号
└── STM32F407XX_FLASH.ld
```

## 开发环境

- STM32CubeMX 6.11 以上（要装 STM32F4 固件包）
- STM32CubeCLT 1.15 以上，自带 CMake / Ninja / arm-none-eabi-gcc
- VS Code + C/C++、CMake Tools、Cortex-Debug 三个插件
- OpenOCD 20231002 以上
- SerialPlot 0.13.0（看波形用）

## 编译和烧录

先把 submodule 拉下来，不然 `sp_middleware` 是空的：

```bash
git clone --recursive <仓库地址>

# 已经普通 clone 过的
git submodule update --init --recursive
```

用 VS Code 打开工程目录，第一次会提示选 preset，选 Debug（错过了就 `ctrl+shift+p` 输入
`cmake: select configure preset`）。

- `f7` 编译
- `ctrl+shift+b` 烧录
- `f5` 调试（Cortex-Debug，可以用 live watch）

命令行的话：

```bash
cmake --preset Debug
cmake --build build/Debug
```

烧录器在 `openocd.cfg` 里选。我现在用的是 CMSIS-DAP（无线烧录器）：

```tcl
source [find interface/cmsis-dap.cfg]
source [find target/stm32f4x.cfg]
```

## 外设配置

都在 `cboard.ioc` 里，不要在 `Src/` 下面手改，下次 CubeMX 生成会被覆盖。

| 功能 | 外设 | 引脚 | 参数 |
|---|---|---|---|
| RGB LED | TIM5 CH1/2/3 | PH10 / PH11 / PH12 | PSC=0, ARR=65535 |
| 蜂鸣器 | TIM4 CH3 | PD14 | PSC=167, ARR=65535 |
| DT7 遥控器 | USART3 | PC10 / PC11 | 100000bps 偶校验，RX 走 DMA1_Stream1 |
| 串口打印 | USART1 | PA9 / PB7 | 921600 |
| IMU | SPI1 | PB3 / PB4 / PA7 | 10.5 Mbit/s，片选 PA4 / PB0 |
| 电机 | CAN1 | PD0 / PD1 | 1 Mbps |
| USB | USB_OTG_FS | PA11 / PA12 | CDC |

时钟：HSE 12MHz，PLLM=6 / PLLN=168 / PLLQ=7，SYSCLK 168MHz。

## 任务

| 任务 | 优先级 | 栈(字) | 入口 | 干什么 |
|---|---|---|---|---|
| defaultTask | Normal | 128 | StartDefaultTask | USB 初始化 |
| ImuTask | AboveNormal | 512 | imu_task | 1kHz 采 IMU 并解算 |
| UartTask | Normal | 256 | uart_task | DBus 接收 |
| PlotterTask | Low | 128 | plotter_task | 1kHz 串口打印 |
| CanTask | Low | 256 | can_task | CAN 收发 + 联动控制 |
| LedTask | Low | 128 | led_task | 流水灯 |
| BuzzerTask | Low | 128 | buzzer_task | 提示音 |

在 CubeMX 里加任务的时候，Code Generation Option 要选 `As external`，这样函数体留在 `applications/`
下面，CubeMX 只生成 extern 声明和创建语句。

## 姿态与电机联动

三个旋转输入端是同轴的，统一用"绕 yaw 轴逆时针为正"的机械角来描述：

```
phi_C = YAW_DIR * yaw - psi0               C 板
phi_A = A_DIR * (motorA.angle - a0)        A 电机
phi_B = B_DIR * (motorB.angle - b_ref0)    B 电机
```

`a0` / `b0` 是上电时两台电机的读数，也就是三个指向标箭头对齐的位置；`b_ref0` 是 B 的比例基准，
换左拨杆档位时会重钉一次，保证换比例本身不产生位移。

右拨杆中档时按 `phi_A = phi_C`、`phi_B = k * phi_C` 联动，`k` 由左拨杆定（下档 +0.5、中档 -1、上档 +3），
换算回电机角度就是 `ref_a = a0 + A_DIR * phi_C`、`ref_b = b_ref0 + B_DIR * k * phi_C`，再交给位置环出力矩。

手动拖动电机的时候，只要 C 板基本没动、而某个电机的位置偏差超过阈值，就认为它被手拖了。
这时重算参考零点、并把那个电机的目标贴到它当前的位置上，所以不会有回正的力矩：

- 拖 A：`psi0 = mech_yaw - phi_A`，零点跟着 A 走，然后 B 按 `k` 跟
- 拖 B：`psi0 = mech_yaw - phi_B / k`，然后 A 按 `1/k` 跟

再转 C 板是从新位置继续联动，不会回原来的零位。

右拨杆三个档：下档发 0 电流让电机无力，中档联动，上档复位——两台电机都开到和 C 板当前
指向一致的位置（这一下按 1:1 走），三个箭头重新指同一个方向，到位后把这儿定成新的零位，
切回中档从新零位接着联动。

零位只在电机第一次上来反馈的时候标定，之后遥控器断连不会重标；遥控器失联或者电机没有
反馈的时候一律失能。

## 注意事项

安全方面：

1. 动电机之前，先确认 DT7 右拨杆下档能让所有电机可靠失能
2. 烧录、接线、拆装、人工转机构之前，把动力电源断掉
3. 改完代码第一次测试，把机构固定住或者让轮子离地，输出限幅先给低一点
4. 出现失控、异响、异味、过热、冒烟、线束被拉扯、结构松动，立刻失能断电

工程方面：

1. `Src/`、`Inc/` 下面都是 CubeMX 生成的文件，手改会丢。自己的代码写在 `applications/` 里，
   或者写在 USER CODE BEGIN/END 之间
2. 新加 `applications/*.cpp` 之后，要在顶层 `CMakeLists.txt` 的 `target_sources` 里登记它自己
   **和它用到的 `sp_middleware` 源文件**；`cmake/stm32cubemx/CMakeLists.txt` 会被 CubeMX 重新生成，别往那里改
3. `huart1` 上只能有一个 Plotter 发帧，两个任务同时发会交错成乱码，所以 IMU 的打印统一放在
   `plotter_task` 里。SerialPlot 端：波特率 921600，Custom Frame / 帧头 `AA BB` / float /
   Little Endian / 不勾 Checksum，Frame Size 选 First byte of the payload is size
   （= 通道数 x 4，现在是 `0x0C`，一帧 15 字节）
4. `imu_task.cpp` 里的 `r_ab` 是 BMI088 到机体系的旋转矩阵，板子装法变了就得改
5. Mahony 的 `dt` 要跟实际调用周期一致（现在是 1ms）
6. FreeRTOS 堆是 20000 字节，7 个任务的栈加起来大概 6KB，够用；再加任务的时候留意一下
7. 两台 GM6020 的 ID 都小于 5，控制帧 ID 都是 `0x1FE`，两条指令必须写进同一帧再发
8. 位置环的 `KI` 现在是 0：带积分的话手动拖动会积累积分量，松手会弹回去
9. `sp_middleware` 是 submodule，提交前确认指针指向的版本是对的

## 还需要做的

- 上机整定：确认 `YAW_DIR` / `A_DIR` / `B_DIR` 的方向，调 `POS_KP` 和 `POS_KD`，
  把右拨杆三个档位和手动拖动都验一遍

## 参考

- RoboMaster 开发板 C 型用户手册、原理图
- RoboMaster GM6020 直流无刷电机使用说明
- RoboMaster 机器人专用遥控器（接收机）用户手册
- 队内 STM32 开发环境安装与使用教程
- `sp_middleware` 各模块的 readme
