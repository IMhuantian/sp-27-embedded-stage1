# SuperPower 27 赛季校内赛 · 嵌入式方向 第一阶段

同济大学 SuperPower 战队 2027 赛季校内赛「嵌入式方向第一阶段」的个人考核工程。

主控：**RoboMaster 开发板 C 型**（STM32F407IGHx，168 MHz）
中间件：[sp_middleware](https://github.com/TongjiSuperPower/sp_middleware)（`sp::` 命名空间，以 git submodule 引入）
工具链：STM32CubeMX + STM32CubeCLT(CMake/Ninja/arm-none-eabi-gcc) + VS Code(Cortex-Debug) + OpenOCD

---

## 一、主要任务

按《SuperPower × MPS杯 暨 27赛季机甲大师校内赛规则手册》4.4.1 的要求，第一阶段个人考核包含四项：

| 编号 | 任务 | 状态 |
|---|---|---|
| 4.4.1.1 | **C板基本功能**（晋级必要条件）：蜂鸣器提示音 / LED 流水灯 / 串口打印 IMU 三轴数据 / DT7 遥控器通讯 | 本仓库已完成 |
| 4.4.1.2 | 姿态与电机联动（右拨杆三档 + 双 GM6020 比例联动） | 未开始 |
| 4.4.1.3 | 24V→5V 降压模块（MP4420A，嘉立创EDA 原理图 + PCB） | 未开始 |
| 4.4.1.4 | 软件与版本管理（git + GitHub 仓库 + README + 可回溯的提交历史） | 本仓库即本项成果 |

### 已完成部分（4.4.1.1）

| 要求（原文） | 实现 |
|---|---|
| 蜂鸣器控制：成功烧录或 c板上电后需要有蜂鸣器提示音 | `applications/buzzer_task.cpp`，上电响三声（5 kHz） |
| LED控制：c板led灯需亮起流水灯灯效，并能够以此判断程序是否堵塞 | `applications/led_task.cpp`，红→绿→蓝循环流水；灯停住即说明程序被堵塞 |
| 串口打印：需在上位机软件中打印 imu 三轴数据 | `applications/imu_task.cpp` 采集 + `applications/plotter_task.cpp` 经 USART1 输出 acc/gyro/姿态角，上位机用 SerialPlot 观察 |
| 遥控器控制：使 c板与 dt7 遥控器能够正常通讯 | `applications/uart_task.cpp`，USART3 + DMA 空闲中断接收，`sp::DBus` 解析 |
| （既有，第三讲内容）CAN 通信验证 | `applications/can_task.cpp`，GM6020(ID=1) 电流控制 |

---

## 二、目录结构

```
.
├── applications/               # 应用层任务（CubeMX 中以 "As external" 注册）
│   ├── buzzer_task.cpp         # 蜂鸣器：上电提示音
│   ├── led_task.cpp            # RGB 流水灯（可判断程序堵塞）
│   ├── imu_task.cpp            # BMI088 采集 + Mahony 姿态解算（1 kHz）
│   ├── uart_task.cpp           # DT7 遥控器(DBus) 接收
│   ├── plotter_task.cpp        # 串口打印 IMU 数据（SerialPlot）
│   └── can_task.cpp            # CAN + GM6020 电机
├── Src/ Inc/                   # CubeMX 生成的 HAL 初始化与外设配置
│   └── freertos.c              # FreeRTOS 任务创建（由 CubeMX 生成，勿手改）
├── Drivers/ Middlewares/       # ST HAL / CMSIS / FreeRTOS / USB 库
├── cmake/                      # 工具链与 CubeMX 子工程
├── sp_middleware/              # 电控中间件（git submodule）
├── cboard.ioc                  # CubeMX 工程文件（外设与 FreeRTOS 配置的唯一来源）
├── CMakeLists.txt              # 顶层构建脚本（手工维护，CubeMX 不会覆盖）
├── CMakePresets.json           # Debug / Release 预设
├── openocd.cfg                 # 烧录器与芯片型号
└── STM32F407XX_FLASH.ld        # 链接脚本
```

---

## 三、环境依赖

1. **STM32CubeMX** ≥ 6.11（需安装 STM32F4 固件包）
2. **STM32CubeCLT** ≥ 1.15（提供 CMake / Ninja / arm-none-eabi-gcc）
   - 自检：`cmake --version`、`ninja --version`、`arm-none-eabi-gcc --version`
3. **VS Code** ≥ 1.92 + 插件：C/C++、CMake Tools、Cortex-Debug
4. **OpenOCD** ≥ 20231002（解压到 C 盘根目录，把 `bin` 加入 PATH）
   - 自检：`openocd --version`
5. **SerialPlot** v0.13.0（用于串口打印上位机）
6. **Git**

---

## 四、获取与构建

### 1. 克隆（**必须带 `--recursive`**，否则 `sp_middleware` 是空目录）

```bash
git clone --recursive <本仓库地址>
# 已经普通 clone 过的话：
git submodule update --init --recursive
```

### 2. 配置与编译

用 VS Code 打开项目文件夹：

- 首次打开时选择 **Debug** preset；
  错过了就 `Ctrl+Shift+P` → `cmake: select configure preset` → **Debug**
- `F7` 编译
- `Ctrl+Shift+B` 烧录
- `F5` 调试（Cortex-Debug，支持 live watch）

命令行等价操作：

```bash
cmake --preset Debug
cmake --build build/Debug
```

> **注意**：若 `build/` 里的缓存是在旧路径或旧配置下生成的，会报
> `The current CMakeCache.txt directory ... is different than the directory ... where CMakeCache.txt was created`，
> 此时执行 `Ctrl+Shift+P` → `CMake: Delete Cache and Reconfigure`（或直接删掉 `build/` 目录）后重新配置。

### 3. 烧录器配置

`openocd.cfg` 里选择烧录器与芯片：

```tcl
# source [find interface/stlink.cfg]
source [find interface/cmsis-dap.cfg]    # 无线烧录器 / DAPLink 走这一行
source [find target/stm32f4x.cfg]
```

按实际烧录器修改对应的一行即可。

---

## 五、外设配置对照表

以下配置全部在 `cboard.ioc` 中维护，**不要在 `Src/` 下手改**（下次 Generate 会被覆盖）。

| 功能 | 外设 | 引脚 | 关键参数 | 代码对象 |
|---|---|---|---|---|
| RGB LED | TIM5 CH1/2/3 | PH10 / PH11 / PH12 | PSC=0, ARR=65535 | `sp::LED led(&htim5)` |
| 蜂鸣器 | TIM4 CH3 | PD14 | PSC=167, ARR=65535（计数 500 kHz） | `sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6)` |
| DT7 遥控器 | USART3 | PC10=TX / PC11=RX | 100000 bps / 偶校验 / 9bit，RX 走 DMA1_Stream1 | `sp::DBus remote(&huart3)` |
| 串口打印 | USART1 | PA9=TX / PB7=RX | 921600 bps，TX/RX 走 DMA | `sp::Plotter plotter(&huart1)` |
| IMU (BMI088) | SPI1 | PB3=SCK / PB4=MISO / PA7=MOSI | 10.5 Mbit/s, CPOL=High, CPHA=2Edge | `sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab)` |
| IMU 片选 | GPIO | PA4(加速度计) / PB0(陀螺仪) | GPIO_Output | 同上 |
| 电机 CAN | CAN1 | PD0=RX / PD1=TX | PSC=3, BS1=10TQ, BS2=3TQ → 1 Mbps | `sp::CAN can1(&hcan1)` |
| 上位机 USB | USB_OTG_FS | PA11=DM / PA12=DP | Device CDC | `MX_USB_DEVICE_Init()` |

时钟：HSE 12 MHz，PLLM=6 / PLLN=168 / PLLQ=7 → SYSCLK 168 MHz，APB1 定时器 84 MHz。

### FreeRTOS 任务

| 任务 | 优先级 | 栈(字) | 入口 | 说明 |
|---|---|---|---|---|
| defaultTask | Normal | 128 | `StartDefaultTask` | CubeMX 默认任务，负责 USB 初始化 |
| ImuTask | AboveNormal | 512 | `imu_task` | 1 kHz 采集 + 姿态解算，优先级最高 |
| UartTask | Normal | 256 | `uart_task` | DBus 接收请求（实际收数在中断里） |
| PlotterTask | Low | 128 | `plotter_task` | 串口打印，1 kHz |
| CanTask | Low | 256 | `can_task` | CAN 收发 + 电机控制 |
| LedTask | Low | 128 | `led_task` | 流水灯 |
| BuzzerTask | Low | 128 | `buzzer_task` | 提示音 |

在 CubeMX 的 `Middleware → FREERTOS → Tasks and Queues` 中新增任务时，
**Code Generation Option 必须选 `As external`**，这样函数体留在 `applications/*.cpp`，
CubeMX 只生成 `extern` 声明与创建语句（`Src/freertos.c`）。

---

## 六、操作注意事项

### ⚠️ 安全（规则手册 6.2）

1. **控制任一电机前，必须先验证 DT7 右拨杆下档能让全部电机可靠失能。**
2. 烧录、接线、拆装或人工转动机构前，**切断动力电源**。
3. 新增或修改代码后首次测试，应使车轮离地或采取等效约束，并先把输出限幅设低。
4. 出现失控、异常噪声、异味、过热、冒烟、线束拉扯或结构松动时，立即失能并断电。
5. 首次通电前检查电源极性、线束绝缘、连接器规格、短路风险。

### 工程注意

1. **`Src/freertos.c`、`Src/*.c`、`Inc/*.h` 都是 CubeMX 生成物**，改动会丢失。自定义代码写在 `USER CODE BEGIN/END` 之间，或放到 `applications/`。
2. 新增 `applications/*.cpp` 后，必须在顶层 `CMakeLists.txt` 的 `target_sources` 中登记，**并同步登记它用到的 `sp_middleware/**/*.cpp`**；`target_include_directories` 需包含 `sp_middleware/`。
   顶层 `CMakeLists.txt` 只生成一次，CubeMX 不会覆盖；但 `cmake/stm32cubemx/CMakeLists.txt` 会被重新生成。
3. **USART1 上只能有一个 `sp::Plotter` 在发帧**，否则两帧交错成乱码。当前统一由 `plotter_task` 打印 IMU。
4. `imu_task.cpp` 中的 `r_ab` 是 BMI088 载体系到机体系的旋转矩阵，**装配方向一变就必须改**，否则 yaw/pitch/roll 全错。确定方法见 `sp_middleware/io/bmi088/readme.md`。
5. `sp::Mahony` 的 `dt` 必须与实际调用周期一致（现为 `osDelay(1)` + 1000 Hz tick → `1e-3f`）。
6. FreeRTOS 堆 `configTOTAL_HEAP_SIZE = 20000`（字节）。当前 7 个任务栈合计约 6 KB，余量充足；继续增加任务时留意。
7. `sp_middleware` 是 submodule，提交前确认其指针版本是预期的提交。

---

## 七、参考资料

- 《SuperPower × MPS杯 暨 27赛季机甲大师校内赛规则手册》 4.4 / 6.2 / 7.2 节
- RoboMaster 开发板 C 型用户手册 / 原理图
- RoboMaster GM6020 直流无刷电机使用说明
- RoboMaster 机器人专用遥控器（接收机）用户手册
- 队内 STM32 开发环境安装与使用教程
- `sp_middleware` 各模块 `readme.md`

---

## 八、后续计划

- [ ] 4.4.1.2 姿态与电机联动：双 GM6020、右拨杆三档状态机（失能/联动/复位）、左拨杆比例（下档 1:0.5、中档 1:-1、上档 1:3）、手动拖动后重算参考零点
- [ ] 4.4.1.3 24V→5V 降压模块（MP4420A，嘉立创EDA）原理图与 PCB
- [ ] 完善蜂鸣器自定义音乐
- [ ] SerialPlot 实测 IMU 三轴波形并记录
