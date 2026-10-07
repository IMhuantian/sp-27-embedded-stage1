# cboard

RoboMaster 开发板 C 型（STM32F407IGHx）上的嵌入式工程。

- 蜂鸣器提示音
- RGB 流水灯（也能用来判断程序有没有堵塞）
- IMU 数据采集 + Mahony 姿态解算；三轴姿态角（roll / pitch / yaw）经串口打印到 SerialPlot
  （USART1 / 921600 / 帧头 `AA BB`）
- DT7 遥控器通讯
- 两台 GM6020 的姿态与电机联动

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

## 参考

- RoboMaster 开发板 C 型用户手册、原理图
- RoboMaster GM6020 直流无刷电机使用说明
- RoboMaster 机器人专用遥控器（接收机）用户手册
- 队内 STM32 开发环境安装与使用教程
- `sp_middleware` 各模块的 readme
