/*
 * 姿态与电机联动 + CAN 收发
 *
 * 硬件接法：C板 + 两台 GM6020，都在 CAN1 上，电机 ID 分别拨到 1(A) 和 2(B)。
 * 上电前先把三个指向标箭头对到一起，程序会拿上电瞬间的电机读数当机械零位。
 *
 * 三个旋转输入端是同轴的，统一用"绕 yaw 轴逆时针为正"的机械角来描述：
 *     C板转角   phi_C = YAW_DIR * yaw - psi0
 *     A电机转角 phi_A = A_DIR * (motorA.angle - a0)
 *     B电机转角 phi_B = B_DIR * (motorB.angle - b0)
 *
 * 右拨杆中档时按下面的关系联动：
 *     phi_A = phi_C
 *     phi_B = k * phi_C        k 由左拨杆定：下档 +0.5、中档 -1、上档 +3
 *
 * 手动拖动电机的时候要重新算 psi0（见 linkage_step 里的拖动检测），
 * 这样拖完之后再转 C 板是从新位置继续跟，不会自己弹回原来的零位。
 *
 * CAN 这边要注意：两台 GM6020 的 ID 都小于 5，控制帧 ID 都是 0x1FE，
 * 所以两条指令必须写进同一帧里、只 send 一次。
 */

#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// 方向标定：装反了就把对应那一项改成 -1
constexpr float YAW_DIR = +1.0f;
constexpr float A_DIR = +1.0f;
constexpr float B_DIR = +1.0f;

// 左拨杆对应的联动比
constexpr float K_LEFT_DOWN = +0.5f;
constexpr float K_LEFT_MID = -1.0f;
constexpr float K_LEFT_UP = +3.0f;

// 位置环参数，输出单位 N·m（GM6020 上限大概 2.22 N·m）
constexpr float PID_DT = 1e-3f;   // 控制周期 1ms，跟 osDelay(1) 对上
constexpr float POS_KP = 1.2f;
constexpr float POS_KI = 0.0f;    // 先用纯 PD，带积分的话手动拖动会让它松手回弹
constexpr float POS_KD = 0.03f;
constexpr float POS_MAX_OUT = 1.0f;
constexpr float POS_MAX_IOUT = 0.3f;
constexpr float POS_ALPHA = 0.8f;  // D 项滤波系数，1 是不滤波

// 拖动检测
constexpr float DRAG_TOL = 0.20f;        // rad，位置误差超过它才当作是被手拖了
constexpr float CBOARD_STILL = 0.0015f;  // rad/ms，一个周期内 C 板转这么少就算它没动

// 目标的斜率限幅，防止换档或者刚进联动的时候电机猛冲一下
constexpr float REF_RATE = 5.0f;  // rad/s

// 复位到位的判定
constexpr float HOME_TOL = 0.08f;  // rad
}  // namespace

sp::CAN can1(&hcan1);
sp::RM_Motor motorA(1, sp::RM_Motors::GM6020);
sp::RM_Motor motorB(2, sp::RM_Motors::GM6020);

extern sp::DBus remote;  // 在 applications/uart_task.cpp 里
extern sp::Mahony imu;   // 在 applications/imu_task.cpp 里

// 给 plotter_task 看的调试量
float linkage_ref_a = 0.0f;
float linkage_ref_b = 0.0f;
float linkage_phi_c = 0.0f;
float linkage_k = 0.0f;

namespace
{
struct LinkageState
{
  bool inited = false;     // 上电标定做完了没有
  bool armed = false;      // 中档联动是不是已经对齐起点
  float psi0_mech = 0.0f;  // 联动参考零点（机械角）
  float a0 = 0.0f;         // A 电机上电时的读数，也就是箭头对齐位
  float b0 = 0.0f;         // B 电机同上
  float ref_a = 0.0f;      // A 的目标（motorA.angle 空间）
  float ref_b = 0.0f;
  float last_yaw = 0.0f;
  sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::DOWN;
};

LinkageState g;

sp::PID pid_a(PID_DT, POS_KP, POS_KI, POS_KD, POS_MAX_OUT, POS_MAX_IOUT, POS_ALPHA);
sp::PID pid_b(PID_DT, POS_KP, POS_KI, POS_KD, POS_MAX_OUT, POS_MAX_IOUT, POS_ALPHA);

// 每个周期最多走 max_step，把 cur 慢慢逼近 target
float slew(float cur, float target, float max_step)
{
  float d = target - cur;
  if (d > max_step) d = max_step;
  if (d < -max_step) d = -max_step;
  return cur + d;
}

float left_ratio(sp::DBusSwitchMode sw_l)
{
  switch (sw_l) {
    case sp::DBusSwitchMode::DOWN:
      return K_LEFT_DOWN;
    case sp::DBusSwitchMode::MID:
      return K_LEFT_MID;
    default:
      return K_LEFT_UP;
  }
}

// 相对上电对齐位的机械转角
float phi_A() { return A_DIR * (motorA.angle - g.a0); }
float phi_B() { return B_DIR * (motorB.angle - g.b0); }

void linkage_step()
{
  const float mech_yaw = YAW_DIR * imu.yaw;
  const float dyaw = sp::limit_angle(imu.yaw - g.last_yaw);
  g.last_yaw = imu.yaw;

  const uint32_t now = osKernelSysTick();

  // 遥控器失联、或者电机还没上来数据，就一律失能。
  // 另外 sp::DBus 的 sw_r/sw_l 在收到第一帧之前是没初始化的，也要靠这个挡掉
  if (!remote.is_alive(now) || !motorA.is_open() || !motorB.is_open()) {
    motorA.cmd(0.0f);
    motorB.cmd(0.0f);
    pid_a.clear();
    pid_b.clear();
    g.inited = false;
    g.armed = false;
    linkage_ref_a = g.ref_a;
    linkage_ref_b = g.ref_b;
    linkage_phi_c = 0.0f;
    linkage_k = 0.0f;
    return;
  }

  // 上电标定：把当前位置当成箭头对齐的机械零位
  if (!g.inited) {
    g.inited = true;
    g.a0 = motorA.angle;
    g.b0 = motorB.angle;
    g.psi0_mech = mech_yaw;
    g.ref_a = motorA.angle;
    g.ref_b = motorB.angle;
    g.armed = false;
    g.last_sw_r = remote.sw_r;
    pid_a.clear();
    pid_b.clear();
  }

  // 右拨杆换档，清一下状态，免得残留积分和目标跳变
  if (remote.sw_r != g.last_sw_r) {
    g.last_sw_r = remote.sw_r;
    g.armed = false;
    pid_a.clear();
    pid_b.clear();
  }

  const bool cboard_still = (fabsf(dyaw) < CBOARD_STILL);
  const float k = left_ratio(remote.sw_l);
  linkage_k = k;

  // 下档：失能，显式发 0 电流，电机没力、可以随便用手转
  if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
    motorA.cmd(0.0f);
    motorB.cmd(0.0f);
    linkage_ref_a = g.ref_a;
    linkage_ref_b = g.ref_b;
    linkage_phi_c = 0.0f;
    return;
  }

  // 上档：复位，两台电机都回到上电对齐位，指向标箭头重新对上
  if (remote.sw_r == sp::DBusSwitchMode::UP) {
    g.ref_a = slew(g.ref_a, g.a0, REF_RATE * PID_DT);
    g.ref_b = slew(g.ref_b, g.b0, REF_RATE * PID_DT);

    pid_a.calc(g.ref_a, motorA.angle);
    pid_b.calc(g.ref_b, motorB.angle);
    motorA.cmd(pid_a.out);
    motorB.cmd(pid_b.out);

    // 都到位了就把 C 板当前朝向记成新的参考零点，这样切回中档是连着的
    if (fabsf(motorA.angle - g.a0) < HOME_TOL && fabsf(motorB.angle - g.b0) < HOME_TOL) {
      g.psi0_mech = mech_yaw;
    }

    linkage_ref_a = g.ref_a;
    linkage_ref_b = g.ref_b;
    linkage_phi_c = 0.0f;
    return;
  }

  // 中档：姿态与电机联动
  if (!g.armed) {
    // 刚切进来，以 A 的当前位置为起点，避免切换瞬间冲一下
    g.armed = true;
    g.psi0_mech = mech_yaw - phi_A();
    g.ref_a = motorA.angle;
    g.ref_b = motorB.angle;
  }

  // 拖动检测：只有 C 板基本没动的时候，电机的偏差才认为是手拖出来的。
  // C 板自己在转的时候电机也会滞后，那种偏差不能当成拖动
  if (cboard_still) {
    float phi_c = mech_yaw - g.psi0_mech;

    if (fabsf(g.a0 + A_DIR * phi_c - motorA.angle) > DRAG_TOL) {
      // A 被拖了：参考零点跟着 A 的位置走，A 的目标直接贴到它现在的位置上
      g.psi0_mech = mech_yaw - phi_A();
      g.ref_a = motorA.angle;
      phi_c = mech_yaw - g.psi0_mech;
    }
    else if (fabsf(g.b0 + B_DIR * k * phi_c - motorB.angle) > DRAG_TOL) {
      // B 被拖了：反过来由 B 的位置推零点，接着 A 会按 1/k 去跟
      g.psi0_mech = mech_yaw - phi_B() / k;
      g.ref_b = motorB.angle;
    }
  }

  const float phi_c = mech_yaw - g.psi0_mech;

  g.ref_a = slew(g.ref_a, g.a0 + A_DIR * phi_c, REF_RATE * PID_DT);
  g.ref_b = slew(g.ref_b, g.b0 + B_DIR * (k * phi_c), REF_RATE * PID_DT);

  pid_a.calc(g.ref_a, motorA.angle);
  pid_b.calc(g.ref_b, motorB.angle);
  motorA.cmd(pid_a.out);
  motorB.cmd(pid_b.out);

  linkage_ref_a = g.ref_a;
  linkage_ref_b = g.ref_b;
  linkage_phi_c = phi_c;
}
}  // namespace

extern "C" void can_task()
{
  osDelay(500);  // 等电机上电稳定

  can1.config();
  can1.start();

  while (true) {
    linkage_step();

    // 两台电机的控制帧 ID 一样，合成一帧只发一次
    if (motorA.tx_id == motorB.tx_id) {
      motorA.write(can1.tx_data);
      motorB.write(can1.tx_data);
      can1.send(motorA.tx_id);
    }
    else {
      motorA.write(can1.tx_data);
      can1.send(motorA.tx_id);
      motorB.write(can1.tx_data);
      can1.send(motorB.tx_id);
    }

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motorA.rx_id)
        motorA.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motorB.rx_id)
        motorB.read(can1.rx_data, stamp_ms);
    }
  }
}
