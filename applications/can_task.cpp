/*
 * 姿态与电机联动 + CAN 收发
 *
 * 硬件接法：C板 + 两台 GM6020，都在 CAN1 上，电机 ID 分别拨到 1(A) 和 2(B)。
 * 上电前先把三个指向标箭头对到一起，程序会拿上电瞬间的电机读数当机械零位。
 *
 * 三个旋转输入端是同轴的，统一用"绕 yaw 轴逆时针为正"的机械角来描述：
 *     C板转角   phi_C = YAW_DIR * yaw - psi0
 *     A电机转角 phi_A = A_DIR * (motorA.angle - a0)
 *     B电机转角 phi_B = B_DIR * (motorB.angle - b_ref0)
 *
 * 右拨杆中档时按下面的关系联动：
 *     phi_A = phi_C
 *     phi_B = k * phi_C        k 由左拨杆定：下档 +0.5、中档 -1、上档 +3
 *
 * 手动拖动电机时重算 psi0 或 b_ref0，拖完之后转 C 板从新位置继续跟，不会弹回原零位。
 * 右拨杆下档失能、上档复位。
 *
 * 两台 GM6020 的 ID 都小于 5，控制帧 ID 都是 0x1FE，两条指令要写进同一帧只 send 一次。
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
constexpr float POS_KI = 0.0f;    // 用纯 PD，带积分的话手动拖动会松手回弹
constexpr float POS_KD = 0.03f;
constexpr float POS_MAX_OUT = 1.0f;
constexpr float POS_MAX_IOUT = 0.3f;
constexpr float POS_ALPHA = 0.8f;  // D 项滤波系数，1 是不滤波

// 拖动检测
constexpr float DRAG_TOL = 0.08f;        // rad，判为手拖的位置偏差阈值
constexpr float DRAG_DECAY = 0.002f;     // rad，偏差每周期至少收敛这么多才算电机在追指令
constexpr uint32_t DRAG_CONFIRM_MS = 20; // 偏差持续不收敛多久才判为手拖
constexpr uint32_t DRAG_HOLD_MS = 500;   // 主导电机的保持时间
constexpr float CBOARD_STILL = 0.0015f;  // rad/ms，一个周期内 C 板转这么少就算它没动
constexpr uint32_t SW_DEBOUNCE_MS = 30;  // 拨杆换档消抖

// 目标斜率限幅，避免换档或刚进联动时电机猛冲
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
  bool reset_done = false; // 上档复位到位了没有
  float psi0_mech = 0.0f;  // 联动参考零点（机械角）
  float a0 = 0.0f;         // A 电机上电时的读数，也就是箭头对齐位
  float b0 = 0.0f;         // B 电机同上
  float ref_a = 0.0f;      // A 的目标（motorA.angle 空间）
  float ref_b = 0.0f;
  float last_yaw = 0.0f;
  float dev_a = 0.0f;      // A 上一周期的位置偏差
  float dev_b = 0.0f;      // B 上一周期的位置偏差
  uint32_t dev_ms = 0;     // 偏差开始不收敛的时刻
  uint8_t leader = 0;      // 判定为被拖动的电机：0 无 / 1 A / 2 B
  uint32_t leader_ms = 0;  // 主导电机的认定时刻
  sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::DOWN;
  sp::DBusSwitchMode pending_sw_r = sp::DBusSwitchMode::DOWN;  // 消抖候选档位
  uint32_t sw_ms = 0;      // 候选档位出现的时刻
  float b_ref0 = 0.0f;     // B 的比例基准：ref_b = b_ref0 + B_DIR*(k*phi_c)
  sp::DBusSwitchMode last_sw_l = sp::DBusSwitchMode::MID;
  sp::DBusSwitchMode pending_sw_l = sp::DBusSwitchMode::MID;
  uint32_t sw_l_ms = 0;    // 左拨杆候选档位出现的时刻
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
float phi_B() { return B_DIR * (motorB.angle - g.b_ref0); }

void linkage_step()
{
  const float mech_yaw = YAW_DIR * imu.yaw;
  const float dyaw = sp::limit_angle(imu.yaw - g.last_yaw);
  g.last_yaw = imu.yaw;

  const uint32_t now = osKernelSysTick();

  // 电机还没上来反馈数据，连零位都还没法定，一律失能
  if (!motorA.is_open() || !motorB.is_open()) {
    motorA.cmd(0.0f);
    motorB.cmd(0.0f);
    pid_a.clear();
    pid_b.clear();
    g.inited = false;
    g.armed = false;
    g.reset_done = false;
    linkage_ref_a = g.ref_a;
    linkage_ref_b = g.ref_b;
    linkage_phi_c = 0.0f;
    linkage_k = 0.0f;
    return;
  }

  // 上电标定：把当前位置当成三个箭头对齐的机械零位。只看电机，不等遥控器
  if (!g.inited) {
    g.inited = true;
    g.a0 = motorA.angle;
    g.b0 = motorB.angle;
    g.b_ref0 = motorB.angle;
    g.psi0_mech = mech_yaw;
    g.ref_a = motorA.angle;
    g.ref_b = motorB.angle;
    g.armed = false;
    g.reset_done = false;
    pid_a.clear();
    pid_b.clear();
  }

  // 遥控器失联就一律失能，零位留着不重标
  if (!remote.is_alive(now)) {
    motorA.cmd(0.0f);
    motorB.cmd(0.0f);
    pid_a.clear();
    pid_b.clear();
    g.armed = false;
    g.reset_done = false;
    linkage_ref_a = g.ref_a;
    linkage_ref_b = g.ref_b;
    linkage_phi_c = 0.0f;
    linkage_k = 0.0f;
    return;
  }

  // 右拨杆换档消抖，新档位稳定 SW_DEBOUNCE_MS 后才切换；下面的分支都用消抖后的档位
  if (remote.sw_r != g.pending_sw_r) {
    g.pending_sw_r = remote.sw_r;
    g.sw_ms = now;
  } else if (g.pending_sw_r != g.last_sw_r && (now - g.sw_ms) > SW_DEBOUNCE_MS) {
    g.last_sw_r = g.pending_sw_r;
    g.armed = false;
    g.reset_done = false;
    g.dev_ms = now;
    g.dev_a = 0.0f;
    g.dev_b = 0.0f;
    g.leader = 0;
    pid_a.clear();
    pid_b.clear();
  }

  const bool cboard_still = (fabsf(dyaw) < CBOARD_STILL);
  // 比例用消抖后的档位
  float k = left_ratio(g.last_sw_l);

  // 下档：失能，发 0 电流
  if (g.last_sw_r == sp::DBusSwitchMode::DOWN) {
    motorA.cmd(0.0f);
    motorB.cmd(0.0f);
    linkage_ref_a = g.ref_a;
    linkage_ref_b = g.ref_b;
    linkage_phi_c = 0.0f;
    return;
  }

  // 上档：复位。两台电机按 1:1 转到和 C 板当前指向一致的位置，三个箭头重新指同一个方向
  if (g.last_sw_r == sp::DBusSwitchMode::UP) {
    const float reset_phi_c = mech_yaw - g.psi0_mech;
    const float target_a = g.a0 + A_DIR * reset_phi_c;
    const float target_b = g.b0 + B_DIR * reset_phi_c;

    g.ref_a = slew(g.ref_a, target_a, REF_RATE * PID_DT);
    g.ref_b = slew(g.ref_b, target_b, REF_RATE * PID_DT);

    pid_a.calc(g.ref_a, motorA.angle);
    pid_b.calc(g.ref_b, motorB.angle);
    motorA.cmd(pid_a.out);
    motorB.cmd(pid_b.out);

    // 只锁一次：到位后把当前位置和朝向定成新的零位
    if (!g.reset_done && fabsf(motorA.angle - target_a) < HOME_TOL &&
        fabsf(motorB.angle - target_b) < HOME_TOL) {
      g.reset_done = true;
      g.a0 = motorA.angle;
      g.b0 = motorB.angle;
      g.b_ref0 = motorB.angle;
      g.psi0_mech = mech_yaw;
      g.ref_a = motorA.angle;
      g.ref_b = motorB.angle;
    }

    linkage_ref_a = g.ref_a;
    linkage_ref_b = g.ref_b;
    linkage_phi_c = 0.0f;
    return;
  }

  // 中档：姿态与电机联动
  if (!g.armed) {
    // 刚切进来，以 A 的当前位置为起点
    g.armed = true;
    g.psi0_mech = mech_yaw - phi_A();
    g.ref_a = motorA.angle;
    g.ref_b = motorB.angle;
    g.dev_ms = now;
    g.dev_a = 0.0f;
    g.dev_b = 0.0f;
    g.leader = 0;
    // 进中档时对齐左拨杆，并把 B 的基准钉在当前角度上
    g.last_sw_l = remote.sw_l;
    g.pending_sw_l = remote.sw_l;
    g.sw_l_ms = now;
    g.b_ref0 = motorB.angle - B_DIR * (left_ratio(g.last_sw_l) * (mech_yaw - g.psi0_mech));
  }

  // 左拨杆换档：k 变了要重钉 B 的基准，否则目标会随 k*phi_c 跳一次
  if (remote.sw_l != g.pending_sw_l) {
    g.pending_sw_l = remote.sw_l;
    g.sw_l_ms = now;
  } else if (g.pending_sw_l != g.last_sw_l && (now - g.sw_l_ms) > SW_DEBOUNCE_MS) {
    g.last_sw_l = g.pending_sw_l;
    k = left_ratio(g.last_sw_l);
    const float phi_c_now = mech_yaw - g.psi0_mech;
    g.b_ref0 = motorB.angle - B_DIR * (k * phi_c_now);
    g.ref_b = motorB.angle;
  }
  linkage_k = k;

  // 拖动检测：只有 C 板基本没动的时候，电机的偏差才认为是手拖出来的。
  // C 板自己在转的时候电机也会滞后，那种偏差不能当成拖动。
  // 另外偏差要持续不收敛才算手拖，正常追指令时偏差很快收敛。
  if (cboard_still) {
    float phi_c = mech_yaw - g.psi0_mech;
    const float dev_a = fabsf(g.a0 + A_DIR * phi_c - motorA.angle);
    const float dev_b = fabsf(g.b_ref0 + B_DIR * k * phi_c - motorB.angle);

    // 偏差在收敛，重新计时
    if (dev_a < g.dev_a - DRAG_DECAY || dev_b < g.dev_b - DRAG_DECAY) {
      g.dev_ms = now;
    }
    g.dev_a = dev_a;
    g.dev_b = dev_b;

    if (now - g.dev_ms > DRAG_CONFIRM_MS) {
      // 一次只认一台被拖动的电机，认住之后 DRAG_HOLD_MS 内不换人
      if (g.leader != 0) {
        const float dev_leader = (g.leader == 1) ? dev_a : dev_b;
        if ((now - g.leader_ms) > DRAG_HOLD_MS && dev_leader < DRAG_TOL) {
          g.leader = 0;
        }
      } else if (dev_a > DRAG_TOL) {
        g.leader = 1;
        g.leader_ms = now;
      } else if (dev_b > DRAG_TOL) {
        g.leader = 2;
        g.leader_ms = now;
      }

      bool fired = false;
      if (g.leader == 1 && dev_a > DRAG_TOL) {
        // A 被拖动：参考零点跟着 A 走
        g.psi0_mech = mech_yaw - phi_A();
        g.ref_a = motorA.angle;
        phi_c = mech_yaw - g.psi0_mech;
        g.leader_ms = now;
        fired = true;
      } else if (g.leader == 2 && dev_b > DRAG_TOL) {
        // B 被拖动：反过来由 B 推零点
        g.psi0_mech = mech_yaw - phi_B() / k;
        g.ref_b = motorB.angle;
        g.leader_ms = now;
        fired = true;
      }

      if (fired) {
        g.dev_ms = now;
        g.dev_a = 0.0f;
        g.dev_b = 0.0f;
      }
    }
  }

  const float phi_c = mech_yaw - g.psi0_mech;

  g.ref_a = slew(g.ref_a, g.a0 + A_DIR * phi_c, REF_RATE * PID_DT);
  g.ref_b = slew(g.ref_b, g.b_ref0 + B_DIR * (k * phi_c), REF_RATE * PID_DT);

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
