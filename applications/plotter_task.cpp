#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

// 定义在 applications/imu_task.cpp
extern sp::BMI088 bmi088;
extern sp::Mahony imu;

// 定义在 applications/can_task.cpp
extern sp::RM_Motor motorA;
extern sp::RM_Motor motorB;
extern float linkage_ref_a;
extern float linkage_ref_b;
extern float linkage_phi_c;

// 串口打印走 USART1 (PA9=TX / PB7=RX) 921600，上位机用 SerialPlot 看波形。
// 注意 huart1 上只能挂一个 Plotter，两个任务同时发帧会交错成乱码
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(
      bmi088.acc[0], bmi088.acc[1], bmi088.acc[2],    // 加速度三轴 m/s^2
      bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2],  // 角速度三轴 rad/s
      imu.yaw,                                         // C板偏航角 rad
      motorA.angle, linkage_ref_a,                     // A 电机：反馈 / 目标
      motorB.angle, linkage_ref_b,                     // B 电机：反馈 / 目标
      linkage_phi_c);                                  // C 板相对参考零点的转角
    osDelay(1);
  }
}
