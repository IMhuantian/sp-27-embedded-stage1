#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

// 这两个对象定义在 applications/imu_task.cpp 里
extern sp::BMI088 bmi088;
extern sp::Mahony imu;

extern sp::RM_Motor motor6020;

// C板串口打印走 USART1 (PA9=TX / PB7=RX)，921600，帧格式见 io/plotter/readme.md
// 上位机用 SerialPlot 打开对应串口即可看到波形
// 注意：huart1 上只能有一个 Plotter 在发，否则两帧会交错成乱码
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(
      bmi088.acc[0], bmi088.acc[1], bmi088.acc[2],     // 加速度三轴 m/s^2
      bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2],  // 角速度三轴 rad/s
      imu.yaw, imu.pitch, imu.roll);                   // 姿态角 rad
    osDelay(1);
  }
}
