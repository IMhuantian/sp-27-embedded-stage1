#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

// bmi088 系 -> 机器人(云台)系 的旋转矩阵
// 这里按"C板横装、CAN 一侧朝前"给出；装配方向一变就要改这个矩阵，否则 yaw/pitch/roll 全是错的
// 确定方法见 sp_middleware/io/bmi088/readme.md
const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// C板 IMU(BMI088) 走 SPI1: SCK=PB3 / MISO=PB4 / MOSI=PA7，片选 加速度计=PA4, 陀螺仪=PB0
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);

// Mahony 的 dt 必须与本任务的实际调用周期严格一致：osDelay(1) + 1000Hz tick => 1ms
sp::Mahony imu(1e-3f);

// IMU 采集 + 姿态解算
// bmi088 / imu 这两个对象在 applications/plotter_task.cpp 里被引用来做串口打印
extern "C" void imu_task()
{
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);

    // 想看数据：用调试器 live watch 观察本文件里的 bmi088 / imu，
    // 或者在 plotter_task 里把它们画到 SerialPlot 上
    osDelay(1);
  }
}
