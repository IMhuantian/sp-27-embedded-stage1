#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"

// 定义在 applications/imu_task.cpp
extern sp::Mahony imu;

// 波形走 USART1（丝印 UART2）921600，上位机用 SerialPlot。
// 一帧 = AA BB + size + 3 个 float，size = 通道数 x 4 = 0x0C，共 15 字节。
// 通道 1 roll、2 pitch、3 yaw，单位 rad。
// huart1 上只能挂一个 Plotter，两个任务同时发帧会交错成乱码
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(imu.roll, imu.pitch, imu.yaw);
    osDelay(1);
  }
}
