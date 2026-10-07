#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// DT7 接收机接在 USART3 上(PC10=TX / PC11=RX)，100000bps / 偶校验 / 8 数据位 / 1 停止位，
// 这些都在 cboard.ioc 里配好了，接收走 DMA1_Stream1
sp::DBus remote(&huart3);

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    // remote 上的数据：ch_lh / ch_lv / ch_rh / ch_rv / ch_lu 是摇杆和拨轮，范围 [-1, 1]；
    // sw_l / sw_r 是左右三位开关；is_alive() 看接收机在不在线
    osDelay(10);
  }
}

// 串口空闲中断 + DMA 收满，把收到的长度交给 DBus 解析，然后马上重新挂上 DMA
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

// 收出错的时候重新挂 DMA
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}
