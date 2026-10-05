#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// C板 DT7 接收机接在 USART3 上(PC10=TX / PC11=RX)，
// 参数: 100000bps / 偶校验 / 8 数据位 / 1 停止位，已在 cboard.ioc 中配好，接收走 DMA1_Stream1
sp::DBus remote(&huart3);

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    // 可读字段(只读)：
    //   remote.ch_lh / ch_lv / ch_rh / ch_rv / ch_lu  摇杆与拨轮，范围 [-1, 1]
    //   remote.sw_l / remote.sw_r                     左/右三位开关: DOWN / MID / UP
    //   remote.mouse / remote.keys                    鼠标与键盘
    //   remote.is_alive(osKernelSysTick())            接收机是否在线
    osDelay(10);
  }
}

// 串口空闲中断 + DMA 收满：每次都把收到的字节数交给 DBus 解析，然后立刻重新挂上 DMA
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

// 接收出错(如噪声导致的帧错误)时重新挂 DMA，否则遥控器会一直收不到数据
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}
