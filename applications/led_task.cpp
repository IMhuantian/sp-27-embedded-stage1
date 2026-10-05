#include "cmsis_os.h"
#include "io/led/led.hpp"

// C板 RGB LED 挂在 TIM5 的三个通道上: PH10(TIM5_CH1) / PH11(TIM5_CH2) / PH12(TIM5_CH3)
sp::LED led(&htim5);

// 流水灯
// —— 只要这个任务还在被调度，灯就一直在变化；
//    如果程序在别处堵塞（某个高优先级任务死等 / DMA 卡死 / HardFault），灯就会停住不动，
//    所以在验收时可以拿"灯还动不动"直接判断程序是否堵塞。
extern "C" void led_task()
{
  led.start();

  while (true) {
    // 红 -> 绿 -> 蓝 依次渐亮渐灭，形成流水效果
    for (uint8_t i = 0; i < 100; i++) {
      led.set(i * 0.01f, 0, 0);
      osDelay(5);
    }
    for (int i = 100; i > 0; i--) {
      led.set(i * 0.01f, 0, 0);
      osDelay(5);
    }

    for (uint8_t i = 0; i < 100; i++) {
      led.set(0, i * 0.01f, 0);
      osDelay(5);
    }
    for (int i = 100; i > 0; i--) {
      led.set(0, i * 0.01f, 0);
      osDelay(5);
    }

    for (uint8_t i = 0; i < 100; i++) {
      led.set(0, 0, i * 0.01f);
      osDelay(5);
    }
    for (int i = 100; i > 0; i--) {
      led.set(0, 0, i * 0.01f);
      osDelay(5);
    }
  }
}
