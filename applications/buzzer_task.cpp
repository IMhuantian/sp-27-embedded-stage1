#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// C板蜂鸣器: PD14(TIM4_CH3)，TIM4 的定时器时钟是 84MHz
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" void buzzer_task()
{
  buzzer.set(5000, 0.5f);  // 5kHz，占空比 0.5 时最响

  // 上电响三声
  for (int i = 0; i < 3; i++) {
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }

  // 要放别的曲子的话，把一串 {频率, 时长} 排成表循环调用就行

  while (true) {
    osDelay(100);
  }
}
