#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// C板蜂鸣器挂在 TIM4_CH3 (PD14) 上；TIM4 的定时器时钟为 84MHz
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 上电 / 烧录成功后的提示音
extern "C" void buzzer_task()
{
  buzzer.set(5000, 0.5f);  // 5kHz，占空比 0.5 时最响（想小声一点就改成 0.1）

  for (int i = 0; i < 3; i++) {
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }

  // 想放自定义音乐：把 {频率, 时长} 列成表，循环 set(hz, duty) -> start() -> osDelay() -> stop()
  // 例：Do 262Hz / Re 294Hz / Mi 330Hz ...

  while (true) {
    osDelay(100);
  }
}
