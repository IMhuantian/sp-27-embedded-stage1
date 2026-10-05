#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::RM_Motor motor6020;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    plotter.plot(
      1.0f
      // motor6020.speed
    );
    osDelay(1);
  }
}