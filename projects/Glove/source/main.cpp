#include <cstdio>
#include <stdint.h>
#include <iterator>
#include "PID.hpp"
#include "L1_Peripheral/lpc40xx/uart.hpp"
#include "L1_Peripheral/lpc40xx/adc.hpp"
#include "L1_Peripheral/lpc40xx/gpio.hpp"
#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "utility/log.hpp"
#include "utility/map.hpp"
#include "utility/time.hpp"
#include "RTOS_Tasks.hpp"
#include "constants.h"

#include "third_party/FreeRTOS/Source/include/FreeRTOS.h"
#include "third_party/FreeRTOS/Source/include/task.h"
#include "third_party/FreeRTOS/Source/include/queue.h"

int main()
{
  Q = xQueueCreate(10, sizeof(vals[NUM_FINGERS]));
  paramsStruct pvParameters;
  xTaskCreate(vUartTask, "uart_task", 1024, (void *) &pvParameters, tskIDLE_PRIORITY + 2, &xUartTaskHandle);
  xTaskCreate(vPotAndBrakeTask, "Potentiometer and Brake task", 1024, (void *) &pvParameters, tskIDLE_PRIORITY + 1, &xPotAndBrakeHandle);
  vTaskStartScheduler();
}
