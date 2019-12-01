#pragma once

#include "third_party/FreeRTOS/Source/include/FreeRTOS.h"
#include "third_party/FreeRTOS/Source/include/task.h"
#include "third_party/FreeRTOS/Source/include/queue.h"

// Constants for system info
constexpr uint8_t NUM_FINGERS = 4;
constexpr float BRAKE_THRESHOLD = 50; // TODO: measure current when LA's get resistance, current value is a placeholder
//Task Handle for Uart task
TaskHandle_t xUartTaskHandle;
TaskHandle_t xPotAndBrakeHandle;
//Queue Handle for updating brakes
QueueHandle_t Q;

// Unionize float and uint32_t for serial communications
union vals
{
  float f;
  uint32_t ui;
};

// struct type to be passed to FreeRTOS tasks
struct paramsStruct
{
  vals rec[NUM_FINGERS];
  vals sen[NUM_FINGERS];
};
