#pragma once

#include "third_party/FreeRTOS/Source/include/FreeRTOS.h"
#include "third_party/FreeRTOS/Source/include/task.h"


// Constants for the Linear Actuator Motor Controllers
constexpr units::frequency::hertz_t motor_controller_freq = 75_Hz;
constexpr std::chrono::microseconds motor_controller_min_pulse = 1000us;
constexpr std::chrono::microseconds motor_controller_max_pulse = 2000us;
//Task Handle for Uart task
TaskHandle_t xUartTaskHandle;
//Task Handle for LinearActuator task
TaskHandle_t xLinearActuatorHandle;
//Task Handle for CurrentSensor task
TaskHandle_t xCurrentSensorHandle;

union vals
{
  float f;
  uint32_t ui;
};

struct paramsStruct
{
  vals rec[2];
  vals sen[2];
};

