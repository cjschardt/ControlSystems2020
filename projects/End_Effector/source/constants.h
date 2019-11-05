#pragma once

#include "third_party/FreeRTOS/Source/include/FreeRTOS.h"
#include "third_party/FreeRTOS/Source/include/task.h"

// Constants for the Linear Actuator Motor Controllers and system info
constexpr units::frequency::hertz_t motor_controller_freq = 75_Hz;
constexpr std::chrono::microseconds motor_controller_min_pulse = 1000us;
constexpr std::chrono::microseconds motor_controller_max_pulse = 2000us;
constexpr uint8_t NUM_FINGERS = 2;
constexpr float CURRENT_MAX = 30; // TODO: replace value with limit of final current sensor used
//Task Handle for Uart task
TaskHandle_t xUartTaskHandle;
//Task Handle for LinearActuator task
TaskHandle_t xLinearActuatorHandle;
//Task Handle for CurrentSensor task
TaskHandle_t xCurrentSensorHandle;

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

