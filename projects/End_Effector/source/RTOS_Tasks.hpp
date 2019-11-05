#pragma once

#include <cstdio>
#include <stdint.h>
#include <iterator>
#include "L1_Peripheral/lpc40xx/uart.hpp"
#include "L1_Peripheral/lpc40xx/adc.hpp"
#include "L1_Peripheral/lpc40xx/gpio.hpp"
#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L2_HAL/actuators/servo/servo.hpp"
#include "utility/log.hpp"
#include "utility/map.hpp"
#include "utility/time.hpp"
#include "constants.h"

#include "third_party/FreeRTOS/Source/include/FreeRTOS.h"
#include "third_party/FreeRTOS/Source/include/task.h"

//Task to send and recive data over UART
void vUartTask(void* pvParameters)
{
  paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  uint8_t receive = 0;
  sjsu::lpc40xx::Uart uart2(sjsu::lpc40xx::Uart::Port::kUart2);
  uart2.Initialize(38400);
  LOG_INFO("uart initialized");
  while(1)
  {
    // Receive a float (Glove data) over UART
    for(size_t i = 0 ; i < NUM_FINGERS; i++)
    {
      for(size_t j = 0; j < 4; j++)
      {
        receive = uart2.Read();
        shared_mem->rec[i].ui = (shared_mem->rec[i].ui << 8) | receive;
      }
      LOG_INFO("Recieved value %f over UART", shared_mem->rec[i].f);
      for(size_t j = 24; j > 0; j -= 8)
      {
        uint8_t sendval = shared_mem->sen[i].ui >> j; 
        uart2.Write(sendval);
      }
      uart2.Write((uint8_t) shared_mem->sen[i].ui);
      LOG_INFO("Sent value %f over UART", shared_mem->sen[i].f);
    }
    // Delay 100 ms
    vTaskDelay(100);
  }
}

//Task to control linear actuators
void vLinearActuatorTask(void* pvParameters)
{
  paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  // Pin initialization for Linear Actuators
  sjsu::lpc40xx::Pwm p2_0(sjsu::lpc40xx::Pwm::Channel::kPwm0);
  sjsu::lpc40xx::Pwm p2_1(sjsu::lpc40xx::Pwm::Channel::kPwm1);
  // Object declaration for Linear actuators
  sjsu::Servo linear_actuator0(p2_0);
  sjsu::Servo linear_actuator1(p2_1);
  sjsu::Servo linear_actuator_arr[NUM_FINGERS] = {linear_actuator0, linear_actuator1};  
  // Set up Linear actuators with proper boundaries and initial conditions
  for(int i = 0; i < NUM_FINGERS; i++)
  {
      linear_actuator_arr[i].Initialize();
      linear_actuator_arr[i].SetFrequency(motor_controller_freq);
      linear_actuator_arr[i].SetPulseBounds(motor_controller_min_pulse, 
                                        motor_controller_max_pulse);
      LOG_INFO("linear_actuator%d initialized", i);
  }
   while (1)
  {
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      // Map the output from the PID controller to proper units for the LA 
      int converted_output = (sjsu::Map(shared_mem->rec[i].f, 0.0f, 3.3f, 1000.0f, 2000.0f));
      // Update the linear actuator position
      linear_actuator_arr[i].SetPulseWidthInMicroseconds(static_cast<std::chrono::microseconds>(converted_output));
    }
    // Delay 100 ms
    vTaskDelay(100);
  }
}

void vCurrentSensorTask(void* pvParameters)
{
  paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  float pot_position = 0;
  sjsu::lpc40xx::Adc adc2(sjsu::lpc40xx::Adc::Channel::kChannel2);
  sjsu::lpc40xx::Adc adc4(sjsu::lpc40xx::Adc::Channel::kChannel4);
  sjsu::lpc40xx::Adc adc_arr[NUM_FINGERS] = {adc2, adc4};
  for(int i = 0; i < NUM_FINGERS; i++)
  {
    adc_arr[i].Initialize();
  }
  LOG_INFO("adc channels initialized");
  while(1)
  {
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      pot_position = adc_arr[i].Read();
      shared_mem->sen[i].f = sjsu::Map(pot_position, 0, 4095, 0.0f, CURRENT_MAX);
    }
    vTaskDelay(100);
  }
}
