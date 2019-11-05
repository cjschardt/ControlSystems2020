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

void vUartTask(void *pvParameters)
{
  paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  uint8_t receive = 0;
  sjsu::lpc40xx::Uart uart2(sjsu::lpc40xx::Uart::Port::kUart2);
  uart2.Initialize(38400);
  LOG_INFO("uart initialized");
  while(1)
  {
    // Send a float (Glove data) over UART
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      for(size_t j = 24; j > 0; j -= 8)
      {
        uint8_t sendval = shared_mem->sen[i].ui >> j; 
        uart2.Write(sendval);
      }
      uart2.Write((uint8_t) shared_mem->sen[i].ui);
      LOG_INFO("Sent value %f over UART", shared_mem->sen[i].f);
      for(size_t j = 0; j < 4; j++)
      {
        receive = uart2.Read();
        shared_mem->rec[i].ui = (shared_mem->rec[i].ui << 8) | receive;
      }
      LOG_INFO("Recieved value %f over UART", shared_mem->rec[i].f);
    }
    vTaskDelay(100);
  }
}

void vPotentiometerTask(void *pvParameters)
{
  paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  sjsu::lpc40xx::Adc adc2(sjsu::lpc40xx::Adc::Channel::kChannel2);
  sjsu::lpc40xx::Adc adc4(sjsu::lpc40xx::Adc::Channel::kChannel4);
  sjsu::lpc40xx::Adc adc_arr[NUM_FINGERS] = {adc2, adc4};
  for(int i = 0; i < NUM_FINGERS; i++)
  {
    adc_arr[i].Initialize();
  }
  LOG_INFO("adc channels initialized");
  uint32_t glove_position = 0;
  while(1)
  {
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      glove_position = adc_arr[i].Read();
      shared_mem->sen[i].f = sjsu::Map(glove_position, 0, 4095, 0.0f, 3.3f);
    }
    vTaskDelay(100);
  }
}

void vBrakeTask(void * pvParameters)
{
  paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  sjsu::lpc40xx::Gpio brake0(1, 19);
  sjsu::lpc40xx::Gpio brake1(2, 3);
  sjsu::lpc40xx::Gpio brake_arr[2] = {brake0, brake1};
  for(int i = 0; i < NUM_FINGERS; i++)
  {
    brake_arr[i].SetAsOutput();
    brake_arr[i].SetLow();
    LOG_INFO("configured brake%d pin as output", i);
  }
  while(1)
  {
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      if(shared_mem->rec[i].f >= BRAKE_THRESHOLD)
      {
        brake_arr[i].SetHigh();
      }
      else
      {
        brake_arr[i].SetLow();
      }
    }
    vTaskDelay(100);
  }
}