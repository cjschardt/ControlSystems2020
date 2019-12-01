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
#include "third_party/FreeRTOS/Source/include/queue.h"

paramsStruct *shared_mem = new paramsStruct;

//Task to send and recive data over UART
void vUartTask(void *pvParameters)
{
  //paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  uint8_t receive = 0;
  //uint32_t prev_vals[NUM_FINGERS];
  //bool update = false;
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
      //LOG_INFO("Sent value %f over UART %i", shared_mem->sen[i].f, i);
      for(size_t j = 0; j < 4; j++)
      {
        receive = uart2.Read();
        shared_mem->rec[i].ui = (shared_mem->rec[i].ui << 8) | receive;
      }
      //if(i == 3)
      //{
      //  printf("Read %f for finger %d\n", shared_mem->rec[i].f, i);
      //}  
    }
    vTaskDelay(200);
  }
}
// void vUartTask(void *pvParameters)
// {
//   paramsStruct *shared_mem = (paramsStruct *) pvParameters;
//   uint8_t receive = 0;
//   sjsu::lpc40xx::Uart uart2(sjsu::lpc40xx::Uart::Port::kUart2);
//   uart2.Initialize(38400);
//   LOG_INFO("uart initialized");
//   int finger = 0;
//   int fing_arr[NUM_FINGERS] = {2,3,4,5};
//   while(1)
//   {
//     // Send a float (Glove data) over UART
//     for(int i = 0; i < NUM_FINGERS; i++)
//     {
//       uart2.Write((uint8_t) 0xF);
//       uart2.Write((uint8_t) 0xF);
//       uart2.Write((uint8_t) 0xF);
//       uart2.Write((uint8_t) 0xF);
//       uart2.Write((uint8_t) i*100);
//       for(size_t j = 24; j > 0; j -= 8)
//       {
//         uint8_t sendval = shared_mem->sen[i].ui >> j; 
//         uart2.Write(sendval);
//       }
//       uart2.Write((uint8_t) shared_mem->sen[i].ui);
//       printf("ADC %d sent value %f over UART %i\n", fing_arr[i], shared_mem->sen[i].f, i);
//     }
//     for(int i = 0; i < NUM_FINGERS; i++)
//     {
//       bool leave = false;
//       int leave_count = 0;
//       while(!leave)
//       {
//         if(uart2.Read() == 0xF)
//         {
//           leave_count++;
//         }
//         else
//         {
//           leave_count = 0;
//         }
//         if(leave_count == 3)
//         {
//           finger = uart2.Read()/100;
//           if(finger == 0 || finger == 1 || finger == 2 || finger == 3)
//           {
//             leave = true;
//           }
//           else 
//           {
//             leave_count = 0;
//           }
//         }
//       }
//       for(size_t j = 0; j < 4; j++)
//       {
//         receive = 0;
//         receive = uart2.Read();
//         shared_mem->rec[finger].ui = (shared_mem->rec[finger].ui << 8) | receive;
//       }
//     }
//     vTaskDelay(100);
//   }
// }

float my_round(float var)
{
  float value = (int)(var * 1000);
  return (value / 1000);
}

void vPotAndBrakeTask(void * pvParameters)
{
  //paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  sjsu::lpc40xx::Gpio brake0(2, 0);
  sjsu::lpc40xx::Gpio brake1(2, 1);
  sjsu::lpc40xx::Gpio brake2(2, 2);
  sjsu::lpc40xx::Gpio brake3(2, 4);
  sjsu::lpc40xx::Adc adc2(sjsu::lpc40xx::Adc::Channel::kChannel2);
  sjsu::lpc40xx::Adc adc3(sjsu::lpc40xx::Adc::Channel::kChannel3);
  sjsu::lpc40xx::Adc adc4(sjsu::lpc40xx::Adc::Channel::kChannel4);
  sjsu::lpc40xx::Adc adc5(sjsu::lpc40xx::Adc::Channel::kChannel5);
  sjsu::lpc40xx::Gpio brake_arr[NUM_FINGERS] = {brake0, brake1, brake2, brake3};
  //sjsu::lpc40xx::Gpio brake_arr[NUM_FINGERS] = {brake0, brake1, brake2};
  sjsu::lpc40xx::Adc adc_arr[NUM_FINGERS] = {adc2, adc3, adc4, adc5};
  //sjsu::lpc40xx::Adc adc_arr[NUM_FINGERS] = {adc2, adc3, adc4};
  float glove_position = 0;
  for(int i = 0; i < NUM_FINGERS; i++)
  {
    brake_arr[i].SetAsOutput();
    brake_arr[i].SetLow();
    LOG_INFO("configured brake%d pin as output", i);
    adc_arr[i].Initialize();
  }
  LOG_INFO("adc channels initialized");

  while(1)
  {
    //Check queue and update brakes if needed
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      //printf("\ncurrent is %f for finger %d\n", shared_mem->rec[i].f, i);
      if(shared_mem->rec[i].f >= BRAKE_THRESHOLD)
      {
        brake_arr[i].SetHigh();
        //if(i == 3)
        //{
        //  printf("Braking finger %d, current value at %f\n", i, shared_mem->rec[i].f);
        //}
      }
      else
      {
        brake_arr[i].SetLow();
      }
    }
    //Poll POTS
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      glove_position = adc_arr[i].Read();
      //LOG_INFO("ROUNDED: %f", my_round(rounded));
      shared_mem->sen[i].f = sjsu::Map(glove_position, 0, 4095, 0.0f, 3.3f);
      //LOG_INFO("Read %f from finger %d", shared_mem->sen[i].f, i);
      shared_mem->sen[i].f = my_round(shared_mem->sen[i].f);
    }
    vTaskDelay(200);
  }
}
