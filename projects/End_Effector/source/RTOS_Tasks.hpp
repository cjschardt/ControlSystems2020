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
  sjsu::lpc40xx::Uart uart2(sjsu::lpc40xx::Uart::Port::kUart2);
  uart2.Initialize(38400);
  LOG_INFO("uart initialized");
  int fing_arr[NUM_FINGERS] = {2,3,4,5};
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
      printf("adc %d Read %f for finger %d\n", fing_arr[i], shared_mem->rec[i].f, i);
    }
    vTaskDelay(200);
  } 
}

void vSensorAndActuatorTask(void *pvParameters)
{
  //paramsStruct *shared_mem = (paramsStruct *) pvParameters;
  float pot_position = 0;
  uint32_t prev_vals[NUM_FINGERS] = {0};
  // Pin initialization for ADC channels
  sjsu::lpc40xx::Adc adc2(sjsu::lpc40xx::Adc::Channel::kChannel2);
  sjsu::lpc40xx::Adc adc3(sjsu::lpc40xx::Adc::Channel::kChannel3);
  sjsu::lpc40xx::Adc adc4(sjsu::lpc40xx::Adc::Channel::kChannel4);
  sjsu::lpc40xx::Adc adc5(sjsu::lpc40xx::Adc::Channel::kChannel5);

  // Pin initialization for Linear Actuators
  sjsu::lpc40xx::Pwm p2_0(sjsu::lpc40xx::Pwm::Channel::kPwm0);
  sjsu::lpc40xx::Pwm p2_1(sjsu::lpc40xx::Pwm::Channel::kPwm1);
  sjsu::lpc40xx::Pwm p2_2(sjsu::lpc40xx::Pwm::Channel::kPwm2);
  sjsu::lpc40xx::Pwm p2_4(sjsu::lpc40xx::Pwm::Channel::kPwm4);
  sjsu::lpc40xx::Pwm p2_5(sjsu::lpc40xx::Pwm::Channel::kPwm5);
  // Object declaration for Linear actuators
  sjsu::Servo linear_actuator0(p2_0);
  sjsu::Servo linear_actuator1(p2_1);
  sjsu::Servo linear_actuator2(p2_2);
  sjsu::Servo linear_actuator3(p2_4);
  sjsu::Servo linear_actuator4(p2_5);
  // Arrays to store peripheral handles
  sjsu::lpc40xx::Adc adc_arr[NUM_FINGERS] = {adc2, adc3, adc4, adc5};
  sjsu::Servo linear_actuator_arr[NUM_FINGERS+1] = {linear_actuator0, linear_actuator1, linear_actuator2, linear_actuator3, linear_actuator4};
  // Set up Linear actuators with proper boundaries and initial conditions
  for(int i = 0; i < NUM_FINGERS; i++)
  {
      shared_mem->rec[i].ui = 0;
      linear_actuator_arr[i].Initialize();
      linear_actuator_arr[i].SetFrequency(motor_controller_freq);
      linear_actuator_arr[i].SetPulseBounds(motor_controller_min_pulse, 
                                            motor_controller_max_pulse);
      LOG_INFO("linear_actuator%d initialized", i);
      adc_arr[i].Initialize();
  }
  linear_actuator_arr[4].Initialize();
  linear_actuator_arr[4].SetFrequency(motor_controller_freq);
  linear_actuator_arr[4].SetPulseBounds(motor_controller_min_pulse, 
                                        motor_controller_max_pulse);
  LOG_INFO("adc channels initialized");
  float pdubs_arr[NUM_FINGERS] = {2.0,2.1,2.2,2.4};
  while(1)
  {
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      if(shared_mem->rec[i].f < 0)
      {
        shared_mem->rec[i].f = 0;
      }
      else if(shared_mem->rec[i].f > 3.3)
      {
        shared_mem->rec[i].f = 3.3;
      }
      // Map the output from the PID controller to proper units for the LA
      int converted_output = (sjsu::Map(shared_mem->rec[i].f, 0.0f, 3.3f, 1000, 2000));
      // Update the linear actuator position 
      linear_actuator_arr[i].SetPulseWidthInMicroseconds(static_cast<std::chrono::microseconds>(converted_output));
    }
    p2_5.SetDutyCycle(p2_4.GetDutyCycle());
    for(int i = 0; i < NUM_FINGERS; i++)
    {
      pot_position = adc_arr[i].Read();
      shared_mem->sen[i].f = sjsu::Map(pot_position, 0, 4095, 0.0f, CURRENT_MAX);
    }
    vTaskDelay(200);
  }
}
