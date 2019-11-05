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
      for(size_t j = 0 ; j < NUM_FINGERS; j++)
      {
        for (size_t i = 0; i < 4; i++)
        {
          receive = uart2.Read();
          shared_mem->rec[j].ui = (shared_mem->rec[j].ui << 8) | receive;
        }
      }
      // Delay 100 ms
      vTaskDelay(100);
    }
}

//Task to control linear actuators
void vLinearActuator(void* pvParameters)
{
	paramsStruct *shared_mem = (paramsStruct *) pvParameters;
	// Pini initialization for Linear Actuators
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
      	linear_actuator_arr[i].SetPulseWidthInMicroseconds(static_cast<std::chrono::microseconds>(converted_output[i]));
      }
      // Delay 100 ms
      vTaskDelay(100);
    }
}