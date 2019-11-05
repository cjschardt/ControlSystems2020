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
void xUartTask(void* pvParameters)
{
	paramsStruct *shared_mem = (paramsStruct *) pvParameters;
	uint8_t receive = 0;
	sjsu::lpc40xx::Uart uart2(sjsu::lpc40xx::Uart::Port::kUart2);

  	uart2.Initialize(38400);
  	LOG_INFO("uart initialized");

    while(1)
    {
      // LA_position = adc2.Read();
      // Receive a float (Glove data) over UART
      for(size_t j = 0 ; j<2; j++)
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
void xLinearActuator(void* pvParameters)
{

	paramsStruct *shared_mem = (paramsStruct *) pvParameters;
	// Pini initialization for Linear Actuators
	sjsu::lpc40xx::Pwm p2_0(sjsu::lpc40xx::Pwm::Channel::kPwm0);
	sjsu::lpc40xx::Pwm p2_1(sjsu::lpc40xx::Pwm::Channel::kPwm1);
	uint32_t LA_position = 0;
	// Object declaration for Linear actuators
	sjsu::Servo linear_actuator1(p2_0);
	sjsu::Servo linear_actuator2(p2_1);
	// Set up Linear actuators with proper boundaries and initial conditions
  	linear_actuator1.Initialize();
  	linear_actuator1.SetFrequency(motor_controller_freq);
  	linear_actuator1.SetPulseBounds(motor_controller_min_pulse, 
                                 motor_controller_max_pulse);
  	linear_actuator2.Initialize();
  	linear_actuator2.SetFrequency(motor_controller_freq);
  	linear_actuator2.SetPulseBounds(motor_controller_min_pulse, 
                                 motor_controller_max_pulse);
    
    while (1)
    {
      // Run the recieved data through the PID algorithm
      // Map the output from the PID controller to proper units for the LA 
      int converted_output[2] = {(sjsu::Map(shared_mem->rec[0].f, 0.0f, 3.3f, 1000.0f, 2000.0f)) , (sjsu::Map(shared_mem->rec[1].f, 0.0f, 3.3f, 1000.0f, 2000.0f))};
    
    //  LOG_INFO("%f",converted_output);
    
      linear_actuator1.SetPulseWidthInMicroseconds(static_cast<std::chrono::microseconds>(converted_output[0]));

      linear_actuator2.SetPulseWidthInMicroseconds(static_cast<std::chrono::microseconds>(converted_output[1]));
      // Delay 100 ms
      vTaskDelay(100);
    }
}