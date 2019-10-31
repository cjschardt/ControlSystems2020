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

#include "third_party/FreeRTOS/Source/include/FreeRTOS.h"
#include "third_party/FreeRTOS/Source/include/task.h"

// Constants for the Linear Actuator Motor Controllers
constexpr units::frequency::hertz_t motor_controller_freq = 75_Hz;
constexpr std::chrono::microseconds motor_controller_min_pulse = 1000us;
constexpr std::chrono::microseconds motor_controller_max_pulse = 2000us;

void xUartTask(void* p);
void xLinearActuator(void* p);

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
}rec[2];

uint8_t receive = 0;
// Declare peripheral objects
sjsu::lpc40xx::Uart uart2(sjsu::lpc40xx::Uart::Port::kUart2);
  
sjsu::lpc40xx::Pwm p2_0(sjsu::lpc40xx::Pwm::Channel::kPwm0);
sjsu::lpc40xx::Pwm p2_1(sjsu::lpc40xx::Pwm::Channel::kPwm1);

uint32_t LA_position = 0;


sjsu::Servo linear_actuator1(p2_0);
sjsu::Servo linear_actuator2(p2_1);

int main()
{
  uart2.Initialize(38400);
  LOG_INFO("uart initialized");

  linear_actuator1.Initialize();
  linear_actuator1.SetFrequency(motor_controller_freq);
  linear_actuator1.SetPulseBounds(motor_controller_min_pulse, 
                                 motor_controller_max_pulse);

  linear_actuator2.Initialize();
  linear_actuator2.SetFrequency(motor_controller_freq);
  linear_actuator2.SetPulseBounds(motor_controller_min_pulse, 
                                 motor_controller_max_pulse);
  LOG_INFO("Motor controller initialized");

  bool prev_sign = false;

  xTaskCreate(xUartTask, "uart_task", 1024, NULL, tskIDLE_PRIORITY + 2, &xUartTaskHandle);
  xTaskCreate(xLinearActuator, "linear_actuator_task", 1024, NULL, tskIDLE_PRIORITY + 1, &xLinearActuatorHandle);

  vTaskStartScheduler();
}

//Task to send and recive data over UART
void xUartTask(void* p)
{
    while(1)
    {
      // LA_position = adc2.Read();
      // Receive a float (Glove data) over UART
      for(size_t j = 0 ; j<2; j++)
      {
        for (size_t i = 0; i < 4; i++)
        {
          receive = uart2.Read();
          rec[j].ui = (rec[j].ui << 8) | receive;
        }
        //LOG_INFO("Read value %f over UART", rec.f);
      }
      // Delay 100 ms
      vTaskDelay(500);
    }
}

//Task to control linear actuators
void xLinearActuator(void* p)
{
    while (1)
    {
      // Run the recieved data through the PID algorithm
      // Map the output from the PID controller to proper units for the LA 
      int converted_output[2] = {(sjsu::Map(rec[0].f, 0.0f, 3.3f, 1000.0f, 2000.0f)) , (sjsu::Map(rec[1].f, 0.0f, 3.3f, 1000.0f, 2000.0f))};
    
    //  LOG_INFO("%f",converted_output);
    
      linear_actuator1.SetPulseWidthInMicroseconds(static_cast<std::chrono::microseconds>(converted_output[0]));

      linear_actuator2.SetPulseWidthInMicroseconds(static_cast<std::chrono::microseconds>(converted_output[1]));
      // Delay 100 ms
      vTaskDelay(100);
    }
}

// //Task to read current sensors
// void xCurrentSensor(void* p)
// {

// }