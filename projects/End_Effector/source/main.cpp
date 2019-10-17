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
// Constants for the Linear Actuator Motor Controllers
constexpr units::frequency::hertz_t motor_controller_freq = 75_Hz;
constexpr std::chrono::microseconds motor_controller_min_pulse = 1000us;
constexpr std::chrono::microseconds motor_controller_max_pulse = 2000us;
int main()
{
  union vals
  {
    float f;
    uint32_t ui;
  }rec;
  // Declare peripheral objects
  sjsu::lpc40xx::Uart uart2(sjsu::lpc40xx::Uart::Port::kUart2);
  
  sjsu::lpc40xx::Pwm p2_0(sjsu::lpc40xx::Pwm::Channel::kPwm0);
  uart2.Initialize(38400);
  LOG_INFO("uart initialized");
  uint32_t LA_position = 0;
  float set_point = 0;
  sjsu::Servo linear_actuator(p2_0);
  linear_actuator.Initialize();
  linear_actuator.SetFrequency(motor_controller_freq);
  linear_actuator.SetPulseBounds(motor_controller_min_pulse, 
                                 motor_controller_max_pulse);
  LOG_INFO("Motor controller initialized");
  bool prev_sign = false;
  while (true)
  {
    // LA_position = adc2.Read();

    // Receive a float (Glove data) over UART
    for (size_t i = 0; i < 4; i++)
    {
      uint8_t receive = uart2.Read();
      rec.ui = (rec.ui << 8) | receive;
    }
    LOG_INFO("Read value %f over UART", rec.f);
    set_point = rec.f;
    // Run the recieved data through the PID algorithm
    // Map the output from the PID controller to proper units for the LA 
    int converted_output 
        = (sjsu::Map(rec.f, 0.0f, 3.3f, 1000.0f, 2000.0f));
        LOG_INFO("%f",converted_output);
      linear_actuator.SetPulseWidthInMicroseconds(static_cast<std::chrono::microseconds>(converted_output));
    // Delay 100 ms
    sjsu::Delay(100ms);
  }
  return 0;
}
