#include <cstdio>
#include <stdint.h>
#include <iterator>
#include "PID.hpp"
#include "L1_Peripheral/lpc40xx/uart.hpp"
#include "L1_Peripheral/lpc40xx/adc.hpp"
#include "L1_Peripheral/lpc40xx/gpio.hpp"
#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "utility/log.hpp"
#include "utility/map.hpp"
#include "utility/time.hpp"

int main()
{
  union vals
  {
    float f;
    uint32_t ui;
  }sen;

  // Declare peripheral objects
  sjsu::lpc40xx::Uart uart2(sjsu::lpc40xx::Uart::Port::kUart2);
  sjsu::lpc40xx::Adc adc4(sjsu::lpc40xx::Adc::Channel::kChannel4);

  adc4.Initialize();
  //adc4.BurstMode(true);
  LOG_INFO("adc4 initialized");
  uart2.Initialize(38400);
  LOG_INFO("uart initialized");
  uint32_t glove_position = 0;

  while (true)
  {
    glove_position = adc4.Read();
    // Map the data from the glove to proper voltages
    sen.f = sjsu::Map(glove_position, 0, 4095, 0.0f, 3.3f);
    // Send a float (Glove data) over UART
    for(size_t i = 24; i > 0; i -= 8)
    {
      uint8_t sendval = sen.ui >> i; 
      uart2.Write(sendval);
    }
    uart2.Write((uint8_t) sen.ui);
    LOG_INFO("Sent value %f over UART", sen.f);
    // Delay 100 ms
    sjsu::Delay(100ms);
  }
  return 0;
}
