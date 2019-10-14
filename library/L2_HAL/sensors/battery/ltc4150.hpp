#pragma once

#include <cstdint>

#include "L2_HAL/sensors/battery/coulomb_counter.hpp"

#include "L1_Peripheral/lpc40xx/gpio.hpp"

namespace sjsu
{
class Ltc4150 : public CoulombCounter
{
 public:
  struct Battery_t
  {
    float battery_percent;
  };

  template <int id>
  class TickHandler
  {
   public:
    inline static Battery_t battery_info;

    static void IsrHandler()
    {
      battery_info.battery_percent -= static_cast<float>(0.1);
    }

    static float GetBatteryPercentage()
    {
      return battery_info.battery_percent;
    }
  };

  template <int id>
  explicit constexpr Ltc4150(const TickHandler<id> & isr,
                             lpc40xx::Gpio int_pin,
                             lpc40xx::Gpio pol)
      : int_pin_(int_pin), pol_(pol)
  {
    isr.battery_info.battery_percent = 100;

    get_percentage_ = isr.GetBatteryPercentage;
    int_pin_.EnableInterrupts();
    int_pin_.AttachInterrupt(&isr.IsrHandler, Gpio::Edge::kEdgeFalling);
  }

  float GetBatteryPercentage() const override
  {
    return get_percentage_();
  }

 private:
  lpc40xx::Gpio int_pin_;
  lpc40xx::Gpio pol_;
  float (*get_percentage_)(void) = nullptr;
};
}  // namespace sjsu
