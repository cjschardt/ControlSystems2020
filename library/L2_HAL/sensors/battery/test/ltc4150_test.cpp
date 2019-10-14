#include "L2_HAL/sensors/battery/ltc4150.hpp"
#include "L4_Testing/testing_frameworks.hpp"

namespace sjsu
{
EMIT_ALL_METHODS(Ltc4150);
TEST_CASE("Test LTC4150")
{
  lpc40xx::Gpio primary_int_pin(2, 1);
  lpc40xx::Gpio primary_pol_pin(2, 0);
  Ltc4150::TickHandler<0> primary_isr;
  Ltc4150 primary_battery(primary_isr, primary_int_pin, primary_pol_pin);

  lpc40xx::Gpio backup_int_pin(2, 3);
  lpc40xx::Gpio backup_pol_pin(2, 2);
  Ltc4150::TickHandler<0> backup_isr;
  Ltc4150 backup_battery(backup_isr, backup_int_pin, backup_pol_pin);

  CHECK(primary_battery.GetBatteryPercentage() == 100.0);
  CHECK(backup_battery.GetBatteryPercentage() == 100.0);

  primary_isr.IsrHandler();
  primary_isr.IsrHandler();
  primary_isr.IsrHandler();
  primary_isr.IsrHandler();
  CHECK(primary_battery.GetBatteryPercentage() == 99.6);

  backup_isr.IsrHandler();
  backup_isr.IsrHandler();
  backup_isr.IsrHandler();
  backup_isr.IsrHandler();
  backup_isr.IsrHandler();
  backup_isr.IsrHandler();
  CHECK(backup_battery.GetBatteryPercentage() == 99.4);
}
}  // namespace sjsu
