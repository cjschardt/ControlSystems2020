#pragma once

namespace sjsu
{
class CoulombCounter
{
 public:
  virtual float GetBatteryPercentage() const = 0;
};
}  // namespace sjsu
