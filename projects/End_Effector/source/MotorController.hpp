#pragma once
#include <stdint.h>
#include <math.h>
#include <cstdio>

#include "utility/log.hpp"
#include "utility/units.hpp"
#include "utility/map.hpp"
#include "L1_Peripheral/lpc40xx/gpio.hpp"
#include "L1_Peripheral/pwm.hpp"

namespace sjsu
{
class MotorController
{
public:
  // We will utilize the revolutions per minute feature of the units library.
  // This will make setting the speed of motors more natural.
  static constexpr units::frequency::hertz_t kDefaultFrequency  = 50_Hz;
  static constexpr units::angular_velocity::\
      revolutions_per_minute_t kDefaultMinRotation              = 0_rpm;
  static constexpr units::angular_velocity::\
      revolutions_per_minute_t kDefaultMaxRotation              = 10_rpm;     
  static constexpr std::chrono::microseconds kDefaultMinPulse   = 0us;
  static constexpr std::chrono::microseconds kDefaultMaxPulse   = 2000us;

  // References to the PWM pin and the direction gpio are passed into the 
  // constructor so that the class has direct access to them.
  explicit constexpr MotorController(sjsu::Pwm & pwm, sjsu::Gpio & gpio)
  : mc_pwm_(pwm),
    direction_pin_(gpio),
    waveform_period_(0),
    pulse_lower_bound_(kDefaultMinPulse),
    pulse_upper_bound_(kDefaultMaxPulse),
    min_rotation_(kDefaultMinRotation),
    max_rotation_(kDefaultMaxRotation)
  {
  }

  // This initializes the PWM with the frequency passed in. If no arguments are 
  // given, it will default to 50 Hz
  virtual void Initialize(
      units::frequency::hertz_t frequency = kDefaultFrequency)
  {
    printf("frequency: %f\n", kDefaultFrequency);
    mc_pwm_.Initialize(frequency);
    SetFrequency(frequency);
  }

  // This will set the frequency to the value specified. If no arguments are 
  // given, it will default to 50 Hz
  virtual void SetFrequency(
      units::frequency::hertz_t frequency = kDefaultFrequency)
  {
    printf("%f\n", frequency);
    mc_pwm_.SetFrequency(frequency);
    waveform_period_ =
        std::chrono::microseconds((1_MHz / frequency).to<uint32_t>());
  }

  // This sets the bounds of the pulse width of the PWM signal in the case that
  // there is an operational range of duty cycles. It will default to 0 us and 
  // 2000 us, respectively, in the case that no inputs are specified.
  virtual void SetPulseBounds(std::chrono::microseconds lower=kDefaultMinPulse,
                              std::chrono::microseconds upper=kDefaultMaxPulse)
  {
    pulse_lower_bound_ = lower;
    pulse_upper_bound_ = upper;
  }

  // This sets the minimum and maximum speeds at which the motor will operate. 
  // It will default to 0 rpm and 10 rpm, respectively, if no arguments are 
  // passed.
  virtual void SetAngularVelocityBounds(
      units::angular_velocity::revolutions_per_minute_t min=kDefaultMinRotation,
      units::angular_velocity::revolutions_per_minute_t max=kDefaultMaxRotation)
  {
    min_rotation_ = min;
    max_rotation_ = max;
  }

  virtual void SetPulseWidthInMicroseconds(
      std::chrono::microseconds pulse_width)
  {
    mc_pwm_.SetDutyCycle(static_cast<float>(pulse_width.count()) /
                            static_cast<float>(waveform_period_.count()));
  }

  virtual void SetDirection(bool direction)
  {
    direction_pin_.Set(static_cast<Gpio::State>(direction));
  }

  virtual void SetAngularVelocity(
      units::angular_velocity::revolutions_per_minute_t vel)
  {

    float pulse_width = Map(vel.to<float>(),
                            min_rotation_.to<float>(),
                            max_rotation_.to<float>(),
                            static_cast<float>(pulse_lower_bound_.count()),
                            static_cast<float>(pulse_upper_bound_.count()));

    SetPulseWidthInMicroseconds(
        std::chrono::microseconds(static_cast<uint32_t>(pulse_width)));
  }

private:
    const Pwm & mc_pwm_;
    const Gpio & direction_pin_;
    std::chrono::microseconds waveform_period_;
    std::chrono::microseconds pulse_lower_bound_;
    std::chrono::microseconds pulse_upper_bound_;
    units::angular_velocity::revolutions_per_minute_t min_rotation_;
    units::angular_velocity::revolutions_per_minute_t max_rotation_;
};
}