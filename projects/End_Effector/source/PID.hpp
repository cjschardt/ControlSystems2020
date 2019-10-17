#pragma once
#include <stdint.h>
#include <math.h>
#include <utility/stopwatch.hpp>
#include <utility/log.hpp>

namespace sjsu
{
class PID
{
public:
  PID(float *Input, float *Output, float *Setpoint, float KP, float KI, float KD)
  {
    //stopwatch.Calibrate();
    //stopwatch.Start();
    input = Input;
    output = Output;
    set_point = Setpoint;
    Kp = KP;
    Ki = KI * (float(sample_time)/1000);
    Kd = KD / (float(sample_time)/1000);
  }

  void Calculate()
  {
    //printf("Kp: %f, Ki: %f, Kd: %f\n", Kp, Ki, Kd);
    float error = *set_point - *input;
    float dinput = *input - last_input;
    //printf("input: %f, setpoint: %f\n", *input, *set_point);
    //printf("error: %f, dinput: %f\n", error, dinput);
    float ki_adjust = Ki * error;
    //printf("ki_adjust: %f\n", ki_adjust);
    output_sum += ki_adjust;
    //printf("output sum: %f\n", output_sum);
    //output_sum -= Kp * dinput;

    output_sum = (output_sum > out_max) ? out_max:output_sum;
    output_sum = (output_sum < out_min) ? out_min:output_sum;
    //printf("output_sum: %f\n", output_sum);

    float out = Kp * error;
    //printf("out: %f\n", out);
    //printf("Kd * dinput = %f * %f = %f\n", Kd, dinput, Kd*dinput);
    //printf("out + output_sum - (Kd * dinput) = %f + %f - (%f * %f) = %f\n", out, output_sum, Kd, dinput, (out+(output_sum -(Kd*dinput))));
    float kd_adjust = Kd * dinput;
    //printf("output_sum - kd_adjust = %f - %f = %f\n", output_sum, kd_adjust, output_sum - kd_adjust);
    out += output_sum - kd_adjust;
    //printf("out: %f\n", out);
    out = (out > out_max) ? out_max:out;
    out = (out < out_min) ? out_min:out;
    //printf("out: %f, output: %f\n", out,*output);
    *output = out;
  return;
  }

  void Tune(float KP, float KI, float KD)
  {
    Kp = KP;
    Ki = KI * (sample_time/1000);
    Kd = KD / (sample_time/1000);
    return;
  }

  void SetLimits(float min, float max)
  {
    if(min < max)
    {
      out_min = min;
      out_max = max;
    }
    else
    {
        LOG_ERROR("Minimum value must be less than Maximum value");
    }
    return;
  }

  void SetSampleRate(uint32_t sample)
  {
    if(sample > 0)
    {
      Ki *= sample/sample_time;
      Kd /= sample/sample_time;
      sample_time = sample;
    }
    else
    {
      LOG_ERROR("Sample period must be greater than 0");
    }
}

private:
  //uint32_t (*funct_ptr)();
  //sjsu::StopWatch stopwatch(funct_ptr);
  uint32_t sample_time = 100;
  float last_input = 0;
  float *input;
  float *output;
  float *set_point;
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float output_sum = 0;
  float out_min = 0;
  float out_max = 255;
};
}