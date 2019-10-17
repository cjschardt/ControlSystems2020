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
	PID(double *Input, double *Output, double *Setpoint, double KP, double KI,
		double KD)
	{
		//stopwatch.Calibrate();
		//stopwatch.Start();
		input = Input;
		output = Output;
		set_point = Setpoint;
		Kp = KP;
		Ki = KI * (sample_time/1000);
		Kd = KD / (sample_time/1000);
	}

	void Calculate()
	{
	    //uint32_t time_change = stopwatch.Stop();

	    //if(time_change > sample_time)
	    //{
	        double error = *set_point - *input;
	        double dinput = *input - last_input;
	        output_sum += Kp * error;

	        output_sum = (output_sum > out_max) ? out_max:output_sum;
	        output_sum = (output_sum < out_min) ? out_min:output_sum;

	        double out = Kp * error;

	        out += output_sum - Kd * dinput;
	        out_sum = (out > out_max) ? out_max:out;
	        out_sum = (out < out_min) ? out_min:out;

	        *output = out;
	    //}
	    //else
	    //{
	    //    LOG_ERROR("Time between samples is longer than the stated period \
	        	      limit");
	    //}
	    //stopwatch.Start();
	    return;
	}

	void Tune(double KP, double KI, double KD)
	{
	    Kp = KP;
	    Ki = KI * (sample_time/1000);
	    Kd = KD / (sample_time/1000);
	    return;
	}

	void SetLimits(double min, double max)
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
	double last_input;
	double *input;
	double *output;
	double *set_point;
	double Kp;
	double Ki;
	double Kd;
	double output_sum;
	double out_sum;
	double out_min = 0;
	double out_max = 255;
};
}