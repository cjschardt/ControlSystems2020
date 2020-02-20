#pragma once
#include <cstdio>
#include <stdint.h>
#include <iterator>
#include "L1_Peripheral/lpc40xx/gpio.hpp"
#include "utility/log.hpp"
#include "utility/status.hpp"

class MUX
{
public:
	MUX(sjsu::lpc40xx::Gpio sel)
	{
		pins = new sjsu::lpc40xx::Gpio[1];
		pins[0] = sel;
		no_pins = 1;
		return;
	}

	MUX(sjsu::lpc40xx::Gpio msb, sjsu::lpc40xx::Gpio lsb)
	{
		pins = new sjsu::lpc40xx::Gpio[2];
		pins[0] = msb;
		pins[1] = lsb;
		no_pins = 2;
		return;
	}

	MUX(sjsu::lpc40xx::Gpio msb, sjsu::lpc40xx::Gpio bsb, sjsu::lpc40xx::Gpio lsb)
	{
		pins = new sjsu::lpc40xx::Gpio[3];
		pins[0] = msb;
		pins[1] = bsb;
		pins[2] = lsb;
		no_pins = 3;
		return;
	}

	void Initialize()
	{
		for(int i = 0; i < no_pins; i++)
		{
			pins[i].SetAsOutput();
			return;
		}
	}

	void Select(uint8_t input)
	{
		switch(no_pins)
		{
			case 1:
			{
				SJ2_ASSERT_WARNING(input < 2, "INPUT CHANNEL IS GREATER THAN MUX SPECIFICATIONS");
				pins[0].Set((bool) input);
				break;
			}
			case 2:
			{
				SJ2_ASSERT_WARNING(input < 4, "INPUT CHANNEL IS GREATER THAN MUX SPECIFICATIONS");
				switch(input)
				{
					case 0:
					{
						pins[0].SetLow();
						pins[1].SetLow();
						break;
					}
					case 1:
					{
						pins[0].SetLow();
						pins[1].SetHigh();
						break;
					}
					case 2:
					{
						pins[0].SetHigh();
						pins[1].SetLow();
						break;
					}
					case 3:
					{
						pins[0].SetHigh();
						pins[1].SetHigh();
					}
					default: break;
				}
				break;
			}
			case 3:
			{
				SJ2_ASSERT_WARNING(input < 8, "INPUT CHANNEL IS GREATER THAN MUX SPECIFICATIONS");
				switch(input)
				{
					case 0:
					{
						pins[0].SetLow();
						pins[1].SetLow();
						pins[2].SetLow();
						break;
					}
					case 1:
					{
						pins[0].SetLow();
						pins[1].SetLow();
						pins[2].SetHigh();
						break;
					}
					case 2:
					{
						pins[0].SetLow();
						pins[1].SetHigh();
						pins[2].SetLow();
						break;
					}
					case 3:
					{
						pins[0].SetLow();
						pins[1].SetHigh();
						pins[2].SetHigh();
						break;
					}
					case 4:
					{
						pins[0].SetHigh();
						pins[1].SetLow();
						pins[2].SetLow();
						break;
					}
					case 5:
					{
						pins[0].SetHigh();
						pins[1].SetLow();
						pins[2].SetHigh();
						break;
					}
					case 6:
					{
						pins[0].SetHigh();
						pins[1].SetHigh();
						pins[2].SetLow();
						break;
					}
					case 7:
					{
						pins[0].SetHigh();
						pins[1].SetHigh();
						pins[2].SetHigh();
						break;
					}
					default: break;
				}
				break;
			}
			default: break;
		}
		return;
	}

	~MUX()
	{
		delete [] pins;
	}

private:
	sjsu::lpc40xx::Gpio *pins;
	int no_pins = 0;
}
