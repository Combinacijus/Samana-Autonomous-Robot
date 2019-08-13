#include "Arduino.h"
#include "is_time.h"

IsTime::IsTime(int _def_period)
{
	default_period = _def_period;
	prev_time = millis();
}

bool IsTime::is_time(int _period)
{
	if (_period == -1) // Default value
		_period = default_period;

	if (millis() - prev_time >= _period)
	{
		prev_time = millis();
		return true;
	}

	return false;
}