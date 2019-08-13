#pragma once

class IsTime
{
public:
	int default_period; // In milliseconds

	IsTime(int _def_period = 1000);
	bool is_time(int _period = -1);

private:
	ulong prev_time;
};
