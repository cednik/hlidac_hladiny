#pragma once

#include <cstdint>

#include <Arduino.h>

#include "stopwatch.hpp"

struct base_timer_type
{
	typedef uint32_t time_type;
	time_type value() const { return micros(); }
};
base_timer_type base_timer;

struct stopwatch
	:detail::stopwatch<base_timer_type>
{
	stopwatch(bool run = true)
		:detail::stopwatch<base_timer_type>(base_timer)
	{
		if(run)
			return;
		stop();
		clear();
	}
};

struct timeout
	:detail::timeout<base_timer_type>
{
	timeout(detail::timeout<base_timer_type>::time_type timeout)
		:detail::timeout<base_timer_type>(base_timer, timeout)
	{
	}
};

void wait(base_timer_type::time_type time)
{
	detail::wait(base_timer, time);
}

template <typename Process>
void wait(base_timer_type::time_type time, Process process)
{
	detail::wait(base_timer, time, process);
}

template <typename Process>
void wait(base_timer_type::time_type time, Process process, int)
{
	detail::wait(base_timer, time, process, 0);
}

#define  sec(value) stopwatch::time_type(1000000UL*value)
#define msec(value) stopwatch::time_type(1000UL*value)
#define usec(value) stopwatch::time_type(1UL*value)
