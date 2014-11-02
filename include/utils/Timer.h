/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __UTILS_TIMER_H__
#define __UTILS_TIMER_H__

#include <chrono>

#define TIMER_START(_X) auto _X##_start = std::chrono::steady_clock::now(), _X##_stop = _X##_start
#define TIMER_STOP(_X) _X##_stop = std::chrono::steady_clock::now()
#define TIMER_NSEC(_X) std::chrono::duration_cast<std::chrono::nanoseconds>(_X##_stop - _X##_start).count()
#define TIMER_USEC(_X) std::chrono::duration_cast<std::chrono::microseconds>(_X##_stop - _X##_start).count()
#define TIMER_MSEC(_X) (0.000001*std::chrono::duration_cast<std::chrono::nanoseconds>(_X##_stop - _X##_start).count())
#define TIMER_SEC(_X)  (0.000001*std::chrono::duration_cast<std::chrono::microseconds>(_X##_stop - _X##_start).count())
#define TIMER_MIN(_X) std::chrono::duration_cast<std::chrono::minutes>(_X##_stop - _X##_start).count()

#endif

