

#pragma once

#include "ros/time.h"

namespace deeproute {

using Time = int64_t;

// Our Time is a 64-bit integer of microseconds. The constants below are for the
// ease of use. For example:
//    Time fifteen_seconds = 15 * kSecond;
//    Time two_hours = 2 * kHour;
constexpr Time kMillisecond = 1000;
constexpr Time kSecond = 1000 * kMillisecond;
constexpr Time kMinute = 60 * kSecond;
constexpr Time kHour = 60 * kMinute;
constexpr Time kDay = 24 * kHour;
constexpr Time kWeek = 7 * kDay;

constexpr Time fromNanoseconds(uint64_t t) { return (t + 500) / 1000; }

constexpr Time fromSeconds(double t) {
  return static_cast<Time>(t * kSecond + 0.5);
}

constexpr Time fromRosTime(const ros::Time &t) {
  return t.sec * kSecond + fromNanoseconds(t.nsec);
}

inline Time now() { return fromRosTime(ros::Time::now()); }

inline ros::Time toRosTime(Time t) {
  return {static_cast<uint32_t>(t / kSecond),
          static_cast<uint32_t>(t % kSecond) * 1000u};
}

inline double timeDiffSeconds(Time t_start, Time t_end) {
  return (double)(t_end - t_start) / (double)kSecond;
}

} // namespace deeproute
