#pragma once
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cassert>
inline double stamp_seconds(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9*t.nanosec;
}
inline builtin_interfaces::msg::Time stamp_from_seconds(double t) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(t*1e9)));
}
