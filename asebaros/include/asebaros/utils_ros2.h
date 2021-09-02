#ifndef ASEBAROS_INCLUDE_ASEBAROS_UTILSROS2_H_
#define ASEBAROS_INCLUDE_ASEBAROS_UTILSROS2_H_

#include "rclcpp/rclcpp.hpp"

#define LOG_INFO(...) RCLCPP_INFO(rclcpp::get_logger("asebaros"), __VA_ARGS__);
#define LOG_WARN(...) RCLCPP_WARN(rclcpp::get_logger("asebaros"), __VA_ARGS__);
#define LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("asebaros"), __VA_ARGS__);
#define LOG_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("asebaros"), __VA_ARGS__);

inline void sleep_for_s(unsigned seconds) {
  rclcpp::sleep_for(std::chrono::seconds(seconds));
}

inline void sleep_for_ms(unsigned milli_seconds) {
  rclcpp::sleep_for(std::chrono::milliseconds(milli_seconds));
}

inline void shutdown() { rclcpp::shutdown(); }

inline bool ok() {
  return rclcpp::ok();
}

#endif /* end of include guard: ASEBAROS_INCLUDE_ASEBAROS_UTILSROS2_H_ */
