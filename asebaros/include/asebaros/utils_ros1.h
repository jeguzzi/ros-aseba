#ifndef ASEBAROS_INCLUDE_ASEBAROS_UTILSROS1_H_
#define ASEBAROS_INCLUDE_ASEBAROS_UTILSROS1_H_

#include "ros/ros.h"

// #define LOG_INFO(...) ROS_INFO_NAMED("asebaros", __VA_ARGS__);
// #define LOG_WARN(...) ROS_WARN_NAMED("asebaros", __VA_ARGS__);
// #define LOG_ERROR(...) ROS_ERROR_NAMED("asebaros", __VA_ARGS__);
// #define LOG_DEBUG(...) ROS_DEBUG_NAMED("asebaros", __VA_ARGS__);

#define LOG_INFO   ROS_INFO
#define LOG_WARN   ROS_WARN
#define LOG_ERROR  ROS_ERROR
#define LOG_DEBUG  ROS_DEBUG

inline void sleep_for_s(unsigned seconds) {
  ros::Duration(seconds).sleep();
}

inline void sleep_for_ms(unsigned milli_seconds) {
  ros::Duration(milli_seconds * 0.001).sleep();
}

inline void shutdown() { ros::shutdown(); }

inline bool ok() {
  return ros::ok();
}

#endif /* end of include guard: ASEBAROS_INCLUDE_ASEBAROS_UTILSROS1_H_ */
