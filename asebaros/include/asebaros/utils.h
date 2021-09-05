#ifndef ASEBAROS_INCLUDE_ASEBAROS_UTILS_H_
#define ASEBAROS_INCLUDE_ASEBAROS_UTILS_H_

#include <string>
#include <cwchar>
#include <vector>
#include <sstream>

#if (ROS_VERSION_M == 2)
#include "asebaros/utils_ros2.h"
#else
#include "asebaros/utils_ros1.h"
#endif  // ROS_VERSION

inline std::string list_repr(const std::vector<unsigned> &vs) {
  std::stringstream ss;
  if (vs.size() == 0) {
    return "no node";
  }
  ss << vs.size() << " node";
  if (vs.size() > 1) {
    ss << "s";
  }
  ss << " (";
  for (size_t i = 0; i < vs.size(); ++i) {
    if (i != 0) {
      ss << ", ";
    }
    ss << vs[i];
  }
  ss << ")";
  return ss.str();
}

// UTF8 to wstring
inline std::wstring widen(const char *src) {
  const size_t destSize(mbstowcs(0, src, 0) + 1);
  std::vector<wchar_t> buffer(destSize, 0);
  mbstowcs(&buffer[0], src, destSize);
  return std::wstring(buffer.begin(), buffer.end() - 1);
}

inline std::wstring widen(const std::string &src) { return widen(src.c_str()); }

// wstring to UTF8
inline std::string narrow(const wchar_t *src) {
  const size_t destSize(wcstombs(0, src, 0) + 1);
  std::vector<char> buffer(destSize, 0);
  wcstombs(&buffer[0], src, destSize);
  return std::string(buffer.begin(), buffer.end() - 1);
}

inline std::string narrow(const std::wstring &src) { return narrow(src.c_str()); }

#endif /* end of include guard: ASEBAROS_INCLUDE_ASEBAROS_UTILS_H_ */
