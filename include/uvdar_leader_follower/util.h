#ifndef SSS_UTIL_H
#define SSS_UTIL_H

namespace sss_util {

template <typename T> T saturation(T value, T min, T max) {
  if (value > max) {
    return max;
  }

  if (value < min) {
    return min;
  }

  return value;
}

template <typename T> T deadzone(T value, T min, T max) {
  if (value > max) {
    return value;
  }

  if (value < min) {
    return value;
  }

  return 0;
}
} // namespace sss_util

#endif /* SSS_UTIL_H */