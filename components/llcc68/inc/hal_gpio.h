//
// Created by Kurosu Chan on 2024/1/15.
//

#ifndef GPIO_H
#define GPIO_H
#include "safe_arduino.h"

namespace hal::gpio {
using pin_t = decltype(PA0);
using mode_t     = decltype(OUTPUT);

inline void digital_write(const pin_t pin, const bool val) {
  digitalWrite(pin, val);
}

inline bool digital_read(const pin_t pin) {
  return digitalRead(pin);
}

inline void set_mode(const pin_t pin, const mode_t mode) {
  pinMode(pin, mode);
}
}

#endif // GPIO_H
