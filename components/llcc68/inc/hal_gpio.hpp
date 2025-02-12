//
// Created by Kurosu Chan on 2024/1/15.
//

#ifndef GPIO_H
#define GPIO_H

#include <driver/gpio.h>

namespace hal::gpio {
using pin_t  = decltype(GPIO_NUM_0);
using mode_t = decltype(GPIO_MODE_OUTPUT);

inline void digital_write(const pin_t pin, const bool val) {
	gpio_set_level(pin, val);
}

inline bool digital_read(const pin_t pin) {
	return gpio_get_level(pin);
}

inline void set_mode(const pin_t pin, const mode_t mode) {
	gpio_set_direction(pin, mode);
}
}

#endif // GPIO_H
