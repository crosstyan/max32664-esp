//
// Created by Kurosu Chan on 2024/1/17.
//
#include "llcc68.h"

namespace llcc68 {
void config_exti() {
	constexpr auto isr = [] {
		details::__dio_flag__ = true;
	};
	// attachInterrupt(DIO1_PIN, isr, RISING);
}
}
