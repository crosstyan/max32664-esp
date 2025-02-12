//
// Created by Kurosu Chan on 2024/1/17.
//
#include "llcc68.hpp"

namespace llcc68 {
namespace details {
	uint32_t __tcxo_delay__;
	transmit_state_t __tx_state__;
	bool __dio_flag__ = false;
}

void config_exti() {
	constexpr auto isr = [] {
		details::__dio_flag__ = true;
	};
	// attachInterrupt(DIO1_PIN, isr, RISING);
}
}
