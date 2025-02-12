#ifndef FAEBFA20_CF33_4D49_BD6D_17BFAAC81249
#define FAEBFA20_CF33_4D49_BD6D_17BFAAC81249

#include <esp_err.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <cstdint>
#include <cstdio>
#include <span>
#include <ranges>
#include <tuple>
#include <freertos/FreeRTOS.h>

namespace utils {
inline void IRAM_ATTR delay_us(uint32_t us) {
	constexpr auto micros = [] { return esp_timer_get_time(); };
	constexpr auto nop    = [] { __asm__ __volatile__("nop"); };
	auto m                = micros();
	if (us) {
		auto e = (m + us);
		[[unlikely]]
		if (m > e) { // overflow
			while (micros() > e) {
				nop();
			}
		} else {
			while (micros() < e) {
				nop();
			}
		}
	}
}

inline void delay_ms(uint32_t ms) {
	vTaskDelay(ms / portTICK_PERIOD_MS);
};

inline uint32_t millis() {
	return esp_timer_get_time() / 1000;
}

inline uint32_t micros() {
	return esp_timer_get_time();
}

inline void print_as_hex(std::span<const uint8_t> data) {
	const auto enumerate = [](const auto &data) {
		return data | std::views::transform([i = 0](const auto &value) mutable {
				   return std::make_tuple(i++, value);
			   });
	};
	for (const auto [i, byte] : enumerate(data)) {
		bool is_end = i == data.size() - 1;
		if (is_end) {
			printf("%02x\n", byte);
		} else {
			if (i % 16 == 15) {
				printf("%02x\n", byte);
			} else {
				printf("%02x ", byte);
			}
		}
	}
}

template <typename T>
inline std::span<const uint8_t> as_bytes(const T &value) {
	return std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(&value), sizeof(value));
}
}

#endif /* FAEBFA20_CF33_4D49_BD6D_17BFAAC81249 */
