//
// Created by Kurosu Chan on 2023/12/4.
//

#ifndef COMMON_INSTANT_H
#define COMMON_INSTANT_H

#include <chrono>
#include <esp_timer.h>

namespace utils {
/**
 * @brief A measurement of a monotonically nondecreasing clock.
 * @tparam T the data type of the counter
 * @sa https://doc.rust-lang.org/std/time/struct.Instant.html
 */
template <typename T = int64_t>
class Instant {
	T time_;

public:
	using time_t          = T;
	using clock_precision = std::micro;
	using duration_t      = std::chrono::duration<time_t, clock_precision>;
	using ms_t            = std::chrono::duration<time_t, std::milli>;

	static time_t millis() {
		constexpr auto US_PER_MS = 1000;
		const auto now           = esp_timer_get_time();
		return static_cast<time_t>(now / US_PER_MS);
	}

	static time_t micros() {
		return static_cast<time_t>(esp_timer_get_time());
	}

	static time_t clock() {
		return micros();
	}

	Instant() {
		this->time_ = this->clock();
	}

	static Instant now() {
		return Instant{};
	}

	[[nodiscard]]
	duration_t elapsed() const {
		const auto now = static_cast<time_t>(clock());
		if (now < this->time_) {
			// overflow
			return duration_t{now + (std::numeric_limits<time_t>::max() - this->time_)};
		}
		return duration_t{now - this->time_};
	}

	[[nodiscard]]
	time_t elapsed_ms() const {
		return static_cast<time_t>(std::chrono::duration_cast<ms_t>(elapsed()).count());
	}

	[[nodiscard]]
	bool has_elapsed_ms(const time_t ms) const {
		return this->elapsed_ms() >= ms;
	}

	/**
	 * @brief Checks if the specified time interval has elapsed since the last reset.
	 *
	 * This method checks if the elapsed time since the last reset is greater than or equal to the input parameter `ms`.
	 * If it is, the method resets the internal timer and returns true. If not, it simply returns false.
	 * The `[[nodiscard]]` attribute indicates that the compiler will issue a warning if the return value of this function is not used.
	 *
	 * @param ms The time interval in milliseconds.
	 * @return Returns true if the elapsed time is greater than or equal to the input time interval, otherwise returns false.
	 */
	[[nodiscard]]
	bool mut_every_ms(const time_t ms) {
		const bool gt = this->elapsed_ms() >= ms;
		if (gt) {
			this->mut_reset();
			return true;
		}
		return false;
	}

	template <typename Rep, typename Period>
	[[nodiscard]]
	bool has_elapsed(const std::chrono::duration<Rep, Period> duration) const {
		return this->elapsed() >= duration;
	}

	template <typename Rep, typename Period>
	[[nodiscard]]
	bool mut_every(const std::chrono::duration<Rep, Period> duration) {
		const bool gt = this->has_elapsed(duration);
		if (gt) {
			this->mut_reset();
			return true;
		}
		return false;
	}

	void mut_reset() {
		this->time_ = clock();
	}

	/**
	 * @deprecated use `mut_reset` instead
	 */
	[[deprecated("use `mut_reset` instead")]]
	void reset() {
		mut_reset();
	}

	[[nodiscard]]
	duration_t mut_elapsed_and_reset() {
		auto now = static_cast<time_t>(clock());
		decltype(now) diff;
		if (now < this->time_) {
			// overflow
			diff = now + (std::numeric_limits<time_t>::max() - this->time_);
		} else {
			diff = now - this->time_;
		}
		const auto duration = duration_t(diff);
		this->time_         = now;
		return duration;
	}

	[[nodiscard]]
	time_t count() const {
		return time_;
	}
};
}

#endif // COMMON_INSTANT_H