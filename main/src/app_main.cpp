#include "esp_err.h"
#include "esp_system.h"
#include "max.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <machine/endian.h>
#include <span>
#include <ranges>
#include <tuple>
#include <flash.hpp>
#include <tl/expected.hpp>
#include <memory>

namespace app {
void IRAM_ATTR delay_us(uint32_t us) {
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

void delay_ms(uint32_t ms) {
	vTaskDelay(ms / portTICK_PERIOD_MS);
};


void print_as_hex(std::span<const uint8_t> data) {
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
} // namespace app

// Wait timeout, in ms. Note: -1 means wait forever.
constexpr auto DEFAULT_I2C_TIMEOUT_MS = 5'000;
extern "C" [[noreturn]]
void app_main();

[[noreturn]]
void app_main() {
	using namespace app;
	using namespace tl;
	using namespace max;
	constexpr auto TAG = "main";
	delay_ms(200);

	i2c_master_bus_config_t i2c_mst_config = {
		.i2c_port          = I2C_NUM_0,
		.sda_io_num        = GPIO_NUM_10,
		.scl_io_num        = GPIO_NUM_11,
		.clk_source        = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags             = {.enable_internal_pullup = true},
	};
	i2c_master_bus_handle_t bus_handle;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
	ESP_ERROR_CHECK(i2c_master_bus_reset(bus_handle));

	// https://github.com/espressif/esp-idf/blob/fb25eb02ebcf78a78b4c34a839238a4a56accec7/examples/peripherals/i2c/i2c_tools/main/cmd_i2ctools.c#L109
	const auto i2c_detect = [bus_handle]() {
		constexpr auto I2C_TOOL_TIMEOUT_VALUE_MS = 50;
		uint8_t address;
		printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
		for (int i = 0; i < 128; i += 16) {
			printf("%02x: ", i);
			for (int j = 0; j < 16; j++) {
				fflush(stdout);
				address       = i + j;
				esp_err_t ret = i2c_master_probe(bus_handle, address, I2C_TOOL_TIMEOUT_VALUE_MS);
				if (ret == ESP_OK) {
					printf("%02x ", address);
				} else if (ret == ESP_ERR_TIMEOUT) {
					printf("UU ");
				} else {
					printf("-- ");
				}
			}
			printf("\r\n");
		}

		return 0;
	};

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address  = 0x55,
		.scl_speed_hz    = 400'000,
	};
	i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

	constexpr auto PIN_RESET = GPIO_NUM_22;
	constexpr auto PIN_MFIO  = GPIO_NUM_2;

	gpio_config_t io_conf = {};
	io_conf.intr_type     = GPIO_INTR_DISABLE;
	io_conf.mode          = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask  = PIN_RESET | PIN_MFIO;
	io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	gpio_set_level(PIN_RESET, 0);
	gpio_set_level(PIN_MFIO, 1);
	delay_ms(10);
	gpio_set_level(PIN_RESET, 1);
	delay_ms(1'500);
	ESP_LOGI(TAG, "ready");
	i2c_detect();

	const auto read_command =
		[dev_handle](uint8_t family_byte, uint8_t index_byte,
					 std::span<uint8_t> out,
					 uint16_t wait_time_ms = 2,
					 bool is_sleep_on_end  = false) -> esp_err_t {
		esp_err_t err;
		gpio_set_level(PIN_MFIO, 0);
		delay_ms(1);
		const auto on_end = [is_sleep_on_end] {
			if (is_sleep_on_end) {
				gpio_set_level(PIN_MFIO, 1);
			} else {
				gpio_set_level(PIN_MFIO, 0);
			}
		};
		const std::array<uint8_t, 2> w_data = {family_byte, index_byte};
		err                                 = i2c_master_transmit(dev_handle, w_data.data(), w_data.size(), DEFAULT_I2C_TIMEOUT_MS);
		if (err != ESP_OK) {
			on_end();
			return err;
		}
		delay_ms(wait_time_ms);
		err = i2c_master_receive(dev_handle, out.data(), out.size(), DEFAULT_I2C_TIMEOUT_MS);
		if (err != ESP_OK) {
			on_end();
			return err;
		}
		on_end();
		return ESP_OK;
	};

	// write register with external buffer, also fill the return value to the
	// out buffer
	const auto write_command_ext_buf_with_ret =
		[dev_handle](
			std::span<const uint8_t> in,
			std::span<uint8_t> out,
			uint16_t wait_time_ms = 2,
			bool is_sleep_on_end  = false) -> esp_err_t {
		esp_err_t err;

		gpio_set_level(PIN_MFIO, 0);
		delay_ms(1);
		const auto on_end = [is_sleep_on_end] {
			if (is_sleep_on_end) {
				gpio_set_level(PIN_MFIO, 1);
			} else {
				gpio_set_level(PIN_MFIO, 0);
			}
		};

		err = i2c_master_transmit(dev_handle, in.data(), in.size(), DEFAULT_I2C_TIMEOUT_MS);
		if (err != ESP_OK) {
			on_end();
			return err;
		}
		delay_ms(wait_time_ms);
		err = i2c_master_receive(dev_handle, out.data(), out.size(), DEFAULT_I2C_TIMEOUT_MS);
		if (err != ESP_OK) {
			on_end();
			return err;
		}
		on_end();
		return ESP_OK;
	};

	// write register with external buffer (note that the first two bytes of `in` should be the family and index bytes)
	const auto write_command_ext_buf =
		[write_command_ext_buf_with_ret](
			std::span<const uint8_t> in,
			uint16_t wait_time_ms = 2,
			bool is_sleep_on_end  = false) -> expected<uint8_t, esp_err_t> {
		using ue = unexpected<esp_err_t>;
		esp_err_t err;
		// for status
		uint8_t out[1];

		err = write_command_ext_buf_with_ret(in, out, wait_time_ms, is_sleep_on_end);
		if (err != ESP_OK) {
			return ue{err};
		}
		return out[0];
	};


	// write register with a single byte of data
	const auto write_command_byte =
		[write_command_ext_buf](uint8_t family_byte, uint8_t index_byte, uint8_t data,
								uint16_t wait_time_ms = 2,
								bool is_sleep_on_end  = false) {
			uint8_t in[3] = {family_byte, index_byte, data};
			return write_command_ext_buf(in, wait_time_ms, is_sleep_on_end);
		};

	const auto write_bootloader = [=]() {
		constexpr auto OK  = max::SUCCESS;
		constexpr auto TAG = "bl";
		{
			uint8_t out[2]{};
			esp_err_t err = read_command(max::FMY_DEV_MODE_R, max::IDX_DEV_MODE_R, out, 10);
			ESP_ERROR_CHECK(err);
			if (auto status = out[0]; out[1] != max::DEV_MODE_R_BL || status != OK) {
				ESP_LOGE(TAG, "query error or not in bootloader mode; out(mode)=(0x%02x, %d)", out[0], out[1]);
				return ESP_FAIL;
			}
		}
		{
			// get ID and MCU type
			uint8_t out[2]{};
			esp_err_t err = read_command(0xff, 0x00, out);
			ESP_ERROR_CHECK(err);
			if (out[0] != OK) {
				ESP_LOGE(TAG, "can't get MCU type; out(mcu)=(0x%02x, 0x%02x)", out[0], out[1]);
				return ESP_FAIL;
			} else {
				ESP_LOGI(TAG, "mcu type=0x%02x", out[1]);
			}
		}
		{
			const auto num_of_pages = max::flash::number_of_pages();
			// write number of page
			const uint8_t in[] = {max::flash::FMY_BL_W, max::flash::IDX_BL_W_PAGE_NUM, 0x00, num_of_pages};
			auto status        = write_command_ext_buf(in);
			if (!status) {
				ESP_LOGE(TAG, "failed to write number of pages; write_command_ext_buf error=%d", status.error());
				return status.error();
			} else if (status.value() != OK) {
				ESP_LOGE(TAG, "failed to write number of pages; status=%d", status.value());
				return ESP_FAIL;
			}
			ESP_LOGI(TAG, "number of pages %d written", num_of_pages);

			// use heap to avoid stack overflow, use unique_ptr to enable RAII behavior
			auto wr_buf_ptr   = std::make_unique<uint8_t[]>(2 + max::flash::MAX_PAGE_SIZE + max::flash::CHECKBYTES_SIZE);
			const auto wr_buf = std::span(wr_buf_ptr.get(), 2 + max::flash::MAX_PAGE_SIZE + max::flash::CHECKBYTES_SIZE);
			// write nonce
			wr_buf[0] = max::flash::FMY_BL_W;
			wr_buf[1] = max::flash::IDX_BL_W_NONCE;
			std::ranges::copy(max::flash::init_vector_bytes(), wr_buf.data() + 2);
			status = write_command_ext_buf(std::span(wr_buf.data(), 2 + max::flash::init_vector_bytes().size()));
			if (!status) {
				ESP_LOGE(TAG, "failed to write nonce; write_command_ext_buf error=%d", status.error());
				return status.error();
			} else if (status.value() != OK) {
				ESP_LOGE(TAG, "failed to write nonce; status=%d", status.value());
				return ESP_FAIL;
			}
			ESP_LOGI(TAG, "nonce written");

			// write auth
			wr_buf[0] = max::flash::FMY_BL_W;
			wr_buf[1] = max::flash::IDX_BL_W_AUTH;
			std::ranges::copy(max::flash::auth_bytes(), wr_buf.data() + 2);
			status = write_command_ext_buf(std::span(wr_buf.data(), 2 + max::flash::auth_bytes().size()), 10);
			if (!status) {
				ESP_LOGE(TAG, "failed to write auth; write_command_ext_buf error=%d", status.error());
				return status.error();
			} else if (status.value() != OK) {
				ESP_LOGE(TAG, "failed to write auth; status=%d", status.value());
				return ESP_FAIL;
			}
			ESP_LOGI(TAG, "auth written");

			const auto erase = [=]() {
				wr_buf[0]   = max::flash::FMY_BL_W;
				wr_buf[1]   = max::flash::IDX_BL_W_ERASE_APP;
				auto status = write_command_ext_buf(std::span(wr_buf.data(), 2), 1'400);
				if (!status) {
					ESP_LOGE(TAG, "failed to erase app; write_command_ext_buf error=%d", status.error());
					return status.error();
				} else if (status.value() != OK) {
					ESP_LOGE(TAG, "failed to erase app; status=0x%02x", status.value());
					return ESP_FAIL;
				}
				return ESP_OK;
			};
			// erase app
			constexpr auto RETRY_DELAY_MS = 100;
			while (erase() != ESP_OK) {
				delay_ms(RETRY_DELAY_MS);
			}

			ESP_LOGI(TAG, "app erased");
			delay_ms(500);

			// send pages
			wr_buf[0]       = max::flash::FMY_BL_W;
			wr_buf[1]       = max::flash::IDX_BL_W_SEND_PAGE;
			uint32_t offset = max::flash::APP_START_OFFSET;
			for (uint16_t i = 0; i < num_of_pages; i++) {
				constexpr auto PAGE_SIZE = max::flash::MAX_PAGE_SIZE + max::flash::CHECKBYTES_SIZE;
				const auto page          = max::flash::msbl().subspan(offset + i * PAGE_SIZE, PAGE_SIZE);
				printf("first 16 bytes of page %d=", i);
				print_as_hex(page.subspan(0, 16));
				printf("last 16 bytes of page %d=", i);
				print_as_hex(page.subspan(page.size() - 16, 16));
				std::ranges::copy(page, wr_buf.data() + 2);
				const auto write_current_page = [=]() {
					auto e = write_command_ext_buf(std::span(wr_buf.data(), 2 + page.size()), 680);
					if (!e) {
						ESP_LOGE(TAG, "failed to write page %d; write_command_ext_buf error=%d (%s)", i, e.error(), esp_err_to_name(e.error()));
						return e.error();
					} else if (e.value() != OK) {
						ESP_LOGE(TAG, "failed to write page %d; status=0x%02x", i, e.value());
						return ESP_FAIL;
					}
					ESP_LOGI(TAG, "page %d written", i);
					return ESP_OK;
				};
				while (write_current_page() != ESP_OK) {
					delay_ms(RETRY_DELAY_MS);
					ESP_LOGW(TAG, "retry page %d", i);
				}
			}
		}

		ESP_LOGI(TAG, "bootloader written");
		auto status = write_command_byte(max::FMY_DEV_MODE_W, max::IDX_DEV_MODE_W, max::DEV_MODE_W_EXIT_BL, 50);
		if (!status) {
			ESP_LOGE(TAG, "failed to exit bootloader; write_command_byte error=%d", status.error());
			return status.error();
		} else if (status.value() != OK) {
			ESP_LOGE(TAG, "failed to exit bootloader; status=%d", status.value());
			return ESP_FAIL;
		}
		return ESP_OK;
	};


init_retry:
	max::DevModeR mode = max::DEV_MODE_R_APP;
	{
		uint8_t out[2];
		auto esp_err = read_command(max::FMY_DEV_MODE_R, max::IDX_DEV_MODE_R, out, 10);
		if (auto status = out[0]; esp_err != ESP_OK || status != max::SUCCESS) {
			ESP_LOGW(TAG, "err='%s'(%d), mode=(%d, %d)", esp_err_to_name(esp_err), status, out[0], out[1]);
			delay_ms(1'000);
			goto init_retry;
		}
		mode = static_cast<max::DevModeR>(out[1]);
	}

	const auto enter_bootloader_by_pin = [] {
		// set RSTN low
		gpio_set_level(PIN_RESET, 0);
		// While RSTN is low, set the MFIO pin to low.
		gpio_set_level(PIN_MFIO, 0);
		// The MFIO pin should be set to low at least 1ms before the RSTN pin is set to high.
		// After the 10ms has elapsed, set the RSTN pin to high.
		delay_ms(20);
		gpio_set_level(PIN_RESET, 1);
		// After an additional 50ms has elapsed, the sensor hub is in Bootloader mode.
		delay_ms(60);
	};
	const auto enter_bootloader_by_command = [=]() {
		return write_command_byte(max::FMY_DEV_MODE_W, max::IDX_DEV_MODE_W, max::DEV_MODE_W_ENTER_BL, 50);
	};

	// check version/bootloader mode, decide whether to write the bootloader
	if (mode == max::DEV_MODE_R_BL) {
		ESP_LOGW(TAG, "I'm in bootloader mode. try to write application to the chip.");
		const auto msbl = max::flash::msbl();
		ESP_LOGI(TAG, "msbl.size()=%d", msbl.size());
		printf("auth_bytes(%d)=", max::flash::auth_bytes().size());
		print_as_hex(max::flash::auth_bytes());
		printf("init_vector(%d)=", max::flash::init_vector_bytes().size());
		print_as_hex(max::flash::init_vector_bytes());
		ESP_LOGI(TAG, "number_of_pages=%d", max::flash::number_of_pages());
		write_bootloader();
		esp_restart();
	} else if (mode == max::DEV_MODE_R_APP) {
		esp_err_t esp_err;
		ESP_LOGI(TAG, "app mode");
		uint8_t version[4];
		esp_err = read_command(0xFF, 0x03, version);
		ESP_LOGI(TAG, "app(version)=(0x%02x, 0x%02x, 0x%02x, 0x%02x)", version[0], version[1],
				 version[2], version[3]);
		const auto status = version[0];
		// MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30.13.31
		constexpr auto EXPECTED_VERSION_TUPLE = std::array<uint8_t, 3>{30, 13, 31};
		if (esp_err != ESP_OK || status != max::SUCCESS) {
			ESP_LOGE(TAG, "failed to read version; esp_err=%s(%d), status=%d", esp_err_to_name(esp_err), esp_err, status);
			delay_ms(1'000);
			goto init_retry;
		}
		auto v = std::span(version + 1, 3);
		if (std::equal(v.begin(), v.end(), EXPECTED_VERSION_TUPLE.begin())) {
			ESP_LOGI(TAG, "version=%d.%d.%d", v[0], v[1], v[2]);
		} else {
			ESP_LOGW(TAG, "unexpected version; expected=%d.%d.%d, actual=%d.%d.%d",
					 EXPECTED_VERSION_TUPLE[0], EXPECTED_VERSION_TUPLE[1], EXPECTED_VERSION_TUPLE[2], v[0], v[1], v[2]);
			ESP_LOGI(TAG, "enter bootloader mode and rewrite the application");
			enter_bootloader_by_pin();
			delay_ms(500);
			enter_bootloader_by_command();
			delay_ms(500);
			goto init_retry;
		}
	} else {
		ESP_LOGE(TAG, "unknown mode; mode=%d", mode);
		goto init_retry;
	}

	// Automatic Exposure Control (AEC) is Maxim's gain control algorithm that is superior to AGC. The
	// AEC algorithm optimally maintains the best SNR range and power optimization. The targeted
	// SNR range is maintained regardless of skin color or ambient temperature within the limits of the
	// LED currents configurations; The AEC dynamically manages the appropriate register settings for
	// sampling rate, LED current, pulse width and integration time.

	// Skin contact detector (SCD)

	// For hardware testing purposes, the user may choose to start the sensor hub to collect raw PPG
	// samples. In this case, the host configures the sensor hub to work in Raw Data mode (no algorithm)
	// by enabling the accelerometer and the AFE.
	const auto hub_set_fifo_mode = [=](max::FIFO_OUTPUT_MODE mode) {
		return write_command_byte(0x10, 0x00, static_cast<uint8_t>(mode));
	};

	const auto hub_set_fifo_threshold = [=](uint8_t threshold) {
		return write_command_byte(0x10, 0x01, threshold);
	};

	enum class AccelSensorEn : uint16_t {
		DisableHubAccel = 0x0000,
		DisableExtAccel = 0x0001,
		EnableHubAccel  = 0x0100,
		EnableExtAccel  = 0x0101,
	};

	const auto hub_enable_accel = [=](AccelSensorEn mode) {
		// Sensor Mode Enable
		// Enable accelerometer
		const auto mode_val = static_cast<uint16_t>(mode);
		uint8_t buf[4]      = {
            0x44,
            0x04,
            static_cast<uint8_t>(mode_val >> 8),
            static_cast<uint8_t>(mode_val & 0xff),
        };
		return write_command_ext_buf(std::span(buf), 20);
	};

	// Any command to change the sensor registers should appear AFTER enabling the sensor or they will be
	// overridden by the default settings.
	// By default, the algorithm sets the following AFE registers:
	//
	// Sample rate: 100Hz, 1-sample averaging (samples report period: 10 msec)
	// Integration time: 117μs
	// ADCs 1 and 2 range: 32μA
	// LEDs 1, 2, and 3 full range: 124mA
	const auto hub_user_enable_sensors = [=] -> esp_err_t {
		auto err = hub_enable_accel(AccelSensorEn::EnableHubAccel);
		if (!err) {
			ESP_LOGE(TAG, "failed to enable accelerometer; err=%s (%d)", esp_err_to_name(err.error()), err.error());
			return err.error();
		}
		if (err.value() != max::SUCCESS) {
			ESP_LOGE(TAG, "failed to enable accelerometer; status=%d", err.value());
			return ESP_FAIL;
		}
		// Enable the MAX86140/ MAX86141/ MAXM86146/ MAXM86161 sensor.
		uint8_t buf[4] = {0x44, 0x00, 0x01, 0x00};
		err            = write_command_ext_buf(std::span(buf), 250);
		if (!err) {
			ESP_LOGE(TAG, "failed to enable AFE; err=%s (%d)", esp_err_to_name(err.error()), err.error());
			return err.error();
		}
		if (err.value() != max::SUCCESS) {
			ESP_LOGE(TAG, "failed to enable AFE; status=%d", err.value());
			return ESP_FAIL;
		}
		return ESP_OK;
	};

	const auto hub_write_afe_registers = [=](std::span<const uint8_t> buf) -> expected<uint8_t, esp_err_t> {
		using ue          = unexpected<esp_err_t>;
		constexpr auto N  = 18;
		constexpr auto NN = N - 2;
		if (buf.size() > NN) {
			return ue{ESP_ERR_INVALID_SIZE};
		}
		std::array<uint8_t, N> buf_ext = {};
		buf_ext[0]                     = 0x40;
		buf_ext[1]                     = 0x00;
		std::ranges::copy(buf, buf_ext.begin() + 2);
		return write_command_ext_buf(std::span(buf_ext.data(), buf_ext.size()), 100);
	};

	// returns true if the operation failed, the result will be logged
	constexpr auto log_when_failed = [](const char *tag, const char *entry, expected<uint8_t, esp_err_t> result) {
		if (!result) {
			ESP_LOGE(tag, "%s; err=%s (%d)", entry, esp_err_to_name(result.error()), result.error());
			return true;
		} else if (result.value() != max::SUCCESS) {
			ESP_LOGE(tag, "%s; status=%d", entry, result.value());
			return true;
		} else {
			ESP_LOGI(tag, "%s; success", entry);
			return false;
		}
	};
	const auto hub_user_configure_afe = [=] -> esp_err_t {
		constexpr auto TAG             = "afe";
		std::array<uint8_t, 2> afe_buf = {};

		// set the sample rate to 100Hz with 1 sample averaging
		afe_buf  = {0x12, 0x18};
		auto res = hub_write_afe_registers(afe_buf);
		if (log_when_failed(TAG, "set sample rate", res)) {
			return ESP_FAIL;
		}

		// set the LED current to half of full scale
		// reduce the value if saturation is observed
		afe_buf = {0x23, 0x7f};
		if (log_when_failed(TAG, "set LED1 current", hub_write_afe_registers(afe_buf))) {
			return ESP_FAIL;
		}

		afe_buf = {0x24, 0x7f};
		if (log_when_failed(TAG, "set LED2 current", hub_write_afe_registers(afe_buf))) {
			return ESP_FAIL;
		}

		afe_buf = {0x25, 0x7f};
		if (log_when_failed(TAG, "set LED3 current", hub_write_afe_registers(afe_buf))) {
			return ESP_FAIL;
		}

		return ESP_OK;
	};

	const auto app_raw_mode_init = [=] {
		if (log_when_failed(TAG, "set FIFO mode", hub_set_fifo_mode(max::FIFO_OUTPUT_MODE::SENSOR_DATA))) {
			return ESP_FAIL;
		}
		if (log_when_failed(TAG, "set FIFO threshold", hub_set_fifo_threshold(0x02))) {
			return ESP_FAIL;
		}
		if (const auto err = hub_user_enable_sensors(); err != ESP_OK) {
			return err;
		}
		if (const auto err = hub_user_configure_afe(); err != ESP_OK) {
			return err;
		}
		return ESP_OK;
	};

	struct SensorHubStatus {
		bool comm_error : 1;       // Bit 0: Sensor comm error
		bool reserved_1 : 2;       // Bits 1 and 2: Reserved
		bool data_rdy_int : 1;     // Bit 3: FIFO filled to threshold (DataRdyInt)
		bool fifo_out_ovr_int : 1; // Bit 4: Output FIFO overflow (FifoOutOvrInt)
		bool fifo_in_ovr_int : 1;  // Bit 5: Input FIFO overflow (FifoInOverInt)
		bool dev_busy : 1;         // Bit 6: Sensor hub busy (DevBusy)
		bool reserved_2 : 1;       // Bit 7: Reserved
	} __attribute__((packed));
	const auto hub_status = [=] -> expected<SensorHubStatus, esp_err_t> {
		using ue = unexpected<esp_err_t>;
		uint8_t buf[2];
		esp_err_t esp_err = read_command(0x00, 0x00, buf);
		if (esp_err != ESP_OK) {
			ESP_LOGE(TAG, "failed to read sensor hub status; esp_err=%s (%d)", esp_err_to_name(esp_err), esp_err);
			return ue{esp_err};
		}
		if (buf[0] != max::SUCCESS) {
			ESP_LOGE(TAG, "failed to read sensor hub status; status=%d", buf[0]);
			return ue{ESP_FAIL};
		}
		return *reinterpret_cast<SensorHubStatus *>(&buf[1]);
	};

	const auto hub_get_number_of_samples = [=] -> expected<uint8_t, esp_err_t> {
		using ue = unexpected<esp_err_t>;
		uint8_t buf[2];
		esp_err_t esp_err = read_command(0x12, 0x00, buf);
		if (esp_err != ESP_OK) {
			ESP_LOGE(TAG, "failed to read sensor hub number of samples; esp_err=%s (%d)", esp_err_to_name(esp_err), esp_err);
			return ue{esp_err};
		}
		if (buf[0] != max::SUCCESS) {
			ESP_LOGE(TAG, "failed to read sensor hub number of samples; status=%d", buf[0]);
			return ue{ESP_FAIL};
		}
		return buf[1];
	};

	static constexpr auto RAW_SAMPLE_SIZE = 3 * 6 + 3 * 2;
	struct raw_sample_t {
		uint32_t green;
		uint32_t ir;
		uint32_t red;

		// two's complement, LSB=0.001g
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;

		static expected<raw_sample_t, esp_err_t> unmarshal(std::span<const uint8_t> buf) {
			using ue = unexpected<esp_err_t>;
			if (buf.size() < RAW_SAMPLE_SIZE) {
				return ue{ESP_ERR_INVALID_SIZE};
			}
			// 3 bytes for each ppg, MSB first
			const uint32_t ppg1 = static_cast<uint32_t>(buf[0]) << 16 | static_cast<uint32_t>(buf[1]) << 8 | static_cast<uint32_t>(buf[2]); // green
			const uint32_t ppg2 = static_cast<uint32_t>(buf[3]) << 16 | static_cast<uint32_t>(buf[4]) << 8 | static_cast<uint32_t>(buf[5]); // ir
			const uint32_t ppg3 = static_cast<uint32_t>(buf[6]) << 16 | static_cast<uint32_t>(buf[7]) << 8 | static_cast<uint32_t>(buf[8]); // red

			const uint32_t ppg4 = static_cast<uint32_t>(buf[9]) << 16 | static_cast<uint32_t>(buf[10]) << 8 | static_cast<uint32_t>(buf[11]);  // green2, N/A for MAX86141
			const uint32_t ppg5 = static_cast<uint32_t>(buf[12]) << 16 | static_cast<uint32_t>(buf[13]) << 8 | static_cast<uint32_t>(buf[14]); // N/A
			const uint32_t ppg6 = static_cast<uint32_t>(buf[15]) << 16 | static_cast<uint32_t>(buf[16]) << 8 | static_cast<uint32_t>(buf[17]); // N/A

			// 2 bytes for each accel, MSB first, LSB=0.001g
			const int16_t accel_x = static_cast<int16_t>(buf[18]) << 8 | static_cast<int16_t>(buf[19]);
			const int16_t accel_y = static_cast<int16_t>(buf[20]) << 8 | static_cast<int16_t>(buf[21]);
			const int16_t accel_z = static_cast<int16_t>(buf[22]) << 8 | static_cast<int16_t>(buf[23]);
			return raw_sample_t{ppg1, ppg2, ppg3, accel_x, accel_y, accel_z};
		}
	};

	const auto hub_read_raw_sample = [=] -> expected<raw_sample_t, esp_err_t> {
		using ue = unexpected<esp_err_t>;
		uint8_t buf[1 + RAW_SAMPLE_SIZE];
		esp_err_t esp_err = read_command(max::FMY_FIFO_OUTPUT_READ, max::IDX_FIFO_OUTPUT_READ_DATA, buf);
		if (esp_err != ESP_OK) {
			ESP_LOGE(TAG, "failed to read raw sample; esp_err=%s (%d)", esp_err_to_name(esp_err), esp_err);
			return ue{esp_err};
		}
		if (buf[0] != max::SUCCESS) {
			ESP_LOGE(TAG, "failed to read raw sample; status=%d", buf[0]);
			return ue{ESP_FAIL};
		}
		return raw_sample_t::unmarshal(std::span(buf + 1, RAW_SAMPLE_SIZE));
	};

	// Host reads samples periodically (do not execute at a faster rate than the samples report period)
	// The host is required to periodically check the sensor hub for an available samples report. The default samples report period is 40ms which means sample rate is 25Hz
	const auto app_raw_iter = [=] {
		auto status_ = hub_status();
		if (!status_) {
			ESP_LOGE(TAG, "failed to get sensor hub status; err=%s (%d)", esp_err_to_name(status_.error()), status_.error());
			return;
		}
		const auto status = *status_;
		if (not status.data_rdy_int) {
			ESP_LOGE(TAG, "no data; status(comm_error=%d, data_rdy_int=%d, fifo_out_ovr_int=%d, fifo_in_ovr_int=%d, dev_busy=%d) (0x%02x)",
					 status.comm_error, status.data_rdy_int, status.fifo_out_ovr_int, status.fifo_in_ovr_int, status.dev_busy,
					 *reinterpret_cast<const uint8_t *>(&status));
			return;
		}
		auto fifo_count_ = hub_get_number_of_samples();
		if (!fifo_count_) {
			return;
		}
		auto fifo_count = *fifo_count_;
		if (fifo_count == 0) {
			ESP_LOGE(TAG, "sensor hub FIFO is empty");
			return;
		}
		while (fifo_count > 0) {
			auto sample_ = hub_read_raw_sample();
		}
	};

	// 0x00: invalid
	// 0x01: 40ms
	// 0x02: 80ms
	// ...
	// 0xff: 40ms * 255 (10.2s)
	const auto hub_set_report_period = [=](uint8_t period_40ms) {
		return write_command_byte(0x10, 0x02, period_40ms);
	};

	const auto hub_algo_aec_en = [=](bool en = true) {
		uint8_t buf[] = {
			FMY_ALGO_CFG,
			IDX_ALGO_CFG_WEARABLE_SUIT,
			0x0b,
			en ? static_cast<uint8_t>(0x01) : static_cast<uint8_t>(0x00),
		};
		return write_command_ext_buf(buf);
	};

	const auto hub_algo_mode = [=](max::ALGO_OP_MODE mode) {
		uint8_t buf[] = {
			FMY_ALGO_CFG,
			IDX_ALGO_CFG_WEARABLE_SUIT,
			0x0a,
			static_cast<uint8_t>(mode),
		};
		return write_command_ext_buf(buf);
	};

	const auto hub_algo_pd_current_calculation = [=](bool en = false) {
		uint8_t buf[] = {
			FMY_ALGO_CFG,
			IDX_ALGO_CFG_WEARABLE_SUIT,
			0x12,
			en ? static_cast<uint8_t>(0x01) : static_cast<uint8_t>(0x00),
		};
		return write_command_ext_buf(buf);
	};

	const auto hub_algo_scd_en = [=](bool en = true) {
		uint8_t buf[] = {
			FMY_ALGO_CFG,
			IDX_ALGO_CFG_WEARABLE_SUIT,
			0x0c,
			en ? static_cast<uint8_t>(0x01) : static_cast<uint8_t>(0x00),
		};
		return write_command_ext_buf(buf);
	};

	// (16-bit unsigned integer, 0.1mA)
	const auto hub_algo_set_agc_target_pd_current = [=](uint16_t current) {
		uint8_t buf[] = {
			FMY_ALGO_CFG,
			IDX_ALGO_CFG_WEARABLE_SUIT,
			0x11,
			static_cast<uint8_t>(current >> 8),
			static_cast<uint8_t>(current & 0xff),
		};
		return write_command_ext_buf(buf);
	};

	constexpr auto ENABLE_ALGO_DELAY_MS  = 465;
	const auto hub_algo_report_normal_en = [=]() {
		uint8_t buf[] = {
			FMY_ALGO_EN,
			FMY_ALGO_EN_WEARABLE_SUIT,
			0x01,
		};
		return write_command_ext_buf(buf, ENABLE_ALGO_DELAY_MS);
	};

	const auto hub_algo_report_extended_en = [=]() {
		uint8_t buf[] = {
			FMY_ALGO_EN,
			FMY_ALGO_EN_WEARABLE_SUIT,
			0x02,
		};
		return write_command_ext_buf(buf, ENABLE_ALGO_DELAY_MS);
	};


	const auto algo_mode_init = [=] {
		constexpr auto TAG = "algo_init";
		if (log_when_failed(TAG, "set FIFO mode", hub_set_fifo_mode(max::FIFO_OUTPUT_MODE::SENSOR_AND_ALGO))) {
			return ESP_FAIL;
		}
		if (log_when_failed(TAG, "set FIFO threshold", hub_set_fifo_threshold(0x02))) {
			return ESP_FAIL;
		}
		// 120ms
		if (log_when_failed(TAG, "set report period", hub_set_report_period(0x03))) {
			return ESP_FAIL;
		}
		if (const auto err = hub_user_enable_sensors(); err != ESP_OK) {
			return err;
		}
		if (log_when_failed(TAG, "enable AEC", hub_algo_aec_en(true))) {
			return ESP_FAIL;
		}
		if (log_when_failed(TAG, "set algo mode", hub_algo_mode(max::ALGO_OP_MODE::CONTINUOUS_HRM))) {
			return ESP_FAIL;
		}
		if (log_when_failed(TAG, "disable PD current calculation",
							hub_algo_pd_current_calculation(false))) {
			return ESP_FAIL;
		}
		if (log_when_failed(TAG, "enable SCD", hub_algo_scd_en(true))) {
			return ESP_FAIL;
		}
		// 10mA
		if (log_when_failed(TAG, "set AGC target PD current",
							hub_algo_set_agc_target_pd_current(100))) {
			return ESP_FAIL;
		}
		if (log_when_failed(TAG, "enable report normal", hub_algo_report_normal_en())) {
			return ESP_FAIL;
		}
		return ESP_OK;
	};

	enum class HubAlgoReportType {
		Normal   = 0x01,
		Extended = 0x02,
		ScdOnly  = 0x03,
	};

	struct __attribute__((packed)) algo_model_data_t {
		// Current operation mode
		max::ALGO_OP_MODE op_mode;
		// 10x calculated heart rate
		uint16_t hr;
		// Confidence level in %, >40 is for consumer devices, >80,90 is for medical devices
		uint8_t hr_conf;
		// 10x RR – inter-beat interval in ms; Only shows a nonzero value when a new value is calculated.
		uint16_t rr;
		// Calculated confidence level of RR in %; Only shows a nonzero value when a new value is calculated.
		uint8_t rr_conf;
		// Activity class (Applicable to wrist form factor only, MAX86141/0)
		max::ACTIVATE_CLASS activity_class;
		// 1000x calculated SpO2 R value
		uint16_t r;
		// SpO2 confidence level in %, >40 is for consumer devices, >80,90 is for medical devices
		uint8_t spo2_conf;
		// 10x calculated SpO2 %
		uint16_t spo2;
		// Calculation progress in % in one-shot mode of algorithm. In
		// continuous mode, it is reported as zero and only jumps to 100
		// when the SpO2 value is updated.
		//
		// - Bit[7]: SpO2 valid
		// - Bit[6..0]: Percent complete
		uint8_t spo2_percent_complete;
		// 0: Good quality
		// 1: Low quality
		uint8_t spo2_low_signal_quality_flag;
		// 0: No motion
		// 1: Excessive motion
		uint8_t spo2_motion_flag;
		// Shows the low perfusion index (PI) of the PPG signal
		// 0: Normal PI; 1: Low PI
		uint8_t spo2_low_pi_flag;
		// Shows the reliability of R
		// 0: Reliable; 1: Unreliable
		uint8_t spo2_unreliable_r_flag;
		// Reported status of the SpO2 algorithm
		max::SPO2_STATE spo2_state;
		// Skin contact state
		max::SCD_STATE scd_contact_state;
		// IBI Offset
		// Unreliable orientation flag
		// Reserved
		uint32_t reserved;

		// make all of uint16_t field to native endian
		// useful when you're reinterpret cast the data buffer
		static algo_model_data_t fix_endianness(algo_model_data_t data) {
			data.hr   = __ntohs(data.hr);
			data.rr   = __ntohs(data.rr);
			data.r    = __ntohs(data.r);
			data.spo2 = __ntohs(data.spo2);
			return data;
		}

		float hr_f() const {
			return static_cast<float>(hr) / 10;
		}

		float spo2_f() const {
			return static_cast<float>(spo2) / 10;
		}

		float r_f() const {
			return static_cast<float>(r) / 1000;
		}

		float rr_f() const {
			return static_cast<float>(rr) / 10;
		}
	};


	constexpr auto ALGO_REPORT_DATA_SIZE = RAW_SAMPLE_SIZE + sizeof(algo_model_data_t);
	struct algo_report_t {
		uint32_t green;
		uint32_t ir;
		uint32_t red;
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;
		algo_model_data_t data;

		static expected<algo_report_t, esp_err_t> unmarshal(std::span<const uint8_t> buf) {
			using ue = unexpected<esp_err_t>;
			if (buf.size() < ALGO_REPORT_DATA_SIZE) {
				return ue{ESP_ERR_INVALID_SIZE};
			}

			// 3 bytes for each ppg, MSB first
			const uint32_t ppg1 = static_cast<uint32_t>(buf[0]) << 16 | static_cast<uint32_t>(buf[1]) << 8 | static_cast<uint32_t>(buf[2]); // green
			const uint32_t ppg2 = static_cast<uint32_t>(buf[3]) << 16 | static_cast<uint32_t>(buf[4]) << 8 | static_cast<uint32_t>(buf[5]); // ir
			const uint32_t ppg3 = static_cast<uint32_t>(buf[6]) << 16 | static_cast<uint32_t>(buf[7]) << 8 | static_cast<uint32_t>(buf[8]); // red

			const uint32_t ppg4 = static_cast<uint32_t>(buf[9]) << 16 | static_cast<uint32_t>(buf[10]) << 8 | static_cast<uint32_t>(buf[11]);  // green2, N/A for MAX86141
			const uint32_t ppg5 = static_cast<uint32_t>(buf[12]) << 16 | static_cast<uint32_t>(buf[13]) << 8 | static_cast<uint32_t>(buf[14]); // N/A
			const uint32_t ppg6 = static_cast<uint32_t>(buf[15]) << 16 | static_cast<uint32_t>(buf[16]) << 8 | static_cast<uint32_t>(buf[17]); // N/A

			// 2 bytes for each accel, MSB first, LSB=0.001g
			const int16_t accel_x = static_cast<int16_t>(buf[18]) << 8 | static_cast<int16_t>(buf[19]);
			const int16_t accel_y = static_cast<int16_t>(buf[20]) << 8 | static_cast<int16_t>(buf[21]);
			const int16_t accel_z = static_cast<int16_t>(buf[22]) << 8 | static_cast<int16_t>(buf[23]);

			auto algo_data = *reinterpret_cast<const algo_model_data_t *>(buf.data() + 24);
			return algo_report_t{
				ppg1,
				ppg2,
				ppg3,
				accel_x,
				accel_y,
				accel_z,
				algo_data,
			};
		}
	};

	const auto hub_algo_read_report = [=] -> expected<algo_report_t, esp_err_t> {
		using ue = unexpected<esp_err_t>;
		uint8_t buf[1 + ALGO_REPORT_DATA_SIZE];
		esp_err_t esp_err = read_command(max::FMY_FIFO_OUTPUT_READ, max::IDX_FIFO_OUTPUT_READ_DATA, buf);
		if (esp_err != ESP_OK) {
			ESP_LOGE(TAG, "failed to read algo report; esp_err=%s (%d)", esp_err_to_name(esp_err), esp_err);
			return ue{esp_err};
		}
		if (const auto status = buf[0]; status != max::SUCCESS) {
			ESP_LOGE(TAG, "failed to read algo report; status=%d", status);
			return ue{ESP_FAIL};
		}
		auto raw_ = algo_report_t::unmarshal(std::span(buf + 1, ALGO_REPORT_DATA_SIZE));
		if (!raw_) {
			return ue{raw_.error()};
		}
		auto raw = *raw_;
		raw.data = algo_model_data_t::fix_endianness(raw.data);
		return raw;
	};

	const auto algo_iter = [=]() {
		const auto status_ = hub_status();
		if (!status_) {
			ESP_LOGE(TAG, "failed to get sensor hub status; err=%s (%d)", esp_err_to_name(status_.error()), status_.error());
			return;
		}
		const auto status = *status_;
		if (not status.data_rdy_int) {
			ESP_LOGE(TAG, "no data; status(comm_error=%d, data_rdy_int=%d, fifo_out_ovr_int=%d, fifo_in_ovr_int=%d, dev_busy=%d) (0x%02x)",
					 status.comm_error, status.data_rdy_int, status.fifo_out_ovr_int, status.fifo_in_ovr_int, status.dev_busy,
					 *reinterpret_cast<const uint8_t *>(&status));
			return;
		}
		auto fifo_count_ = hub_get_number_of_samples();
		if (!fifo_count_) {
			return;
		}
		auto fifo_count = *fifo_count_;
		if (fifo_count == 0) {
			ESP_LOGE(TAG, "sensor hub FIFO is empty");
			return;
		}
		while (fifo_count > 0) {
			auto report_ = hub_algo_read_report();
			if (!report_) {
				ESP_LOGE(TAG, "failed to read algo report; err=%s (%d)", esp_err_to_name(report_.error()), report_.error());
				return;
			}
			const auto report = *report_;
			ESP_LOGI(TAG, "green=%" PRIu32 " ir=%" PRIu32 " red=%" PRIu32 " accel=[%" PRIi16 ", %" PRIi16 ", %" PRIi16 "] "
						  "hr=%.1f conf=%d%% rr=%.1f rr_conf=%d%% activity=%s "
						  "spo2=%.1f%% spo2_conf=%d%% r=%.3f "
						  "flags(low_sig=%d motion=%d low_pi=%d r_unreliable=%d) "
						  "spo2_state=%s scd_state=%s",
					 report.green, report.ir, report.red,
					 report.accel_x, report.accel_y, report.accel_z,
					 report.data.hr_f(),
					 report.data.hr_conf,
					 report.data.rr_f(),
					 report.data.rr_conf,
					 max::activate_class_to_string(report.data.activity_class),
					 report.data.spo2_f(),
					 report.data.spo2_conf,
					 report.data.r_f(),
					 report.data.spo2_low_signal_quality_flag,
					 report.data.spo2_motion_flag,
					 report.data.spo2_low_pi_flag,
					 report.data.spo2_unreliable_r_flag,
					 max::spo2_state_to_string(report.data.spo2_state),
					 max::scd_state_to_string(report.data.scd_contact_state));
			fifo_count--;
		}
	};

	// app_raw_mode_init();
	algo_mode_init();
	ESP_LOGI(TAG, "initialized");
	while (true) {
		delay_ms(240);
		// app_raw_iter();
		algo_iter();
	}
}
