#include "esp_err.h"
#include "esp_system.h"
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
#include <span>
#include <ranges>
#include <tuple>
#include <flash.hpp>
#include <tl/expected.hpp>

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


constexpr auto DEFAULT_I2C_TIMEOUT_MS = 50;
extern "C" [[noreturn]]
void app_main();

[[noreturn]]
void app_main() {
	using namespace app;
	using namespace tl;
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
		.scl_speed_hz    = 100'000,
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
		err                                 = i2c_master_transmit(dev_handle, w_data.data(), w_data.size(), -1);
		if (err != ESP_OK) {
			on_end();
			return err;
		}
		delay_ms(wait_time_ms);
		err = i2c_master_receive(dev_handle, out.data(), out.size(), -1);
		if (err != ESP_OK) {
			on_end();
			return err;
		}
		on_end();
		return ESP_OK;
	};

	// write register with external buffer (note that the first two bytes of `in` should be the family and index bytes)
	const auto write_command_ext_buf =
		[dev_handle](
			std::span<const uint8_t> in,
			uint16_t wait_time_ms = 2) -> expected<uint8_t, esp_err_t> {
		using ue = unexpected<esp_err_t>;
		esp_err_t err;
		uint8_t status = 0xff;

		gpio_set_level(PIN_MFIO, 0);
		constexpr auto on_end = [] {
			gpio_set_level(PIN_MFIO, 0);
		};
		delay_us(250);

		err = i2c_master_transmit(dev_handle, in.data(), in.size(), DEFAULT_I2C_TIMEOUT_MS);
		if (err != ESP_OK) {
			on_end();
			return ue{err};
		}
		delay_ms(wait_time_ms);
		err = i2c_master_receive(dev_handle, &status, 1, DEFAULT_I2C_TIMEOUT_MS);
		if (err != ESP_OK) {
			on_end();
			return ue{err};
		}
		on_end();
		return status;
	};

	// write register with a stack-allocated buffer of size N
	// TODO: whether put device to sleep
	const auto write_command =
		[write_command_ext_buf]<uint16_t N = 18>(
			uint8_t family_byte, uint8_t index_byte, std::span<const uint8_t> in,
			uint16_t wait_time_ms = 2) -> std::tuple<esp_err_t, uint8_t> {
		uint8_t status            = 0xff;
		constexpr auto MAX_IN_BUF = N - 2;
		if (in.size() > MAX_IN_BUF) {
			return {ESP_ERR_NO_MEM, status};
		}
		std::array<uint8_t, N> w_data = {family_byte, index_byte};
		std::ranges::copy(in, w_data.begin() + 2);
		auto in_buf = std::span(w_data.data(), in.size() + 2);
		return write_command_ext_buf(in_buf, wait_time_ms);
	};

	// write register with a single byte of data
	const auto write_command_byte =
		[write_command_ext_buf](uint8_t family_byte, uint8_t index_byte, uint8_t data,
								uint16_t wait_time_ms = 2) {
			uint8_t in[3] = {family_byte, index_byte, data};
			return write_command_ext_buf(in, wait_time_ms);
		};

	const auto write_bootloader = [=]() {
		constexpr auto OK  = flash::SUCCESS;
		constexpr auto tag = "bl";
		{
			uint8_t out[2]{};
			esp_err_t err = read_command(flash::FMY_DEV_MODE_R, flash::FMY_DEV_MODE_R, out);
			ESP_ERROR_CHECK(err);
			if (out[0] != OK || out[1] != flash::DEV_MODE_R_BL) {
				ESP_LOGE(tag, "query error or not in bootloader mode; out(mode)=(0x%02x, %d)", out[0], out[1]);
				return ESP_FAIL;
			}
		}
		{
			// get ID and MCU type
			uint8_t out[2]{};
			esp_err_t err = read_command(0xff, 0x00, out);
			ESP_ERROR_CHECK(err);
			if (out[0] != OK) {
				ESP_LOGE(tag, "can't get MCU type; out(mcu)=(0x%02x, 0x%02x)", out[0], out[1]);
				return ESP_FAIL;
			} else {
				ESP_LOGI(tag, "mcu type=0x%02x", out[1]);
			}
		}
		{
			const auto num_of_pages = flash::number_of_pages();
			// write number of page
			const uint8_t in[] = {flash::FMY_BL_W, flash::IDX_BL_W_PAGE_NUM, 0x00, num_of_pages};
			auto status        = write_command_ext_buf(in);
			if (!status) {
				ESP_LOGE(tag, "failed to write number of pages; write_command_ext_buf error=%d", status.error());
				return status.error();
			} else if (status.value() != OK) {
				ESP_LOGE(tag, "failed to write number of pages; status=%d", status.value());
				return ESP_FAIL;
			}
			ESP_LOGI(tag, "number of pages %d written", num_of_pages);

			// use heap to avoid stack overflow
			uint8_t *wr_buf_heap  = new uint8_t[2 + flash::MAX_PAGE_SIZE + flash::CHECKBYTES_SIZE];
			const auto wr_buf     = std::span(wr_buf_heap, 2 + flash::MAX_PAGE_SIZE + flash::CHECKBYTES_SIZE);
			const auto on_cleanup = [wr_buf_heap] {
				delete[] wr_buf_heap;
			};

			// write nonce
			wr_buf[0] = flash::FMY_BL_W;
			wr_buf[1] = flash::IDX_BL_W_NONCE;
			std::ranges::copy(flash::init_vector_bytes(), wr_buf.data() + 2);
			status = write_command_ext_buf(std::span(wr_buf.data(), 2 + flash::init_vector_bytes().size()));
			if (!status) {
				ESP_LOGE(tag, "failed to write nonce; write_command_ext_buf error=%d", status.error());
				on_cleanup();
				return status.error();
			} else if (status.value() != OK) {
				ESP_LOGE(tag, "failed to write nonce; status=%d", status.value());
				on_cleanup();
				return ESP_FAIL;
			}
			ESP_LOGI(tag, "nonce written");

			// write auth
			wr_buf[0] = flash::FMY_BL_W;
			wr_buf[1] = flash::IDX_BL_W_AUTH;
			std::ranges::copy(flash::auth_bytes(), wr_buf.data() + 2);
			status = write_command_ext_buf(std::span(wr_buf.data(), 2 + flash::auth_bytes().size()), 10);
			if (!status) {
				ESP_LOGE(tag, "failed to write auth; write_command_ext_buf error=%d", status.error());
				on_cleanup();
				return status.error();
			} else if (status.value() != OK) {
				ESP_LOGE(tag, "failed to write auth; status=%d", status.value());
				on_cleanup();
				return ESP_FAIL;
			}
			ESP_LOGI(tag, "auth written");

			const auto erase = [=]() {
				// erase app
				wr_buf[0]   = flash::FMY_BL_W;
				wr_buf[1]   = flash::IDX_BL_W_ERASE_APP;
				auto status = write_command_ext_buf(std::span(wr_buf.data(), 2), 1'400);
				if (!status) {
					ESP_LOGE(tag, "failed to erase app; write_command_ext_buf error=%d", status.error());
					return status.error();
				} else if (status.value() != OK) {
					ESP_LOGE(tag, "failed to erase app; status=0x%02x", status.value());
					return ESP_FAIL;
				}
				return ESP_OK;
			};
			constexpr auto RETRY_DELAY_MS = 100;
			while (erase() != ESP_OK) {
				delay_ms(RETRY_DELAY_MS);
			}
			ESP_LOGI(tag, "app erased");

			// send pages
			wr_buf[0]       = flash::FMY_BL_W;
			wr_buf[1]       = flash::IDX_BL_W_SEND_PAGE;
			uint32_t offset = flash::APP_START_OFFSET;
			for (uint16_t i = 0; i < num_of_pages; i++) {
				const auto page = flash::msbl().subspan(offset + i * (flash::MAX_PAGE_SIZE + flash::CHECKBYTES_SIZE),
														flash::MAX_PAGE_SIZE + flash::CHECKBYTES_SIZE);
				printf("first 16 bytes of page %d=", i);
				print_as_hex(page.subspan(0, 16));
				printf("last 16 bytes of page %d=", i);
				print_as_hex(page.subspan(page.size() - 16, 16));
				std::ranges::copy(page, wr_buf.data() + 2);
				const auto write_current_page = [=]() {
					auto status = write_command_ext_buf(std::span(wr_buf.data(), 2 + page.size()), 680);
					if (!status) {
						ESP_LOGE(tag, "failed to write page %d; write_command_ext_buf error=%d (%s)", i, status.error(), esp_err_to_name(status.error()));
						return status.error();
					} else if (status.value() != OK) {
						ESP_LOGE(tag, "failed to write page %d; status=0x%02x", i, status.value());
						return ESP_FAIL;
					}
					ESP_LOGI(tag, "page %d written", i);
					return ESP_OK;
				};
				while (write_current_page() != ESP_OK) {
					delay_ms(RETRY_DELAY_MS);
				}
				ESP_LOGI(tag, "page %d written", i);
			}
			on_cleanup();
		}

		ESP_LOGI(tag, "bootloader written");
		auto status = write_command_byte(flash::FMY_DEV_MODE_W, flash::IDX_DEV_MODE_W, flash::DEV_MODE_W_EXIT_BL, 10);
		if (!status) {
			ESP_LOGE(tag, "failed to exit bootloader; write_command_byte error=%d", status.error());
			return status.error();
		} else if (status.value() != OK) {
			ESP_LOGE(tag, "failed to exit bootloader; status=%d", status.value());
			return ESP_FAIL;
		}
		return ESP_OK;
	};


init_retry:
	flash::DevModeR mode = flash::DEV_MODE_R_APP;
	{
		uint8_t out[2];
		auto esp_err = read_command(0x02, 0x00, out);
		if (auto status = out[0]; esp_err != ESP_OK || status != flash::SUCCESS) {
			ESP_LOGW(TAG, "err='%s'(%d), mode=(%d, %d)", esp_err_to_name(esp_err), status, out[0], out[1]);
			delay_ms(100);
			goto init_retry;
		}
		mode = static_cast<flash::DevModeR>(out[1]);
	}

	const auto reset_to_bootloader_by_pin = [] {
		// set RSTN low
		gpio_set_level(PIN_RESET, 0);
		// While RSTN is low, set the MFIO pin to low.
		gpio_set_level(PIN_MFIO, 0);
		// The MFIO pin should be set to low at least 1ms before the RSTN pin is set to high.
		// After the 10ms has elapsed, set the RSTN pin to high.
		delay_ms(10);
		gpio_set_level(PIN_RESET, 1);
		// After an additional 50ms has elapsed, the sensor hub is in Bootloader mode.
		delay_ms(100);
	};

	if (mode == flash::DEV_MODE_R_BL) {
		ESP_LOGW(TAG, "I'm in bootloader mode. try to write application to the chip.");
		const auto msbl = flash::msbl();
		ESP_LOGI(TAG, "msbl.size()=%d", msbl.size());
		printf("auth_bytes(%d)=", flash::auth_bytes().size());
		print_as_hex(flash::auth_bytes());
		printf("init_vector(%d)=", flash::init_vector_bytes().size());
		print_as_hex(flash::init_vector_bytes());
		ESP_LOGI(TAG, "number_of_pages=%d", flash::number_of_pages());
		write_bootloader();
		esp_restart();
	} else {
		esp_err_t esp_err;
		ESP_LOGI(TAG, "app mode");
		uint8_t version[4];
		esp_err = read_command(0xFF, 0x03, version);
		ESP_LOGI(TAG, "app(version)=(0x%02x, 0x%02x, 0x%02x, 0x%02x)", version[0], version[1],
				 version[2], version[3]);
		const auto status = version[0];
		// MAX32664C_OS58_I2C_1PD_WHRM_AEC_SCD_WSPO2_C_32.9.23
		constexpr auto EXPECTED_VERSION_TUPLE = std::array<uint8_t, 3>{32, 9, 23};
		if (esp_err != ESP_OK || status != flash::SUCCESS) {
			ESP_LOGE(TAG, "failed to read version; esp_err=%s(%d), status=%d", esp_err_to_name(esp_err), esp_err, status);
			delay_ms(1'000);
			goto init_retry;
		}
		auto v = std::span(version + 1, 3);
		if (std::equal(v.begin(), v.end(), EXPECTED_VERSION_TUPLE.begin())) {
			ESP_LOGI(TAG, "version=%d.%d.%d", version[0], version[1], version[2]);
		} else {
			ESP_LOGW(TAG, "unexpected version; expected=%d.%d.%d, actual=%d.%d.%d",
					 EXPECTED_VERSION_TUPLE[0], EXPECTED_VERSION_TUPLE[1], EXPECTED_VERSION_TUPLE[2], version[0], version[1], version[2]);
			ESP_LOGI(TAG, "enter bootloader mode and rewrite the application");
			reset_to_bootloader_by_pin();
			goto init_retry;
		}
	}

	while (true) {
		esp_err_t esp_err;
		uint8_t out[2];
		esp_err = read_command(0x02, 0x00, out);
		delay_ms(10);
		if (esp_err != ESP_OK) {
			ESP_LOGW(TAG, "mode=(%d, %d)", out[0], out[1]);
			continue;
		} else {
			mode = static_cast<flash::DevModeR>(out[1]);
		}

		if (mode == flash::DEV_MODE_R_BL) {
			esp_err = read_command(0xFF, 0x00, out);
			ESP_LOGI(TAG, "bl(mcu)=(0x%02x, 0x%02x)", out[0], out[1]);

			uint8_t version[4];
			esp_err = read_command(0x81, 0x00, version);
			ESP_LOGI(TAG, "bl(version)=(0x%02x, 0x%02x, 0x%02x, 0x%02x)", version[0], version[1],
					 version[2], version[3]);

			uint8_t page_size[3];
			esp_err                   = read_command(0x81, 0x01, page_size);
			uint16_t &page_size_be    = *reinterpret_cast<uint16_t *>(page_size + 1);
			uint16_t page_size_native = __ntohs(page_size_be);
			ESP_LOGI(TAG, "bl(page_size)=(0x%02x, 0x%02x, 0x%02x) size=%d", page_size[0],
					 page_size[1], page_size[2], page_size_native);
		} else {
			esp_err = read_command(0xFF, 0x00, out);
			ESP_LOGI(TAG, "app(mcu)=(0x%02x, 0x%02x)", out[0], out[1]);

			uint8_t version[4];
			esp_err = read_command(0xFF, 0x03, version);
			ESP_LOGI(TAG, "app(version)=(0x%02x, 0x%02x, 0x%02x, 0x%02x)", version[0], version[1],
					 version[2], version[3]);

			uint8_t afe_attr[3];
			esp_err = read_command(0x42, 0x00, afe_attr);
			ESP_LOGI(TAG, "app(afe_attr, 86141)=(0x%02x, 0x%02x, 0x%02x)", afe_attr[0], afe_attr[1], afe_attr[2]);

			uint8_t output_mode[2];
			esp_err = read_command(0x11, 0x00, output_mode);
			ESP_LOGI(TAG, "app(output_mode)=(0x%02x, 0x%02x)", output_mode[0], output_mode[1]);

			uint8_t hub_status[2];
			esp_err = read_command(0x00, 0x00, hub_status);
			ESP_LOGI(TAG, "app(hub_status)=(0x%02x, 0x%02x)", hub_status[0], hub_status[1]);

			uint8_t i2c_addr[2];
			esp_err = read_command(0x11, 0x03, i2c_addr);
			ESP_LOGI(TAG, "app(i2c_addr)=(0x%02x, 0x%02x)", i2c_addr[0], i2c_addr[1]);
		}
		delay_ms(900);
	}
}
