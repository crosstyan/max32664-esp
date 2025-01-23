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
#include <ranges>
#include <span>
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

static constexpr auto delay_ms = [](uint32_t ms) {
	vTaskDelay(ms / portTICK_PERIOD_MS);
};

void print_as_hex(std::span<const uint8_t> data) {
	int i = 0;
	for (const auto &byte : data) {
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
		i += 1;
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
		.glitch_ignore_cnt = 14,
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
	i2c_detect();

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address  = 0x55,
		.scl_speed_hz    = 800'000,
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

	const auto read_command =
		[dev_handle](uint8_t family_byte, uint8_t index_byte,
					 std::span<uint8_t> out,
					 uint16_t wait_time_ms = 2) -> esp_err_t {
		esp_err_t err;
		gpio_set_level(GPIO_NUM_2, 0);
		delay_us(250);
		constexpr auto on_end = [] {
			gpio_set_level(GPIO_NUM_2, 1);
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
		gpio_set_level(GPIO_NUM_2, 1);
		on_end();
		return ESP_OK;
	};

	// write register with external buffer (note that the first two bytes of `in` should be the family and index bytes)
	const auto write_command_ext_buf =
		[dev_handle](
			std::span<uint8_t> in,
			uint16_t wait_time_ms = 2) -> std::tuple<esp_err_t, uint8_t> {
		esp_err_t err;
		uint8_t status = 0xff;

		gpio_set_level(GPIO_NUM_2, 0);
		constexpr auto on_end = [] {
			gpio_set_level(GPIO_NUM_2, 1);
		};
		delay_us(250);

		err = i2c_master_transmit(dev_handle, in.data(), in.size(), DEFAULT_I2C_TIMEOUT_MS);
		if (err != ESP_OK) {
			on_end();
			return {err, status};
		}
		delay_ms(wait_time_ms);
		err = i2c_master_receive(dev_handle, &status, 1, DEFAULT_I2C_TIMEOUT_MS);
		if (err != ESP_OK) {
			on_end();
			return {err, status};
		}
		on_end();
		return {ESP_OK, status};
	};

	// write register with a stack-allocated buffer of size N
	const auto write_command =
		[write_command_ext_buf]<uint16_t N = 18>(
			uint8_t family_byte, uint8_t index_byte, std::span<uint8_t> in,
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
		constexpr auto OK  = 0;
		constexpr auto tag = "bl";
		uint8_t out[2]{};
		esp_err_t err = read_command(flash::FMY_DEV_MODE_W, flash::IDX_BL_W_NONCE, out);
		ESP_ERROR_CHECK(err);
		if (out[0] != OK || out[1] != flash::DEV_MODE_R_BL) {
			ESP_LOGE(tag, "query error or not in bootloader mode; out(mode)=(%d, %d)", out[0], out[1]);
			return ESP_FAIL;
		}
		// {
		// 	auto []
		// }

		return ESP_OK;
	};

	const auto msbl = flash::msbl();
	ESP_LOGI(TAG, "msbl.size()=%d", msbl.size());
	printf("auth_bytes(%d)=", flash::auth_bytes().size());
	print_as_hex(flash::auth_bytes());
	printf("init_vector(%d)=", flash::init_vector_bytes().size());
	print_as_hex(flash::init_vector_bytes());
	ESP_LOGI(TAG, "number_of_pages=%d", flash::number_of_pages());

	while (true) {
		esp_err_t esp_err;
		uint8_t out[2];

	retry:
		esp_err = read_command(0x02, 0x00, out);
		delay_ms(10);
		if (esp_err != ESP_OK) {
			ESP_LOGW(TAG, "out(mode)=(%d, %d)", out[0], out[1]);
			goto retry;
		} else {
			ESP_LOGI(TAG, "out(mode)=(%d, %d)", out[0], out[1]);
		}

		esp_err = read_command(0xFF, 0x00, out);
		ESP_LOGI(TAG, "out(mcu)=(%d, %d)", out[0], out[1]);
		delay_ms(10);

		uint8_t version[4];
		esp_err = read_command(0x81, 0x00, version);
		ESP_LOGI(TAG, "out(version)=(%d, %d, %d, %d)", version[0], version[1],
				 version[2], version[3]);
		delay_ms(10);

		uint8_t page_size[3];
		esp_err                   = read_command(0x81, 0x01, page_size);
		uint16_t &page_size_be    = *reinterpret_cast<uint16_t *>(page_size + 1);
		uint16_t page_size_native = __ntohs(page_size_be);
		ESP_LOGI(TAG, "out(page_size)=(%d, %d, %d) size=%d", page_size[0],
				 page_size[1], page_size[2], page_size_native);
		delay_ms(10);
		delay_ms(900);
	}
}
