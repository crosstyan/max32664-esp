#include <algorithm>
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
} // namespace app

extern "C" void app_main();
void app_main() {
	using namespace app;
	constexpr auto TAG = "main";
	delay_ms(200);

	i2c_master_bus_config_t i2c_mst_config = {
		.i2c_port   = I2C_NUM_0,
		.sda_io_num = GPIO_NUM_10,
		.scl_io_num = GPIO_NUM_11,
		.clk_source = I2C_CLK_SRC_DEFAULT,

		.glitch_ignore_cnt = 7,
		.flags             = {.enable_internal_pullup = true},
	};
	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

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

	const auto read_register =
		[dev_handle](uint8_t family_byte, uint8_t index_byte,
					 std::span<uint8_t> out,
					 uint16_t wait_time_ms = 2) -> esp_err_t {
		esp_err_t err;
		gpio_set_level(GPIO_NUM_2, 0);
		delay_us(250);
		const std::array<uint8_t, 2> w_data = {family_byte, index_byte};
		err                                 = i2c_master_transmit(dev_handle, w_data.data(), w_data.size(), -1);
		if (err != ESP_OK) {
			gpio_set_level(GPIO_NUM_2, 1);
			return err;
		}
		delay_ms(wait_time_ms);
		err = i2c_master_receive(dev_handle, out.data(), out.size(), -1);
		if (err != ESP_OK) {
			gpio_set_level(GPIO_NUM_2, 1);
			return err;
		}
		gpio_set_level(GPIO_NUM_2, 1);
		return ESP_OK;
	};

	const auto write_register =
		[dev_handle](
			uint8_t family_byte, uint8_t index_byte, std::span<uint8_t> in,
			uint16_t wait_time_ms = 2) -> std::tuple<esp_err_t, uint8_t> {
		esp_err_t err;
		uint8_t status = 0xff;
		gpio_set_level(GPIO_NUM_2, 0);
		delay_us(250);
		constexpr auto MAX_IN_ARRAY_SIZE = 32;
		constexpr auto MAX_IN_BUF        = MAX_IN_ARRAY_SIZE - 2;
		if (in.size() > MAX_IN_BUF) {
			return {ESP_ERR_NO_MEM, status};
		}
		std::array<uint8_t, MAX_IN_ARRAY_SIZE> w_data = {family_byte, index_byte};
		std::ranges::copy(in, w_data.begin() + 2);
		auto in_buf = std::span(w_data.data(), in.size() + 2);
		err         = i2c_master_transmit(dev_handle, in_buf.data(), in_buf.size(), -1);
		if (err != ESP_OK) {
			gpio_set_level(GPIO_NUM_2, 1);
			return {err, status};
		}
		delay_ms(wait_time_ms);
		err = i2c_master_receive(dev_handle, &status, 1, -1);
		if (err != ESP_OK) {
			gpio_set_level(GPIO_NUM_2, 1);
			return {err, status};
		}
		gpio_set_level(GPIO_NUM_2, 1);
		return {ESP_OK, status};
	};

	const auto write_register_byte =
		[write_register](uint8_t family_byte, uint8_t index_byte, uint8_t data,
						 uint16_t wait_time_ms = 2) {
			uint8_t in[] = {data};
			return write_register(family_byte, index_byte, in, wait_time_ms);
		};

	const auto read_info = [read_register] {
		uint8_t out[2]{0};
		auto err = read_register(0x02, 0x00, out);
		ESP_ERROR_CHECK(err);
		if (auto status = out[0]; status == 0) {
			ESP_LOGI(TAG, "value=%d", out[1]);
		} else {
			ESP_LOGE(TAG, "status=%d", status);
			ESP_LOGW(TAG, "value=%d", out[1]);
		}
	};

	const auto enter_bootloader = [dev_handle] {
		esp_err_t err;
		gpio_set_level(GPIO_NUM_2, 0);
		delay_us(250);
		const std::array<uint8_t, 3> w_data = {0x01, 0x00, 0x08};
		err                                 = i2c_master_transmit(dev_handle, w_data.data(), w_data.size(), -1);
		if (err != ESP_OK) {
			return err;
		}
		delay_ms(2);
		std::array<uint8_t, 1> out{0};
		err = i2c_master_receive(dev_handle, out.data(), out.size(), -1);
		if (err != ESP_OK) {
			return err;
		}
		gpio_set_level(GPIO_NUM_2, 1);
		return ESP_OK;
	};

	ESP_ERROR_CHECK(i2c_master_probe(bus_handle, 0x55, -1));

	while (true) {
		esp_err_t esp_err;
		uint8_t out[2];
		esp_err = read_register(0x02, 0x00, out);
		ESP_LOGI(TAG, "out(mode)=(%d, %d)", out[0], out[1]);
		delay_ms(10);

		esp_err = read_register(0xFF, 0x00, out);
		ESP_LOGI(TAG, "out(mcu)=(%d, %d)", out[0], out[1]);
		delay_ms(10);

		uint8_t version[4];
		esp_err = read_register(0x81, 0x00, version);
		ESP_LOGI(TAG, "out(version)=(%d, %d, %d, %d)", version[0], version[1],
				 version[2], version[3]);
		delay_ms(10);

		uint8_t page_size[3];
		esp_err                   = read_register(0x81, 0x01, page_size);
		uint16_t &page_size_be    = *reinterpret_cast<uint16_t *>(page_size + 1);
		uint16_t page_size_native = __ntohs(page_size_be);
		ESP_LOGI(TAG, "out(page_size)=(%d, %d, %d) size=%d", page_size[0],
				 page_size[1], page_size[2], page_size_native);
		delay_ms(10);
		delay_ms(900);
	}
}
