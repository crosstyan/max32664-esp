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

void IRAM_ATTR delay_us(uint32_t us) {
  constexpr auto micros = [] { return esp_timer_get_time(); };
  constexpr auto nop = [] { __asm__ __volatile__("nop"); };
  auto m = micros();
  if (us) {
    auto e = (m + us);
    [[unlikely]] if (m > e) { // overflow
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

extern "C" void app_main();
void app_main() {
  constexpr auto TAG = "main";
  // Max32664 max;
  vTaskDelay(200 / portTICK_PERIOD_MS);
  // Wire.begin( I2C_SDA_PIN, I2C_SCL_PIN,I2C_FREQ_HZ);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  // Check that communication was successful, not that the IC is outputting
  // correct format.
  // max.sensor_init(MODE_ONE);

  i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = GPIO_NUM_10,
      .scl_io_num = GPIO_NUM_11,
      .clk_source = I2C_CLK_SRC_DEFAULT,

      .glitch_ignore_cnt = 7,
      .flags = {.enable_internal_pullup = true},
  };
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = 0x55,
      .scl_speed_hz = 400'000,
  };
  i2c_master_dev_handle_t dev_handle;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

  //
  // static constexpr auto i2c_init = [] {
  //
  //   constexpr auto I2C_MASTER_CHAN_NUM = I2C_NUM_0;
  //   // 400kHz fast mode
  //   // See 9.5.1.3 @ADS1115 datasheet
  //   constexpr auto I2C_MASTER_FREQ_HZ1        = 400'000;
  //   constexpr auto I2C_MASTER_RX_BUF_DISABLE = 0;
  //   constexpr auto I2C_MASTER_TX_BUF_DISABLE = 0;
  //       constexpr auto master_conf = i2c_config_t{
  //         .mode          = I2C_MODE_MASTER,
  //         .sda_io_num    = I2C_MASTER_SDA,
  //         .scl_io_num    = I2C_MASTER_SCL,
  //         .sda_pullup_en = GPIO_PULLUP_ENABLE,
  //         .scl_pullup_en = GPIO_PULLUP_ENABLE,
  //         .master        = {
  //           .clk_speed = I2C_MASTER_FREQ_HZ1,
  //          },
  //  };
  //   auto err = i2c_param_config(I2C_MASTER_CHAN_NUM, &master_conf);
  //   if (err != ESP_OK) {
  //     return err;
  //   }
  //   err = i2c_driver_install(I2C_MASTER_CHAN_NUM, master_conf.mode,
  //   I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0); if (err !=
  //   ESP_OK) {
  //     return err;
  //   }
  //   return err;
  // };
  // auto i2c_ = i2c_init();
  // if (not i2c_) {
  //   ESP_LOGE("main", "Failed to initialize I2C: %s", esp_err_to_name(i2c_));
  // }
  // i2c_port_t i2c_num    = I2C_NUM_0;
  // const auto i2c_detect = [i2c_num]() {
  //   constexpr auto WRITE_BIT     = 0;
  //   constexpr auto READ_BIT      = 1;
  //   constexpr auto ACK_CHECK_EN  = 0x1;
  //   constexpr auto ACK_CHECK_DIS = 0x0;
  //   constexpr auto ACK_VAL       = 0x0;
  //   constexpr auto NACK_VAL      = 0x1;
  //   uint8_t address;
  //   printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
  //   for (int i = 0; i < 128; i += 16) {
  //     printf("%02x: ", i);
  //     for (int j = 0; j < 16; j++) {
  //       fflush(stdout);
  //       address              = i + j;
  //       i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //       i2c_master_start(cmd);
  //       i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
  //       i2c_master_stop(cmd);
  //       auto ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(50));
  //       i2c_cmd_link_delete(cmd);
  //       if (ret == ESP_OK) {
  //         printf("%02x ", address);
  //       } else if (ret == ESP_ERR_TIMEOUT) {
  //         printf("UU ");
  //       } else {
  //         printf("-- ");
  //       }
  //     }
  //     printf("\r\n");
  //   }
  // };
  //   const auto i2c_general_call = [i2c_num] {
  //     uint8_t buf[1] = {0x06};
  //     return i2c_master_write_to_device(i2c_num, 0, buf, 1,
  //     I2C_MASTER_TIMEOUT_MS);
  // };
  // i2c_detect();
  // i2c_general_call();
  constexpr auto PIN_RESET = GPIO_NUM_22;
  constexpr auto PIN_MFIO = GPIO_NUM_2;

  // zero-initialize the config structure.
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = PIN_RESET | PIN_MFIO;
  // disable pull-down mode
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  // disable pull-up mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  // configure GPIO with the given settings
  gpio_config(&io_conf);

  static constexpr auto delay_ms = [](uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
  };

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
    err = i2c_master_transmit(dev_handle, w_data.data(), w_data.size(), -1);
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
    constexpr auto MAX_IN_BUF = MAX_IN_ARRAY_SIZE - 2;
    if (in.size() > MAX_IN_BUF) {
      return {ESP_ERR_NO_MEM, status};
    }
    std::array<uint8_t, MAX_IN_ARRAY_SIZE> w_data = {family_byte, index_byte};
    std::ranges::copy(in, w_data.begin() + 2);
    auto in_buf = std::span(w_data.data(), in.size() + 2);
    err = i2c_master_transmit(dev_handle, in_buf.data(), in_buf.size(), -1);
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
    err = i2c_master_transmit(dev_handle, w_data.data(), w_data.size(), -1);
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
    esp_err = read_register(0x81, 0x01, page_size);
    uint16_t &page_size_be = *reinterpret_cast<uint16_t *>(page_size + 1);
    uint16_t page_size_native = __ntohs(page_size_be);
    ESP_LOGI(TAG, "out(page_size)=(%d, %d, %d) size=%d", page_size[0],
             page_size[1], page_size[2], page_size_native);
    delay_ms(10);

    // auto [tmp_esp_err, status] = write_register_byte(0x01, 0x00, 0x08, 5);
    // esp_err = tmp_esp_err;
    // ESP_LOGI(TAG, "out(w1)=(%d, %d)", esp_err, status);
    delay_ms(900);
  }
}
