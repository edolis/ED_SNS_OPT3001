/**
* @file main.cpp
* @brief ADS1115 test using ED_ADS1115 library
 *
 * @author Emanuele Dolis (emanuele.dolis@gmail.com)
 * @version GIT_VERSION: v1.1.3-4-gf0e7061-dirty
 * @date 2026-05-17
 * @submodules-start
 *   ED_WIFI : v1.0.0-1-g10b3d09
 * @submodules-end
 */
/**
 * @file main.cpp
 * @brief OPT3001 test with interrupt on GPIO 6
 *
 * Uses the refactored ED_OPT3001 driver and I2CBus wrapper for ESP‑IDF 5.5.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "ED_i2c.h"
#include "ED_OPT3001.h"

#define CONFIG_IDF_TARGET_ESP32C6

#include "ed_board.h"

// I2C configuration (adjust to your board)
#define I2C_MASTER_SCL_IO   GPIO_NUM_8
#define I2C_MASTER_SDA_IO   GPIO_NUM_9
#define I2C_MASTER_FREQ_HZ  400000
#define OPT3001_I2C_ADDR    0x44
#define OPT3001_INT_GPIO    GPIO_NUM_6

// Event group bits
#define BIT_OPT3001_INT     BIT0

static const char *TAG = "opt3001_int_test";
static EventGroupHandle_t evt_group = nullptr;

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting OPT3001 test with interrupt on GPIO %d", OPT3001_INT_GPIO);

    // 1. Create I2C bus
    I2CBus i2cBus(I2C_NUM_0, ED_I2C_SDA, ED_I2C_SCL, I2C_MASTER_FREQ_HZ);

    // 2. Get device handle for OPT3001
    i2c_master_dev_handle_t opt3001_dev = nullptr;
    esp_err_t err = i2cBus.get_device(OPT3001_I2C_ADDR, &opt3001_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device: %s", esp_err_to_name(err));
        return;
    }

    // 3. Create OPT3001 object with interrupt pin
    ED_OPT3001::OPT3001 sensor(opt3001_dev, OPT3001_INT_GPIO);

    // 4. Configure the sensor
    ED_OPT3001::OPT3001::ConfigReg cfg{};
    cfg.mode_of_conversion = ED_OPT3001::OPT3001::CONTINUOUS_B;   // continuous mode
    cfg.conversion_time    = ED_OPT3001::OPT3001::TIME_800MS;     // 800 ms integration
    cfg.range_number_field = ED_OPT3001::OPT3001::RN_AUTO;        // auto range
    cfg.polarity_field     = ED_OPT3001::OPT3001::INT_ACTIVE_LOW; // active low
    cfg.latch_field        = ED_OPT3001::OPT3001::LATCH_WINDOW;   // latch until read

    err = sensor.configure(cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Configuration failed: %s", esp_err_to_name(err));
        return;
    }

    // 5. Enable End‑Of‑Conversion interrupt mode
    err = sensor.enableEOCInterrupt();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "enableEOCInterrupt failed: %s", esp_err_to_name(err));
        return;
    }

    // Clear any stale interrupt by reading the config register
    sensor.getConfig(cfg);

    // 6. Create event group and enable GPIO interrupt
    evt_group = xEventGroupCreate();
    configASSERT(evt_group);

    err = sensor.enableInterrupt(evt_group, BIT_OPT3001_INT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "enableInterrupt failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Interrupt enabled on GPIO %d", OPT3001_INT_GPIO);

    // 7. Main loop: wait for interrupt and read lux
    float lux = 0.0f;
    while (1) {
        // Wait for the interrupt bit (cleared automatically after wait)
        EventBits_t bits = xEventGroupWaitBits(evt_group, BIT_OPT3001_INT,
                                               pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & BIT_OPT3001_INT) {
            // Optionally wait for conversion_ready flag (optional)
            ED_OPT3001::OPT3001::ConfigReg status;
            sensor.getConfig(status);
            if (status.conversion_ready) {
                err = sensor.readLux(lux);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Lux: %.2f lx", lux);
                } else {
                    ESP_LOGE(TAG, "readLux error: %s", esp_err_to_name(err));
                }
            } else {
                ESP_LOGW(TAG, "Interrupt but conversion_ready not set");
            }
        }
    }
}