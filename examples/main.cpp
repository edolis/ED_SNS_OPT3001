
/**
* @file main.cpp
* @brief OPT3001 test with interrupt on GPIO 6
 *
 * @author Emanuele Dolis (emanuele.dolis@gmail.com)
 * @version GIT_VERSION: v1.1.3-4-gf0e7061-dirty
 * @date 2026-05-17
 * @submodules-start
 *   ED_WIFI : v1.0.0-1-g10b3d09
 * @submodules-end
 */


#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "ED_i2c.h"
#include "ED_OPT3001.h"
#include "ed_board.h"          // provides ED_I2C_SDA, ED_I2C_SCL

// I2C configuration
#define I2C_MASTER_FREQ_HZ      400000
#define OPT3001_I2C_ADDR        0x44
#define OPT3001_INT_GPIO        GPIO_NUM_6

// Event group bits
#define BIT_OPT3001_INT         BIT0

static const char *TAG = "opt3001_full_test";
static EventGroupHandle_t evt_group = nullptr;
static ED_OPT3001::OPT3001 *g_sensor = nullptr;

// Forward declarations
static void test_basic_reading(ED_OPT3001::OPT3001 &sensor);
static void test_manual_range(ED_OPT3001::OPT3001 &sensor);
static void test_thresholds_and_faults(ED_OPT3001::OPT3001 &sensor);
static void test_raw_access(ED_OPT3001::OPT3001 &sensor);
static void test_device_id(ED_OPT3001::OPT3001 &sensor);
static void test_single_shot(ED_OPT3001::OPT3001 &sensor);
static void test_mask_exponent(ED_OPT3001::OPT3001 &sensor);
static void interrupt_task(void *arg);

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== OPT3001 Full Feature Test ===");

    // 1. Create I2C bus using board pins
    I2CBus i2cBus(I2C_NUM_0, (gpio_num_t)ED_I2C_SDA, (gpio_num_t)ED_I2C_SCL, I2C_MASTER_FREQ_HZ);

    // 2. Get device handle
    i2c_master_dev_handle_t opt3001_dev = nullptr;
    esp_err_t err = i2cBus.get_device(OPT3001_I2C_ADDR, &opt3001_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device: %s", esp_err_to_name(err));
        return;
    }

    // 3. Create sensor object (with interrupt pin)
    static ED_OPT3001::OPT3001 sensor(opt3001_dev, OPT3001_INT_GPIO);
    g_sensor = &sensor;

    // 4. Run all tests (non‑interrupt part)
    test_device_id(sensor);
    test_basic_reading(sensor);
    test_manual_range(sensor);
    test_thresholds_and_faults(sensor);
    test_raw_access(sensor);
    test_single_shot(sensor);
    test_mask_exponent(sensor);

    // 5. Set up EOC interrupt mode and event group
    ED_OPT3001::OPT3001::ConfigReg cfg{};
    cfg.mode_of_conversion = ED_OPT3001::OPT3001::CONTINUOUS_B;
    cfg.conversion_time    = ED_OPT3001::OPT3001::TIME_800MS;
    cfg.range_number_field = ED_OPT3001::OPT3001::RN_AUTO;
    cfg.polarity_field     = ED_OPT3001::OPT3001::INT_ACTIVE_LOW;
    cfg.latch_field        = ED_OPT3001::OPT3001::LATCH_WINDOW;
    sensor.configure(cfg);
    sensor.enableEOCInterrupt();

    // Clear any stale interrupt
    sensor.getConfig(cfg);

    evt_group = xEventGroupCreate();
    configASSERT(evt_group);
    sensor.enableInterrupt(evt_group, BIT_OPT3001_INT);
    ESP_LOGI(TAG, "Interrupt enabled on GPIO %d, waiting for readings...", OPT3001_INT_GPIO);

    // 6. Create a separate task for interrupt handling
    xTaskCreate(interrupt_task, "int_handler", 4096, &sensor, 5, nullptr);

    // 7. Also run periodic background tests while interrupts occur
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        // Every 10 seconds, print current status flags
        bool overflow, ready, high, low;
        sensor.getOverflowFlag(overflow);
        sensor.isConversionReady(ready);
        sensor.getFlagHigh(high);
        sensor.getFlagLow(low);
        ESP_LOGI(TAG, "Status: OVF=%d, CR=%d, FH=%d, FL=%d", overflow, ready, high, low);
    }
}

// ---------------------------------------------------------------------
// Test: read Manufacturer and Device ID
// ---------------------------------------------------------------------
static void test_device_id(ED_OPT3001::OPT3001 &sensor) {
    uint16_t manuf, dev;
    ESP_ERROR_CHECK(sensor.getManufacturerID(manuf));
    ESP_ERROR_CHECK(sensor.getDeviceID(dev));
    ESP_LOGI(TAG, "Manufacturer ID: 0x%04X (expected 0x5449)", manuf);
    ESP_LOGI(TAG, "Device ID: 0x%04X (expected 0x3001)", dev);
    if (manuf != ED_OPT3001::OPT3001::MANUFACTURER_ID ||
        dev != ED_OPT3001::OPT3001::DEVICE_ID) {
        ESP_LOGW(TAG, "ID mismatch – check sensor");
    }
}

// ---------------------------------------------------------------------
// Test: basic continuous reading (auto‑range, no interrupt)
// ---------------------------------------------------------------------
static void test_basic_reading(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Basic continuous reading (5 samples) ===");
    ED_OPT3001::OPT3001::ConfigReg cfg{};
    cfg.mode_of_conversion = ED_OPT3001::OPT3001::CONTINUOUS_B;
    cfg.conversion_time    = ED_OPT3001::OPT3001::TIME_100MS;
    cfg.range_number_field = ED_OPT3001::OPT3001::RN_AUTO;
    sensor.configure(cfg);

    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(150)); // wait > conversion time
        float lux;
        if (sensor.readLux(lux) == ESP_OK) {
            ESP_LOGI(TAG, "Lux[%d] = %.2f", i, lux);
        } else {
            ESP_LOGE(TAG, "readLux failed");
        }
    }
}

// ---------------------------------------------------------------------
// Test: manual full‑scale range selection
// ---------------------------------------------------------------------
static void test_manual_range(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Manual range test (RN = 0..3) ===");
    for (uint8_t rn = 0; rn <= 3; rn++) {
        sensor.setFullScaleRange(rn);
        vTaskDelay(pdMS_TO_TICKS(200));
        float lux;
        if (sensor.readLux(lux) == ESP_OK) {
            ESP_LOGI(TAG, "RN=%d -> Lux=%.2f lx", rn, lux);
        } else {
            ESP_LOGE(TAG, "readLux failed for RN=%d", rn);
        }
    }
    // Back to auto‑range
    sensor.setAutoRange(true);
}

// ---------------------------------------------------------------------
// Test: thresholds, fault count, and latch mode
// ---------------------------------------------------------------------
static void test_thresholds_and_faults(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Threshold & fault count test ===");
    // Set a window: low=50 lx, high=500 lx
    sensor.setThresholds(50.0f, 500.0f);
    sensor.setFaultCount(ED_OPT3001::OPT3001::FAULT_COUNT_2); // need 2 consecutive faults
    sensor.setLatch(ED_OPT3001::OPT3001::LATCH_WINDOW);

    // Read thresholds back
    float low, high;
    sensor.getThresholds(low, high);
    ESP_LOGI(TAG, "Thresholds: low=%.2f, high=%.2f", low, high);

    // Read current flags (should be 0 initially)
    bool high_flag, low_flag;
    sensor.getFlagHigh(high_flag);
    sensor.getFlagLow(low_flag);
    ESP_LOGI(TAG, "Initial flags: FH=%d, FL=%d", high_flag, low_flag);

    // Wait a few conversions to see if flags change
    vTaskDelay(pdMS_TO_TICKS(2000));
    sensor.getFlagHigh(high_flag);
    sensor.getFlagLow(low_flag);
    ESP_LOGI(TAG, "After 2 sec: FH=%d, FL=%d", high_flag, low_flag);
}

// ---------------------------------------------------------------------
// Test: raw result register access (exponent + mantissa)
// ---------------------------------------------------------------------
static void test_raw_access(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Raw result access ===");
    uint16_t raw;
    sensor.getResultRegister(raw);
    uint8_t exp;
    uint16_t mant;
    sensor.readRawResult(exp, mant);
    float lux = ED_OPT3001::OPT3001::convertRawToLux(raw);   // ✅ public static method
    ESP_LOGI(TAG, "Raw: 0x%04X, Exp=%u, Mant=%u -> Lux=%.2f", raw, exp, mant, lux);
}

// ---------------------------------------------------------------------
// Test: single‑shot mode
// ---------------------------------------------------------------------
static void test_single_shot(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Single‑shot test ===");
    sensor.setMode(ED_OPT3001::OPT3001::SINGLE_SHOT);
    // Wait for conversion ready flag
    bool ready = false;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        sensor.isConversionReady(ready);
        if (ready) break;
    }
    float lux;
    if (sensor.readLux(lux) == ESP_OK) {
        ESP_LOGI(TAG, "Single‑shot lux = %.2f", lux);
    } else {
        ESP_LOGE(TAG, "Single‑shot read failed");
    }
    // Back to continuous for interrupt test
    sensor.setMode(ED_OPT3001::OPT3001::CONTINUOUS_B);
}

// ---------------------------------------------------------------------
// Test: mask exponent feature (force exponent to 0 in manual range)
// ---------------------------------------------------------------------
static void test_mask_exponent(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Mask exponent test ===");
    // Set manual range (e.g., RN=4, 655.20 lux full scale)
    sensor.setFullScaleRange(4); // RN=4 -> FS=655.20 lux
    sensor.setMaskExponent(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    uint16_t raw;
    sensor.getResultRegister(raw);
    uint8_t exp;
    uint16_t mant;
    sensor.readRawResult(exp, mant);
    ESP_LOGI(TAG, "With ME=1: raw=0x%04X, exponent (forced) = %u", raw, exp);
    sensor.setMaskExponent(false);
    sensor.getResultRegister(raw);
    sensor.readRawResult(exp, mant);
    ESP_LOGI(TAG, "With ME=0: raw=0x%04X, exponent = %u", raw, exp);
    // Back to auto range
    sensor.setAutoRange(true);
}

// ---------------------------------------------------------------------
// Interrupt task: waits for INT pin and reads lux
// ---------------------------------------------------------------------
static void interrupt_task(void *arg) {
    ED_OPT3001::OPT3001 *sensor = (ED_OPT3001::OPT3001 *)arg;
    float lux;
    while (1) {
        xEventGroupWaitBits(evt_group, BIT_OPT3001_INT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (sensor->readLux(lux) == ESP_OK) {
            ESP_LOGI(TAG, "[INT] Lux = %.2f lx", lux);
        } else {
            ESP_LOGE(TAG, "[INT] readLux error");
        }
    }
}