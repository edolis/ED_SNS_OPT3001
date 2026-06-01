/**
* @file main.cpp
* @brief OPT3001 full test suite with working interrupt (config read clear)
 *
 * @author Emanuele Dolis (emanuele.dolis@gmail.com)
 * @version GIT_VERSION: v1.1.3-4-gf0e7061-dirty
 * @date 2026-06-01
 * @submodules-start
 *   ED_WIFI : v1.1.0-0-ga015030
 * @submodules-end
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "ED_i2c.h"
#include "ED_OPT3001.h"

#define CONFIG_IDF_TARGET_ESP32C6
#include "ed_board.h"

#define I2C_MASTER_FREQ_HZ      400000
#define OPT3001_I2C_ADDR        0x44
#define OPT3001_INT_GPIO        GPIO_NUM_6
#define BIT_OPT3001_INT         BIT0

static const char *TAG = "opt3001_full_test";
static EventGroupHandle_t evt_group = nullptr;

static void interrupt_task(void *arg);
static void test_basic_reading(ED_OPT3001::OPT3001 &sensor);
static void test_manual_range(ED_OPT3001::OPT3001 &sensor);
static void test_thresholds_and_faults(ED_OPT3001::OPT3001 &sensor);
static void test_raw_access(ED_OPT3001::OPT3001 &sensor);
static void test_device_id(ED_OPT3001::OPT3001 &sensor);
static void test_single_shot(ED_OPT3001::OPT3001 &sensor);
static void test_mask_exponent(ED_OPT3001::OPT3001 &sensor);

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== OPT3001 Full Feature Test ===");

    esp_log_level_set("ed_i2c", ESP_LOG_WARN);
    esp_log_level_set("ED_OPT3001", ESP_LOG_WARN);

    I2CBus i2cBus(I2C_NUM_0, (gpio_num_t)ED_I2C_SDA, (gpio_num_t)ED_I2C_SCL, I2C_MASTER_FREQ_HZ);
    static ED_OPT3001::OPT3001 sensor(i2cBus, OPT3001_I2C_ADDR, OPT3001_INT_GPIO);

    test_device_id(sensor);
    test_basic_reading(sensor);
    test_manual_range(sensor);
    test_thresholds_and_faults(sensor);
    test_raw_access(sensor);
    test_single_shot(sensor);
    test_mask_exponent(sensor);

    // Configure continuous, 100ms, auto‑range, transparent latch, active low
    ED_OPT3001::OPT3001::ConfigReg cfg{};
    cfg.mode_of_conversion = ED_OPT3001::OPT3001::CONTINUOUS_B;
    cfg.conversion_time    = ED_OPT3001::OPT3001::TIME_100MS;
    cfg.range_number_field = ED_OPT3001::OPT3001::RN_AUTO;
    cfg.polarity_field     = ED_OPT3001::OPT3001::INT_ACTIVE_LOW;
    cfg.latch_field        = ED_OPT3001::OPT3001::LATCH_TRANSPARENT;
    sensor.configure(cfg);
    sensor.enableEOCInterrupt();

    evt_group = xEventGroupCreate();
    configASSERT(evt_group);
    sensor.enableInterrupt(evt_group, BIT_OPT3001_INT);
    ESP_LOGI(TAG, "Interrupt enabled on GPIO %d", OPT3001_INT_GPIO);

    xTaskCreate(interrupt_task, "int_handler", 8192, &sensor, 5, nullptr);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------------------------------------------------------------------
// Test functions (unchanged, only public methods)
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

static void test_basic_reading(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Basic continuous reading (5 samples) ===");
    ED_OPT3001::OPT3001::ConfigReg cfg{};
    cfg.mode_of_conversion = ED_OPT3001::OPT3001::CONTINUOUS_B;
    cfg.conversion_time    = ED_OPT3001::OPT3001::TIME_100MS;
    cfg.range_number_field = ED_OPT3001::OPT3001::RN_AUTO;
    sensor.configure(cfg);

    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(150));
        float lux;
        if (sensor.readLux(lux) == ESP_OK) {
            ESP_LOGI(TAG, "Lux[%d] = %.2f", i, lux);
        } else {
            ESP_LOGE(TAG, "readLux failed");
        }
    }
}

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
    sensor.setAutoRange(true);
}

static void test_thresholds_and_faults(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Threshold & fault count test ===");
    sensor.setThresholds(50.0f, 500.0f);
    sensor.setFaultCount(ED_OPT3001::OPT3001::FAULT_COUNT_2);
    sensor.setLatch(ED_OPT3001::OPT3001::LATCH_WINDOW);

    float low, high;
    sensor.getThresholds(low, high);
    ESP_LOGI(TAG, "Thresholds: low=%.2f, high=%.2f", low, high);

    bool high_flag, low_flag;
    sensor.getFlagHigh(high_flag);
    sensor.getFlagLow(low_flag);
    ESP_LOGI(TAG, "Initial flags: FH=%d, FL=%d", high_flag, low_flag);

    vTaskDelay(pdMS_TO_TICKS(2000));
    sensor.getFlagHigh(high_flag);
    sensor.getFlagLow(low_flag);
    ESP_LOGI(TAG, "After 2 sec: FH=%d, FL=%d", high_flag, low_flag);
}

static void test_raw_access(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Raw result access ===");
    uint16_t raw;
    sensor.getResultRegister(raw);
    uint8_t exp;
    uint16_t mant;
    sensor.readRawResult(exp, mant);
    float lux = ED_OPT3001::OPT3001::convertRawToLux(raw);
    ESP_LOGI(TAG, "Raw: 0x%04X, Exp=%u, Mant=%u -> Lux=%.2f", raw, exp, mant, lux);
}

static void test_single_shot(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Single‑shot test ===");
    sensor.setMode(ED_OPT3001::OPT3001::SINGLE_SHOT);
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
    sensor.setMode(ED_OPT3001::OPT3001::CONTINUOUS_B);
}

static void test_mask_exponent(ED_OPT3001::OPT3001 &sensor) {
    ESP_LOGI(TAG, "=== Mask exponent test ===");
    sensor.setFullScaleRange(4);
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
    sensor.setAutoRange(true);
}

// ---------------------------------------------------------------------
// Interrupt task – explicit config read to clear flag
// ---------------------------------------------------------------------
static void interrupt_task(void *arg) {
    ED_OPT3001::OPT3001 *sensor = (ED_OPT3001::OPT3001 *)arg;
    float lux;
    int count = 0;
    ED_OPT3001::OPT3001::ConfigReg dummy;   // used to clear interrupt

    while (1) {
        xEventGroupWaitBits(evt_group, BIT_OPT3001_INT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (sensor->readLux(lux) != ESP_OK) continue;
        count++;
        if (count >= 10) {
            count = 0;
            bool overflow, ready, high, low;
            sensor->getOverflowFlag(overflow);
            sensor->isConversionReady(ready);
            sensor->getFlagHigh(high);
            sensor->getFlagLow(low);
            ESP_LOGI(TAG, "Lux = %8.2f lx   | OVF=%d, CR=%d, FH=%d, FL=%d",
                     lux, overflow, ready, high, low);
        }
    }
}