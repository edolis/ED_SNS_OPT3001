// #region StdManifest
/**
 * @file ED_OPT3001.h
 * @brief library for the TI OPT3001 luminosity sensor
 *
 * @author Emanuele Dolis (edoliscom@gmail.com)
 * @version 0.2
 * @date 2026-05-17
 */
// #endregion
#pragma once

#include <cstdint>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

namespace ED_OPT3001 {

class OPT3001 {
public:
    // === Constants ===
    static constexpr uint8_t RESULT_REG          = 0x00;
    static constexpr uint8_t CONFIG_REG          = 0x01;
    static constexpr uint8_t LOW_LIMIT_REG       = 0x02;
    static constexpr uint8_t HIGH_LIMIT_REG      = 0x03;
    static constexpr uint8_t MANUFACTURER_ID_REG = 0x7E;
    static constexpr uint8_t DEVICE_ID_REG       = 0x7F;

    static constexpr uint16_t MANUFACTURER_ID = 0x5449;
    static constexpr uint16_t DEVICE_ID       = 0x3001;

    // Ranging mode (always automatic)
    static constexpr uint16_t RN_AUTO = 0x0C;

    // Conversion modes
    enum ConversionMode : uint8_t {
        LOWPOWER     = 0x00,
        SINGLE_SHOT  = 0x01,
        CONTINUOUS_A = 0x02,
        CONTINUOUS_B = 0x03
    };

    // Conversion times
    enum ConversionTime : uint8_t {
        TIME_100MS = 0x00,
        TIME_800MS = 0x01
    };

    // Interrupt polarity
    enum Polarity : uint8_t {
        INT_ACTIVE_LOW  = 0x00,
        INT_ACTIVE_HIGH = 0x01
    };

    // Latch styles
    enum Latch : uint8_t {
        LATCH_TRANSPARENT = 0x00,   // auto‑clearing when light returns within limits
        LATCH_WINDOW      = 0x01    // stays asserted until config or result register is read
    };

    // === Config register bitfield ===
    union ConfigReg {
        uint16_t config_register;
        struct {
            unsigned fault_count_field   : 2;
            unsigned mask_exponent_field : 1;
            unsigned polarity_field      : 1;
            unsigned latch_field         : 1;
            unsigned low_field_flag      : 1;
            unsigned high_field_flag     : 1;
            unsigned conversion_ready    : 1;
            unsigned overflow_flag       : 1;
            unsigned mode_of_conversion  : 2;
            unsigned conversion_time     : 1;
            unsigned range_number_field  : 4;
        };
    };

    // === Construction & device setup ===
    /**
     * @brief Construct a new OPT3001 object
     * @param dev I2C master device handle (obtained from I2CBus::get_device)
     * @param intPin GPIO number for interrupt pin (optional, set to GPIO_NUM_NC if not used)
     */
    OPT3001(i2c_master_dev_handle_t dev, gpio_num_t intPin = GPIO_NUM_NC);

    // No static addDevice() – use I2CBus::get_device instead.

    // === Basic operations ===
    esp_err_t configure(const ConfigReg& cfg) const;
    esp_err_t getConfig(ConfigReg& cfg) const;
    esp_err_t readLux(float& lux) const;   // returns ESP_OK and fills lux
    esp_err_t readRaw(uint8_t reg, uint16_t& out) const;

    // === Threshold & interrupt control ===
    esp_err_t setThresholds(float lowLux, float highLux) const;   // normal window mode
    esp_err_t getThresholds(float& lowLux, float& highLux) const;
    esp_err_t enableEOCInterrupt() const;                         // End‑Of‑Conversion mode
    esp_err_t disableEOCInterrupt(float restoreLowLux) const;     // back to normal thresholds

    // === Configuration helpers ===
    esp_err_t setMode(uint8_t conversion_mode) const;
    esp_err_t setConversionTime(uint8_t conv_time) const;
    esp_err_t setPolarity(uint8_t polarity) const;
    esp_err_t setLatch(uint8_t latch) const;

    // === Device identification ===
    esp_err_t getManufacturerID(uint16_t& id) const;
    esp_err_t getDeviceID(uint16_t& id) const;

    // === Interrupt handling (GPIO) ===
    /**
     * @brief Enable interrupt on INT pin (falling edge only)
     * @param evtGroup FreeRTOS event group to signal
     * @param bitMask  Bit mask to set when interrupt occurs
     */
    esp_err_t enableInterrupt(EventGroupHandle_t evtGroup, EventBits_t bitMask);

    /**
     * @brief Disable interrupt (remove ISR handler)
     */
    esp_err_t disableInterrupt();

private:
    i2c_master_dev_handle_t dev_;
    gpio_num_t int_gpio_;

    // Interrupt runtime data
    EventGroupHandle_t evt_group_ = nullptr;
    EventBits_t        evt_bit_   = 0;
    bool               isr_enabled_ = false;

    // Low‑level register access
    esp_err_t writeRegister(uint8_t reg, uint16_t value) const;
    esp_err_t readRegister(uint8_t reg, uint16_t& value) const;

    // Lux ↔ register conversion
    static uint8_t  lsbFromLux(float lux);
    static uint16_t floatToRegister(float lux);
    static float    registerToFloat(uint16_t reg);

    // ISR trampoline
    static void IRAM_ATTR isr_trampoline(void* arg);

    static constexpr TickType_t TIMEOUT_MS = 1000;  // 1 second timeout for I2C
};

} // namespace ED_OPT3001