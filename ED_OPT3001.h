#pragma once

#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "ED_i2c.h"

namespace ED_OPT3001 {

class OPT3001 {
public:
    static constexpr uint8_t RESULT_REG          = 0x00;
    static constexpr uint8_t CONFIG_REG          = 0x01;
    static constexpr uint8_t LOW_LIMIT_REG       = 0x02;
    static constexpr uint8_t HIGH_LIMIT_REG      = 0x03;
    static constexpr uint8_t MANUFACTURER_ID_REG = 0x7E;
    static constexpr uint8_t DEVICE_ID_REG       = 0x7F;

    static constexpr uint16_t MANUFACTURER_ID = 0x5449;
    static constexpr uint16_t DEVICE_ID       = 0x3001;

    static constexpr uint8_t RN_MANUAL_MIN = 0x00;
    static constexpr uint8_t RN_MANUAL_MAX = 0x0B;
    static constexpr uint8_t RN_AUTO       = 0x0C;

    enum ConversionMode : uint8_t {
        LOWPOWER     = 0x00,
        SINGLE_SHOT  = 0x01,
        CONTINUOUS_A = 0x02,
        CONTINUOUS_B = 0x03
    };
    enum ConversionTime : uint8_t { TIME_100MS = 0x00, TIME_800MS = 0x01 };
    enum Polarity : uint8_t { INT_ACTIVE_LOW = 0x00, INT_ACTIVE_HIGH = 0x01 };
    enum Latch : uint8_t { LATCH_TRANSPARENT = 0x00, LATCH_WINDOW = 0x01 };
    enum FaultCount : uint8_t {
        FAULT_COUNT_1 = 0x00,
        FAULT_COUNT_2 = 0x01,
        FAULT_COUNT_4 = 0x02,
        FAULT_COUNT_8 = 0x03
    };

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

    OPT3001(I2CBus& bus, uint8_t dev_addr, gpio_num_t intPin = GPIO_NUM_NC);

    static float convertRawToLux(uint16_t rawRegister);
    esp_err_t configure(const ConfigReg& cfg) const;
    esp_err_t getConfig(ConfigReg& cfg) const;
    esp_err_t readLux(float& lux) const;
    esp_err_t readRawResult(uint8_t& exponent, uint16_t& mantissa) const;
    esp_err_t getResultRegister(uint16_t& raw) const;

    esp_err_t setThresholds(float lowLux, float highLux) const;
    esp_err_t getThresholds(float& lowLux, float& highLux) const;
    esp_err_t enableEOCInterrupt() const;
    esp_err_t disableEOCInterrupt(float restoreLowLux) const;

    esp_err_t setMode(uint8_t conversion_mode) const;
    esp_err_t setConversionTime(uint8_t conv_time) const;
    esp_err_t setPolarity(uint8_t polarity) const;
    esp_err_t setLatch(uint8_t latch) const;
    esp_err_t setFaultCount(uint8_t count) const;
    esp_err_t setMaskExponent(bool enable) const;
    esp_err_t setFullScaleRange(uint8_t range) const;
    esp_err_t setAutoRange(bool enable) const;

    esp_err_t getOverflowFlag(bool& overflow) const;
    esp_err_t isConversionReady(bool& ready) const;
    esp_err_t getFlagHigh(bool& high) const;
    esp_err_t getFlagLow(bool& low) const;

    esp_err_t getManufacturerID(uint16_t& id) const;
    esp_err_t getDeviceID(uint16_t& id) const;

    esp_err_t enableInterrupt(EventGroupHandle_t evtGroup, EventBits_t bitMask);
    esp_err_t disableInterrupt();

private:
    I2CBus& m_bus;
    uint8_t m_addr;
    gpio_num_t int_gpio_;
    EventGroupHandle_t evt_group_ = nullptr;
    EventBits_t evt_bit_ = 0;
    bool isr_enabled_ = false;

    esp_err_t writeRegister(uint8_t reg, uint16_t value) const;
    esp_err_t readRegister(uint8_t reg, uint16_t& value) const;

    static uint8_t  lsbFromLux(float lux);
    static uint16_t floatToRegister(float lux);
    static float    registerToFloat(uint16_t reg);

    gpio_int_type_t getInterruptEdge() const;
    static void IRAM_ATTR isr_trampoline(void* arg);
};

} // namespace ED_OPT3001