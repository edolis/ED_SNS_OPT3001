#include "ED_OPT3001.h"
#include <cmath>
#include "freertos/portmacro.h"
#include "esp_check.h"

namespace ED_OPT3001 {

static const char* TAG = "ED_OPT3001";

// ---------------------------------------------------------------------
// Construction & destruction
// ---------------------------------------------------------------------
OPT3001::OPT3001(i2c_master_dev_handle_t dev, gpio_num_t intPin)
    : dev_(dev), int_gpio_(intPin)
{}

// ---------------------------------------------------------------------
// Register access
// ---------------------------------------------------------------------
esp_err_t OPT3001::writeRegister(uint8_t reg, uint16_t value) const {
    uint8_t wr[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_transmit(dev_, wr, sizeof(wr), pdMS_TO_TICKS(TIMEOUT_MS));
}

esp_err_t OPT3001::readRegister(uint8_t reg, uint16_t& value) const {
    uint8_t rx[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(dev_, &reg, 1, rx, 2, pdMS_TO_TICKS(TIMEOUT_MS));
    if (err == ESP_OK) {
        value = ((uint16_t)rx[0] << 8) | rx[1];
    }
    return err;
}

// ---------------------------------------------------------------------
// Lux / register conversion
// ---------------------------------------------------------------------
float OPT3001::convertRawToLux(uint16_t rawRegister) {
    return registerToFloat(rawRegister);  // reuse private helper
}

uint8_t OPT3001::lsbFromLux(float lux) {
    uint8_t exp = 0;
    while (lux > (40.95f * (1 << exp)) && exp < 0x0B) exp++;
    return exp;
}

uint16_t OPT3001::floatToRegister(float lux) {
    if (lux < 0.01f) lux = 0.01f;
    if (lux > 83865.60f) lux = 83865.60f;
    uint8_t expVal = lsbFromLux(lux);
    float lsb = 0.01f * powf(2.0f, (float)expVal);
    uint16_t mant = (uint16_t)(lux / lsb);
    return ((uint16_t)expVal << 12) | (mant & 0x0FFF);
}

float OPT3001::registerToFloat(uint16_t reg) {
    uint8_t exp   = (reg >> 12) & 0x0F;
    uint16_t mant = reg & 0x0FFF;
    float lsb = 0.01f * powf(2.0f, (float)exp);
    return lsb * mant;
}

// ---------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------
esp_err_t OPT3001::configure(const ConfigReg& cfg) const {
    return writeRegister(CONFIG_REG, cfg.config_register);
}

esp_err_t OPT3001::getConfig(ConfigReg& cfg) const {
    uint16_t raw;
    esp_err_t err = readRegister(CONFIG_REG, raw);
    if (err == ESP_OK) cfg.config_register = raw;
    return err;
}

// ---------------------------------------------------------------------
// Thresholds (normal window mode)
// ---------------------------------------------------------------------
esp_err_t OPT3001::setThresholds(float lowLux, float highLux) const {
    if (highLux > 83865.60f) highLux = 83865.60f;
    if (lowLux  < 0.01f)     lowLux  = 0.01f;
    if (highLux <= lowLux) {
        ESP_LOGE(TAG, "High lux (%.2f) must be greater than low lux (%.2f)", highLux, lowLux);
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t highReg = floatToRegister(highLux);
    uint16_t lowReg  = floatToRegister(lowLux);
    ESP_RETURN_ON_ERROR(writeRegister(HIGH_LIMIT_REG, highReg), TAG, "write high limit failed");
    ESP_RETURN_ON_ERROR(writeRegister(LOW_LIMIT_REG,  lowReg),  TAG, "write low limit failed");
    return ESP_OK;
}

esp_err_t OPT3001::getThresholds(float& lowLux, float& highLux) const {
    uint16_t raw;
    esp_err_t err = readRegister(HIGH_LIMIT_REG, raw);
    if (err != ESP_OK) return err;
    highLux = registerToFloat(raw);
    err = readRegister(LOW_LIMIT_REG, raw);
    if (err != ESP_OK) return err;
    lowLux = registerToFloat(raw);
    return ESP_OK;
}

// ---------------------------------------------------------------------
// End‑Of‑Conversion interrupt mode
// ---------------------------------------------------------------------
esp_err_t OPT3001::enableEOCInterrupt() const {
    uint16_t eoc_code = 0xC000;
    return writeRegister(LOW_LIMIT_REG, eoc_code);
}

esp_err_t OPT3001::disableEOCInterrupt(float restoreLowLux) const {
    if (restoreLowLux < 0.01f) restoreLowLux = 0.01f;
    uint16_t lowReg = floatToRegister(restoreLowLux);
    return writeRegister(LOW_LIMIT_REG, lowReg);
}

// ---------------------------------------------------------------------
// Convenience setters
// ---------------------------------------------------------------------
esp_err_t OPT3001::setMode(uint8_t conversion_mode) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    cfg.mode_of_conversion = conversion_mode & 0x03;
    return configure(cfg);
}

esp_err_t OPT3001::setConversionTime(uint8_t conv_time) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    cfg.conversion_time = conv_time & 0x01;
    return configure(cfg);
}

esp_err_t OPT3001::setPolarity(uint8_t polarity) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    cfg.polarity_field = polarity & 0x01;
    return configure(cfg);
}

esp_err_t OPT3001::setLatch(uint8_t latch) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    cfg.latch_field = latch & 0x01;
    return configure(cfg);
}

esp_err_t OPT3001::setFaultCount(uint8_t count) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    // Convert user count (1,2,4,8) to field value
    uint8_t fc;
    switch (count) {
        case 1:  fc = FAULT_COUNT_1; break;
        case 2:  fc = FAULT_COUNT_2; break;
        case 4:  fc = FAULT_COUNT_4; break;
        case 8:  fc = FAULT_COUNT_8; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    cfg.fault_count_field = fc;
    return configure(cfg);
}

esp_err_t OPT3001::setMaskExponent(bool enable) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    cfg.mask_exponent_field = enable ? 1 : 0;
    return configure(cfg);
}

esp_err_t OPT3001::setFullScaleRange(uint8_t range) const {
    if (range > RN_AUTO && range != RN_AUTO) {
        ESP_LOGE(TAG, "Invalid range %u, must be 0..11 or RN_AUTO (12)", range);
        return ESP_ERR_INVALID_ARG;
    }
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    cfg.range_number_field = range & 0x0F;
    return configure(cfg);
}

esp_err_t OPT3001::setAutoRange(bool enable) const {
    return setFullScaleRange(enable ? RN_AUTO : RN_MANUAL_MIN);
}

// ---------------------------------------------------------------------
// Status flags
// ---------------------------------------------------------------------
esp_err_t OPT3001::getOverflowFlag(bool& overflow) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    overflow = (cfg.overflow_flag != 0);
    return ESP_OK;
}

esp_err_t OPT3001::isConversionReady(bool& ready) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    ready = (cfg.conversion_ready != 0);
    return ESP_OK;
}

esp_err_t OPT3001::getFlagHigh(bool& high) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    high = (cfg.high_field_flag != 0);
    return ESP_OK;
}

esp_err_t OPT3001::getFlagLow(bool& low) const {
    ConfigReg cfg{};
    esp_err_t err = getConfig(cfg);
    if (err != ESP_OK) return err;
    low = (cfg.low_field_flag != 0);
    return ESP_OK;
}

// ---------------------------------------------------------------------
// Raw result access
// ---------------------------------------------------------------------
esp_err_t OPT3001::readRawResult(uint8_t& exponent, uint16_t& mantissa) const {
    uint16_t raw;
    esp_err_t err = readRegister(RESULT_REG, raw);
    if (err != ESP_OK) return err;
    exponent = (raw >> 12) & 0x0F;
    mantissa = raw & 0x0FFF;
    return ESP_OK;
}

esp_err_t OPT3001::getResultRegister(uint16_t& raw) const {
    return readRegister(RESULT_REG, raw);
}

esp_err_t OPT3001::readLux(float& lux) const {
    uint16_t raw = 0;
    esp_err_t err = readRegister(RESULT_REG, raw);
    if (err != ESP_OK) return err;
    lux = registerToFloat(raw);
    return ESP_OK;
}

esp_err_t OPT3001::readRaw(uint8_t reg, uint16_t& out) const {
    return readRegister(reg, out);
}

// ---------------------------------------------------------------------
// Device identification
// ---------------------------------------------------------------------
esp_err_t OPT3001::getManufacturerID(uint16_t& id) const {
    return readRegister(MANUFACTURER_ID_REG, id);
}

esp_err_t OPT3001::getDeviceID(uint16_t& id) const {
    return readRegister(DEVICE_ID_REG, id);
}

// ---------------------------------------------------------------------
// Interrupt edge helper
// ---------------------------------------------------------------------
gpio_int_type_t OPT3001::getInterruptEdge() const {
    ConfigReg cfg{};
    if (getConfig(cfg) != ESP_OK) {
        // Default to falling edge if we cannot read polarity
        return GPIO_INTR_NEGEDGE;
    }
    // Active low → falling edge triggers interrupt
    // Active high → rising edge triggers interrupt
    return (cfg.polarity_field == INT_ACTIVE_LOW) ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
}

// ---------------------------------------------------------------------
// Interrupt handling
// ---------------------------------------------------------------------
esp_err_t OPT3001::enableInterrupt(EventGroupHandle_t evtGroup, EventBits_t bitMask) {
    if (int_gpio_ == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "No interrupt pin specified in constructor");
        return ESP_ERR_INVALID_ARG;
    }
    evt_group_ = evtGroup;
    evt_bit_   = bitMask;

    // Determine interrupt edge from current polarity setting
    gpio_int_type_t edge = getInterruptEdge();

    // Configure GPIO as input with pull‑up (INT is open‑drain)
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << int_gpio_,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = edge,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    // Install ISR service once
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err != ESP_OK) return err;
        isr_service_installed = true;
    }

    err = gpio_isr_handler_add(int_gpio_, isr_trampoline, this);
    if (err != ESP_OK) return err;

    gpio_intr_enable(int_gpio_);
    isr_enabled_ = true;
    return ESP_OK;
}

esp_err_t OPT3001::disableInterrupt() {
    if (int_gpio_ == GPIO_NUM_NC || !isr_enabled_) {
        return ESP_OK;
    }
    gpio_intr_disable(int_gpio_);
    gpio_isr_handler_remove(int_gpio_);
    isr_enabled_ = false;
    return ESP_OK;
}

void IRAM_ATTR OPT3001::isr_trampoline(void* arg) {
    auto* self = static_cast<OPT3001*>(arg);
    BaseType_t hpw = pdFALSE;
    if (self->evt_group_) {
        xEventGroupSetBitsFromISR(self->evt_group_, self->evt_bit_, &hpw);
    }
    if (hpw) portYIELD_FROM_ISR();
}

} // namespace ED_OPT3001