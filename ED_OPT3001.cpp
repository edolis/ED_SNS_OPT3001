#include "ED_OPT3001.h"
#include <cmath>
#include "esp_log.h"
#include "esp_check.h"

namespace ED_OPT3001 {

static const char* TAG = "ED_OPT3001";

OPT3001::OPT3001(I2CBus& bus, uint8_t dev_addr, gpio_num_t intPin)
    : m_bus(bus), m_addr(dev_addr), int_gpio_(intPin) {}

esp_err_t OPT3001::writeRegister(uint8_t reg, uint16_t value) const {
    uint8_t wr[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return m_bus.write(m_addr, wr, sizeof(wr));
}

esp_err_t OPT3001::readRegister(uint8_t reg, uint16_t& value) const {
    uint8_t rx[2];
    esp_err_t err = m_bus.write_then_read(m_addr, &reg, 1, rx, 2);
    if (err == ESP_OK) value = ((uint16_t)rx[0] << 8) | rx[1];
    return err;
}

float OPT3001::convertRawToLux(uint16_t rawRegister) {
    return registerToFloat(rawRegister);
}

static uint8_t lsbFromLux(float lux) {
    uint8_t exp = 0;
    while (lux > (40.95f * (1 << exp)) && exp < 0x0B) exp++;
    return exp;
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
    uint8_t exp = (reg >> 12) & 0x0F;
    uint16_t mant = reg & 0x0FFF;
    return 0.01f * powf(2.0f, (float)exp) * mant;
}

esp_err_t OPT3001::configure(const ConfigReg& cfg) const {
    return writeRegister(CONFIG_REG, cfg.config_register);
}

esp_err_t OPT3001::getConfig(ConfigReg& cfg) const {
    return readRegister(CONFIG_REG, cfg.config_register);
}

esp_err_t OPT3001::setThresholds(float lowLux, float highLux) const {
    if (highLux > 83865.60f) highLux = 83865.60f;
    if (lowLux  < 0.01f)     lowLux  = 0.01f;
    if (highLux <= lowLux) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(writeRegister(HIGH_LIMIT_REG, floatToRegister(highLux)), TAG, "high limit");
    ESP_RETURN_ON_ERROR(writeRegister(LOW_LIMIT_REG,  floatToRegister(lowLux)),  TAG, "low limit");
    return ESP_OK;
}

esp_err_t OPT3001::getThresholds(float& lowLux, float& highLux) const {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(readRegister(HIGH_LIMIT_REG, raw), TAG, "read high limit");
    highLux = registerToFloat(raw);
    ESP_RETURN_ON_ERROR(readRegister(LOW_LIMIT_REG, raw), TAG, "read low limit");
    lowLux = registerToFloat(raw);
    return ESP_OK;
}

esp_err_t OPT3001::enableEOCInterrupt() const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    cfg.fault_count_field = FAULT_COUNT_1;
    ESP_RETURN_ON_ERROR(configure(cfg), TAG, "set fault count");
    ESP_RETURN_ON_ERROR(writeRegister(HIGH_LIMIT_REG, 0xFFFF), TAG, "set high limit");
    ESP_RETURN_ON_ERROR(writeRegister(LOW_LIMIT_REG,  0xC000), TAG, "set low limit");
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    cfg.latch_field = LATCH_TRANSPARENT;
    return configure(cfg);
}

esp_err_t OPT3001::disableEOCInterrupt(float restoreLowLux) const {
    if (restoreLowLux < 0.01f) restoreLowLux = 0.01f;
    return writeRegister(LOW_LIMIT_REG, floatToRegister(restoreLowLux));
}

esp_err_t OPT3001::setMode(uint8_t conversion_mode) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    cfg.mode_of_conversion = conversion_mode & 0x03;
    return configure(cfg);
}
esp_err_t OPT3001::setConversionTime(uint8_t conv_time) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    cfg.conversion_time = conv_time & 0x01;
    return configure(cfg);
}
esp_err_t OPT3001::setPolarity(uint8_t polarity) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    cfg.polarity_field = polarity & 0x01;
    return configure(cfg);
}
esp_err_t OPT3001::setLatch(uint8_t latch) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    cfg.latch_field = latch & 0x01;
    return configure(cfg);
}
esp_err_t OPT3001::setFaultCount(uint8_t count) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    uint8_t fc;
    switch (count) {
        case 1: fc = FAULT_COUNT_1; break;
        case 2: fc = FAULT_COUNT_2; break;
        case 4: fc = FAULT_COUNT_4; break;
        case 8: fc = FAULT_COUNT_8; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    cfg.fault_count_field = fc;
    return configure(cfg);
}
esp_err_t OPT3001::setMaskExponent(bool enable) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    cfg.mask_exponent_field = enable ? 1 : 0;
    return configure(cfg);
}
esp_err_t OPT3001::setFullScaleRange(uint8_t range) const {
    if (range > RN_AUTO && range != RN_AUTO) return ESP_ERR_INVALID_ARG;
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    cfg.range_number_field = range & 0x0F;
    return configure(cfg);
}
esp_err_t OPT3001::setAutoRange(bool enable) const {
    return setFullScaleRange(enable ? RN_AUTO : RN_MANUAL_MIN);
}

esp_err_t OPT3001::getOverflowFlag(bool& overflow) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    overflow = (cfg.overflow_flag != 0);
    return ESP_OK;
}
esp_err_t OPT3001::isConversionReady(bool& ready) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    ready = (cfg.conversion_ready != 0);
    return ESP_OK;
}
esp_err_t OPT3001::getFlagHigh(bool& high) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    high = (cfg.high_field_flag != 0);
    return ESP_OK;
}
esp_err_t OPT3001::getFlagLow(bool& low) const {
    ConfigReg cfg;
    ESP_RETURN_ON_ERROR(getConfig(cfg), TAG, "getConfig");
    low = (cfg.low_field_flag != 0);
    return ESP_OK;
}

esp_err_t OPT3001::readRawResult(uint8_t& exponent, uint16_t& mantissa) const {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(readRegister(RESULT_REG, raw), TAG, "read result");
    exponent = (raw >> 12) & 0x0F;
    mantissa = raw & 0x0FFF;
    return ESP_OK;
}
esp_err_t OPT3001::getResultRegister(uint16_t& raw) const {
    return readRegister(RESULT_REG, raw);
}
esp_err_t OPT3001::readLux(float& lux) const {
    uint16_t raw;
    esp_err_t err = readRegister(RESULT_REG, raw);
    if (err != ESP_OK) return err;
    lux = registerToFloat(raw);
    // Read config register to ensure interrupt flag is cleared
    ConfigReg dummy;
    return getConfig(dummy);   // ignore error, but ensures clearing
}

esp_err_t OPT3001::getManufacturerID(uint16_t& id) const {
    return readRegister(MANUFACTURER_ID_REG, id);
}
esp_err_t OPT3001::getDeviceID(uint16_t& id) const {
    return readRegister(DEVICE_ID_REG, id);
}

gpio_int_type_t OPT3001::getInterruptEdge() const {
    ConfigReg cfg;
    if (getConfig(cfg) != ESP_OK) return GPIO_INTR_NEGEDGE;
    return (cfg.polarity_field == INT_ACTIVE_LOW) ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
}

esp_err_t OPT3001::enableInterrupt(EventGroupHandle_t evtGroup, EventBits_t bitMask) {
    if (int_gpio_ == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;
    evt_group_ = evtGroup;
    evt_bit_ = bitMask;
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << int_gpio_,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = getInterruptEdge(),
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio config");
    static bool isr_installed = false;
    if (!isr_installed) {
        ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_IRAM), TAG, "isr service");
        isr_installed = true;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(int_gpio_, isr_trampoline, this), TAG, "add isr");
    gpio_intr_enable(int_gpio_);
    isr_enabled_ = true;
    return ESP_OK;
}

esp_err_t OPT3001::disableInterrupt() {
    if (int_gpio_ == GPIO_NUM_NC || !isr_enabled_) return ESP_OK;
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