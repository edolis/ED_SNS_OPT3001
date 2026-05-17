# ED_OPT3001 – Driver for Texas Instruments OPT3001 Ambient Light Sensor

## IC Technical Features Summary

The **OPT3001** is a digital ambient light sensor (ALS) with a spectral response closely matching the human eye (photopic response). Key specifications:

| Parameter                      | Value / Description                                      |
|--------------------------------|----------------------------------------------------------|
| **Measurement range**          | 0.01 lux to 83,865 lux (23‑bit effective dynamic range) |
| **Resolution**                 | 0.01 lux (lowest full‑scale range)                       |
| **Automatic full‑scale range** | Selects optimal range without user intervention          |
| **Infrared rejection**         | >99% (typ) – rejects IR that human eye cannot see       |
| **Conversion times**           | 100 ms or 800 ms (integrated noise filtering)            |
| **Operating current**          | 1.8 µA (typ, active), 0.3 µA (shutdown)                  |
| **Power supply**               | 1.6 V to 3.6 V (I²C pins tolerant up to 5.5 V)           |
| **Interrupt**                  | Open‑drain, programmable polarity, latch/transparent mode|
| **I²C speed**                  | Up to 2.6 MHz (high‑speed mode)                          |
| **Package**                    | 2.0 mm × 2.0 mm × 0.65 mm USON‑6                         |
| **Operating temperature**      | –40°C to +85°C                                           |

The OPT3001 integrates an ADC, a photodiode with optical filtering, and a digital I²C interface. It is ideal for display backlight control, lighting automation, and any human‑centric light sensing application.

---

## Driver Overview

The **ED_OPT3001** driver provides a complete C++ class for ESP‑IDF 5.5 (and later) that exposes all hardware capabilities of the OPT3001. It is designed to work with the `I2CBus` wrapper (provided separately) and integrates with FreeRTOS event groups for interrupt‑driven operation.

This driver supports:

- **Continuous and single‑shot** conversion modes
- **Automatic full‑scale range** (auto‑gain) or **manual range selection** (12 fixed ranges)
- **End‑of‑Conversion (EOC)** interrupt – INT pin asserts after each measurement
- **Window comparator** with programmable low/high thresholds and fault count (1,2,4,8 consecutive faults)
- **Latch modes**: window‑latching (user must clear) or transparent (self‑clearing)
- **Polarity control** – active‑low or active‑high INT pin (driver automatically selects GPIO interrupt edge)
- **Mask exponent** – forces result register exponent to 0 when manually programming the range (simplifies processing)
- **Raw register access** – read the 16‑bit result register, extract exponent (4 bits) and mantissa (12 bits)
- **Status flags**: overflow, conversion ready, flag high, flag low
- **Device identification** – read manufacturer ID (0x5449) and device ID (0x3001)

All methods return `esp_err_t` and follow ESP‑IDF error handling conventions.

---

## New Implementations (Features Beyond Basic Reading)

Compared to a minimal driver that only reads lux, this implementation adds the following capabilities. Each feature is exposed through a clear public method and is described with its intended use case.

### 1. Manual Full‑Scale Range Selection

**Methods:** `setFullScaleRange(uint8_t range)`, `setAutoRange(bool enable)`

The OPT3001 has 12 fixed full‑scale ranges (0.01 × 2^exp lux, exp = 0…11). By default, the auto‑range mode (`RN_AUTO`) selects the best range for each measurement. However, if the lighting environment is stable and well known (e.g., a dark room or a brightly lit office), fixing the range can reduce conversion noise and improve repeatability. Manual range also allows the mask exponent feature (see below).

**Use case:** A laboratory instrument that measures light under constant illumination – fixing the range eliminates range‑switching artefacts.

### 2. Fault Count

**Method:** `setFaultCount(uint8_t count)` (allowed values: 1, 2, 4, 8)

The fault count defines how many consecutive measurements must be outside the window (above high threshold or below low threshold) before the interrupt pin and status flags are activated. This prevents false triggers from short light flickers (e.g., passing shadows or fluorescent lamp startup).

**Use case:** A smart streetlight that should react only when darkness persists for several seconds, ignoring momentary shadows from passing cars.

### 3. Mask Exponent (ME)

**Method:** `setMaskExponent(bool enable)`

When the full‑scale range is manually set (RN < RN_AUTO) and ME = 1, the device forces the exponent field in the result register to 0. Consequently, the lux value becomes simply `lux = 0.01 × mantissa`. This simplifies software processing because the user no longer needs to decode the exponent.

**Use case:** A simple microcontroller with limited mathematical capabilities – the lux value is directly proportional to the raw mantissa.

### 4. Status Flags – Overflow, Conversion Ready, High/Low Flags

**Methods:** `getOverflowFlag()`, `isConversionReady()`, `getFlagHigh()`, `getFlagLow()`

These read‑only flags give immediate insight into the sensor’s state:

- **Overflow (OVF)** – indicates that the input light exceeded the selected full‑scale range (or the maximum range in auto‑mode). The measurement may be invalid.
- **Conversion Ready (CRF)** – set when a new conversion result is available. Useful for polling without using the interrupt pin.
- **Flag High (FH)** and **Flag Low (FL)** – reflect whether the last result (after fault count) is above the high threshold or below the low threshold. In latched mode, they stay asserted until the configuration register is read.

**Use case:** A building automation controller that polls the sensor every 100 ms – it can check `isConversionReady()` before reading to avoid fetching stale data.

### 5. Raw Register Access

**Methods:** `getResultRegister(uint16_t& raw)`, `readRawResult(uint8_t& exponent, uint16_t& mantissa)`, `static float convertRawToLux(uint16_t rawRegister)`

Advanced users may want to process the raw 16‑bit result themselves, for example to implement custom filtering or to log the raw exponent/mantissa for debugging. The driver provides three ways:

- `getResultRegister()` returns the complete 16‑bit value.
- `readRawResult()` splits it into exponent (4 bits) and mantissa (12 bits).
- `convertRawToLux()` is a public static helper that converts any raw register value to lux using the formula `lux = 0.01 × 2^exp × mantissa`.

**Use case:** A data logger that stores raw values to save memory (2 bytes per sample) and converts them later on a PC.

### 6. Automatic Interrupt Edge Selection Based on Polarity

**Method:** `enableInterrupt()` (internal detection)

When the user calls `enableInterrupt()`, the driver reads the current polarity bit (`POL`) from the configuration register. If `POL = INT_ACTIVE_LOW` (default), it configures the GPIO interrupt as **falling edge**; if `POL = INT_ACTIVE_HIGH`, it configures **rising edge**. This ensures that the interrupt always triggers when the INT pin becomes active, regardless of the polarity setting, without any extra code from the user.

**Use case:** A system where the interrupt line is shared with another device that requires active‑high signalling – the user simply calls `setPolarity(INT_ACTIVE_HIGH)` and `enableInterrupt()` adapts automatically.

---

## Detailed Usage Examples

### Example 1: Continuous Reading with Auto‑Range and Polling (No Interrupt)

This is the simplest configuration. The sensor runs continuously, and the application reads the latest lux value once per second.

```cpp
#include "ED_i2c.h"
#include "ED_OPT3001.h"

I2CBus bus(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000);
i2c_master_dev_handle_t dev;
bus.get_device(0x44, &dev);

ED_OPT3001::OPT3001 sensor(dev, GPIO_NUM_NC);  // interrupt not used

ED_OPT3001::OPT3001::ConfigReg cfg{};
cfg.mode_of_conversion = ED_OPT3001::OPT3001::CONTINUOUS_B;
cfg.conversion_time    = ED_OPT3001::OPT3001::TIME_800MS;
cfg.range_number_field = ED_OPT3001::OPT3001::RN_AUTO;
sensor.configure(cfg);

while (true) {
    float lux;
    if (sensor.readLux(lux) == ESP_OK) {
        printf("Lux: %.2f\n", lux);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
}
```

**Explanation:**
- The sensor is initialised with automatic full‑scale range (no user range management).
- 800 ms integration time gives best resolution and rejects 50/60 Hz flicker.
- The loop reads the result register once per second – the sensor updates internally every 800 ms, so the value may be the same for two consecutive reads if the light does not change.
- No interrupt pin is needed; the application polls at its own pace.

---

### Example 2: Interrupt‑Driven Reading with End‑of‑Conversion Mode (EOC)

This is ideal for battery‑powered devices. The processor sleeps until the INT pin signals that a new measurement is ready.

```cpp
#include "freertos/event_groups.h"

#define INT_BIT BIT0

EventGroupHandle_t evt = xEventGroupCreate();

// (I2C initialisation and sensor creation as above)
sensor.configure(cfg);          // continuous mode, auto‑range, 800 ms
sensor.enableEOCInterrupt();    // INT asserts after each conversion
sensor.enableInterrupt(evt, INT_BIT);

float lux;
while (true) {
    xEventGroupWaitBits(evt, INT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    sensor.readLux(lux);
    printf("Lux: %.2f\n", lux);
}
```

**Explanation:**
- `enableEOCInterrupt()` writes the special value 0xC000 to the low‑limit register, configuring the device to assert the INT pin after every completed conversion.
- `enableInterrupt()` sets up the GPIO as an input with a pull‑up, installs an ISR, and connects it to the event group. The driver automatically chooses the correct interrupt edge based on the polarity setting (default active‑low → falling edge).
- The main task blocks on `xEventGroupWaitBits()` until the ISR sets the bit. It then reads the lux value. The INT pin is cleared because reading the result register resets the latch (in window‑latch mode).

**Use case:** A wearable device that wakes up every 800 ms to log light level and immediately returns to sleep, consuming minimal power.

---

### Example 3: Window Comparator with Fault Count (Day/Night Detection)

This example triggers an interrupt only when the light level remains **outside** the range 50–500 lux for **two consecutive** measurements. Useful for ignoring brief shadows or flashlights.

```cpp
sensor.setThresholds(50.0f, 500.0f);
sensor.setFaultCount(ED_OPT3001::OPT3001::FAULT_COUNT_2);
sensor.setLatch(ED_OPT3001::OPT3001::LATCH_WINDOW);
sensor.setPolarity(ED_OPT3001::OPT3001::INT_ACTIVE_LOW);
sensor.enableInterrupt(evt, INT_BIT);

while (true) {
    xEventGroupWaitBits(evt, INT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    bool high, low;
    sensor.getFlagHigh(high);
    sensor.getFlagLow(low);
    if (high) printf("Light above 500 lux for 2 consecutive samples\n");
    if (low)  printf("Light below 50 lux for 2 consecutive samples\n");
    // After reading the config register (inside getFlagHigh), the INT pin clears.
}
```

**Explanation:**
- The window is defined by `setThresholds()`.
- Fault count = 2 means that two consecutive results above 500 lux (or below 50 lux) are required to set the interrupt.
- Latch mode = `LATCH_WINDOW` keeps the INT pin and flags active until the configuration register is read (which happens inside `getFlagHigh()`).
- The application distinguishes between high and low events using the flag high/low methods.

**Use case:** A greenhouse controller that triggers ventilation only when the light has been too intense for several seconds, avoiding nuisance triggers from a passing cloud.

---

### Example 4: Manual Full‑Scale Range with Mask Exponent

For a fixed indoor lighting environment (e.g., an office with 200–300 lux), manual range can be used. By enabling the mask exponent, the conversion is simplified.

```cpp
// Manually select the range where full‑scale is 655.20 lux (RN = 4)
sensor.setFullScaleRange(4);
sensor.setMaskExponent(true);
sensor.setMode(ED_OPT3001::OPT3001::CONTINUOUS_B);

while (true) {
    uint16_t raw;
    sensor.getResultRegister(raw);
    // With ME=1, exponent is forced to 0, so lux = 0.01 * mantissa
    uint16_t mantissa = raw & 0x0FFF;
    float lux = 0.01f * mantissa;
    printf("Lux = %.2f (raw=0x%04X)\n", lux, raw);
    vTaskDelay(pdMS_TO_TICKS(1000));
}
```

**Explanation:**
- The user selects a manual range (RN=4 → full‑scale 655.20 lux).
- `setMaskExponent(true)` forces the result register’s exponent bits to 0.
- The raw 16‑bit value now has mantissa in bits 11…0, and the four exponent bits are zero. Lux becomes simply `0.01 × mantissa`. No floating‑point pow() is required.

**Use case:** A low‑power sensor node that sends raw 16‑bit values over a radio link; the receiver converts to lux using a simple integer multiplication.

---

## Error Handling and Debugging

All methods return `esp_err_t`. Common errors:

| Error code | Meaning |
|------------|---------|
| `ESP_OK` | Operation successful. |
| `ESP_ERR_INVALID_ARG` | Invalid parameter (e.g., threshold low > high, fault count not 1/2/4/8, range out of 0…11). |
| `ESP_ERR_TIMEOUT` | I²C communication timed out (e.g., sensor not responding, wrong address). |
| `ESP_FAIL` | General failure (e.g., sensor not powered, configuration not accepted). |

**Debugging tips:**
- Use `getManufacturerID()` and `getDeviceID()` to confirm the sensor is responding correctly. Expected values: 0x5449 and 0x3001.
- Monitor the `overflow_flag` – if it is set frequently, the light exceeds the maximum range (83k lux).
- In polling mode, check `isConversionReady()` before reading to ensure you are not reading stale data.
- If interrupts do not fire, verify that `enableInterrupt()` is called after configuring the sensor and that the GPIO pin is correctly connected. Use a logic analyser to see INT pin toggling.

---

## Integration with I2CBus

The driver expects an already created I²C device handle. Use the `I2CBus` wrapper as shown below. Do not call any static `addDevice()` method – it has been removed.

```cpp
#include "ED_i2c.h"

I2CBus bus(I2C_NUM_0, SDA_PIN, SCL_PIN, 400000);
i2c_master_dev_handle_t opt_handle;
ESP_ERROR_CHECK(bus.get_device(OPT3001_I2C_ADDR, &opt_handle));
ED_OPT3001::OPT3001 sensor(opt_handle, INT_PIN);
```

---

## API Reference Summary

| Category | Methods |
|----------|---------|
| **Configuration** | `configure()`, `getConfig()`, `setMode()`, `setConversionTime()`, `setPolarity()`, `setLatch()`, `setFaultCount()`, `setMaskExponent()`, `setFullScaleRange()`, `setAutoRange()` |
| **Reading** | `readLux()`, `readRawResult()`, `getResultRegister()`, `convertRawToLux()` (static) |
| **Thresholds** | `setThresholds()`, `getThresholds()`, `enableEOCInterrupt()`, `disableEOCInterrupt()` |
| **Status** | `getOverflowFlag()`, `isConversionReady()`, `getFlagHigh()`, `getFlagLow()` |
| **Identification** | `getManufacturerID()`, `getDeviceID()` |
| **Interrupt** | `enableInterrupt()`, `disableInterrupt()` |

---

## Dependencies

- ESP‑IDF v5.5 or later
- `driver/i2c_master.h`
- `freertos/FreeRTOS.h`, `freertos/event_groups.h`
- `esp_check.h`, `esp_log.h`
- `ED_i2c` (I2CBus wrapper)

---

## Revision History

| Version | Date       | Changes |
|---------|------------|---------|
| 0.3     | 2026-05-17 | Added manual range, fault count, mask exponent, status flags, raw access, automatic interrupt edge selection. Removed `addDevice()`. |
```