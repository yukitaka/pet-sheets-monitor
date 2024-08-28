#include <M5CoreS3.h>
#include <Wire.h>
#include "bme68x.h"
#include "bme68x_defs.h"

#define SAMPLE_COUNT  UINT8_C(300)

struct bme68x_dev bme;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data data[3];

BME68X_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(device_addr);
    Wire.write(reg_addr);
    int endResult = Wire.endTransmission(false);
    if (endResult != 0) {
      Serial.printf("I2C Read Error: endTransmission failed with code %d\n", endResult);
      return 1;
    }

    uint32_t received = Wire.requestFrom((int)device_addr, (uint8_t)len);
    if (received != len) {
      Serial.printf("I2C Read Error: Requested %d bytes, but received %d\n", len, received);
      return 1;
    }

    for (uint32_t i = 0; i < len; i++) {
        if (Wire.available()) {
            reg_data[i] = Wire.read();
        } else {
            Serial.printf("I2C Read Error: Not enough data available at index %d\n", i);
        }
    }

    return 0;
}

BME68X_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(device_addr);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) {
        Wire.write(reg_data[i]);
    }
    if (Wire.endTransmission() != 0) {
      return 1;
    }

    return 0;
}

void delay_us(uint32_t us, void *intf_ptr)
{
    delayMicroseconds(us);
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Wire.begin();
    Serial.begin(9600);
    delay(1000);

    while (!Serial) delay(10);

    Serial.println("Scanning I2C bus...");
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02X\n", address);
        }
    }

    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("BME688 Sensor Data");

    uint8_t dev_addr = BME68X_I2C_ADDR_HIGH;
    bme.intf = BME68X_I2C_INTF;
    bme.read = i2c_read;
    bme.write = i2c_write;
    bme.delay_us = delay_us;
    bme.intf_ptr = &dev_addr;
    bme.amb_temp = 25;

    Wire.beginTransmission(dev_addr);
    if (Wire.endTransmission() != 0) {
        Serial.println("I2C device not found. Please check connections.");
        while(1);
    }

    int8_t rslt;
    Serial.println("Initializing BME688...");
    rslt = bme68x_init(&bme);
    if (rslt != BME68X_OK) {
        Serial.printf("Failed to initialize BME688, error code: %d\n", rslt);
        while (1);
    }
    Serial.println("BME688 initialized successfully");

    Serial.println("Set sensor");
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    if (rslt != BME68X_OK) {
        Serial.printf("Failed to set sensor configuration, error code: %d\n", rslt);
        while (1);
    }

    Serial.println("Set heater");
    uint16_t temp_prof[10] = { 200, 240, 280, 320, 360, 360, 320, 280, 240, 200 };
    uint16_t dur_prof[10] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };

    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof = temp_prof;
    heatr_conf.heatr_dur_prof = dur_prof;
    heatr_conf.profile_len = 10;
    rslt = bme68x_set_heatr_conf(BME68X_SEQUENTIAL_MODE, &heatr_conf, &bme);
    if (rslt != BME68X_OK) {
        Serial.printf("Failed to set heater configuration, error code: %d\n", rslt);
        while (1);
    }
    Serial.println("Setting operation mode...");
    rslt = bme68x_set_op_mode(BME68X_SEQUENTIAL_MODE, &bme);
    if (rslt != BME68X_OK) {
        Serial.printf("Failed to set operation mode, error code: %d\n", rslt);
        return;
    } else {
        Serial.println("Operation mode set successfully");
    }
}

void loop() {
    uint8_t n_fields;
    uint32_t del_period;
    int8_t rslt;
    uint16_t sample_count = 1;

    while (sample_count <= SAMPLE_COUNT) {
        del_period = bme68x_get_meas_dur(BME68X_SEQUENTIAL_MODE, &conf, &bme) + (heatr_conf.heatr_dur_prof[0] * 1000);
        bme.delay_us(del_period, bme.intf_ptr);

        rslt = bme68x_get_data(BME68X_SEQUENTIAL_MODE, data, &n_fields, &bme);
        if (rslt != BME68X_OK) {
            Serial.printf("Failed to get sensor data, error code: %d\n", rslt);
            return;
        }

        for (uint8_t i = 0; i < n_fields; i++) {
            M5.Lcd.fillRect(0, 40, 320, 200, BLACK);
            M5.Lcd.setCursor(0, 40);
            M5.Lcd.printf("Temp: %.2f C", data[i].temperature);
            M5.Lcd.setCursor(0, 70);
            M5.Lcd.printf("Press: %.2f hPa", data[i].pressure / 100.0f);
            M5.Lcd.setCursor(0, 100);
            M5.Lcd.printf("Humidity: %.2f %%", data[i].humidity);
            M5.Lcd.setCursor(0, 130);
            M5.Lcd.printf("Gas: %.2f kOhm", data[i].gas_resistance / 1000.0f);

            Serial.printf("%u, %.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d",
                sample_count,
                data[i].temperature,
                data[i].pressure / 100.0f,
                data[i].humidity,
                data[i].gas_resistance / 1000.0f,
                data[i].status,
                data[i].gas_index,
                data[i].meas_index);
            Serial.println();
            sample_count++;
        }
    }

    delay(2000);
}

