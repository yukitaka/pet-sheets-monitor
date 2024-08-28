#include <M5CoreS3.h>
#include <Wire.h>
#include "bme68x.h"
#include "bme68x_defs.h"

struct bme68x_dev bme;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data data;

BME68X_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    Wire.beginTransmission((uint8_t)(*((uint8_t *)intf_ptr)));
    Wire.write(reg_addr);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)(*((uint8_t *)intf_ptr)), (uint8_t)len);
    for (uint32_t i = 0; i < len; i++) {
        reg_data[i] = Wire.read();
    }
    return 0;
}

BME68X_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    Wire.beginTransmission((uint8_t)(*((uint8_t *)intf_ptr)));
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) {
        Wire.write(reg_data[i]);
    }
    Wire.endTransmission();
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
    while (!Serial) delay(10);

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

    int8_t rslt;
    Serial.println("Initializing BME688...");
    rslt = bme68x_init(&bme);
    if (rslt != BME68X_OK) {
        Serial.printf("Failed to initialize BME688, error code: %d\n", rslt);
        while (1);
    }
    Serial.println("BME688 initialized successfully");

    Serial.println("Set sensor");
    conf.filter = BME68X_FILTER_SIZE_3;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_2X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;
    rslt = bme68x_set_conf(&conf, &bme);

    Serial.println("Set heater");
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);

    Serial.println("Setting operation mode...");
    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
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


    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
    bme.delay_us(del_period, bme.intf_ptr);

    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    if (rslt != BME68X_OK) {
        Serial.println("Failed to get sensor data");
        return;
    }

    M5.Lcd.fillRect(0, 40, 320, 200, BLACK);
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf("Temp: %.2f C", data.temperature);
    M5.Lcd.setCursor(0, 70);
    M5.Lcd.printf("Press: %.2f hPa", data.pressure / 100.0f);
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.printf("Humidity: %.2f %%", data.humidity);
    M5.Lcd.setCursor(0, 130);
    M5.Lcd.printf("Gas: %.2f kOhm", data.gas_resistance / 1000.0f);

    Serial.printf("Temperature: %.2f C\n", data.temperature);
    Serial.printf("Pressure: %.2f hPa\n", data.pressure / 100.0f);
    Serial.printf("Humidity: %.2f %%\n", data.humidity);
    Serial.printf("Gas resistance: %.2f kOhm\n", data.gas_resistance / 1000.0f);
    Serial.println();

    delay(2000);
}
