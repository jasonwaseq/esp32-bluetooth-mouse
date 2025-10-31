#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "ICM-42670-P-Mouse"

// I2C Configuration
#define I2C_MASTER_SCL_IO   8
#define I2C_MASTER_SDA_IO   10
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000

// ICM-42670-P Registers
#define REG_DEVICE_CONFIG    0x11
#define REG_WHO_AM_I         0x75
#define REG_PWR_MGMT0        0x1F
#define REG_ACCEL_CONFIG0    0x21
#define REG_ACCEL_X1         0x0B
#define REG_ACCEL_X0         0x0C
#define REG_ACCEL_Y1         0x0D
#define REG_ACCEL_Y0         0x0E
#define REG_ACCEL_Z1         0x0F
#define REG_ACCEL_Z0         0x10
#define WHOAMI_EXPECT        0x67

// Tilt tuning
#define TILT_THRESHOLD_LSB   4000
#define DEADZONE_LSB         1000
#define READ_INTERVAL_MS     100

static uint8_t s_icm_addr = 0x68;

// ------------------------ I2C helpers ------------------------

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) return ret;

    i2c_set_timeout(I2C_MASTER_NUM, 0xFFFFF);
    return ESP_OK;
}

static esp_err_t i2c_cmd_begin_retry(i2c_cmd_handle_t cmd)
{
    const int max_attempts = 5;
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        if (ret == ESP_OK) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_FAIL;
}

static esp_err_t i2c_write_u8(uint8_t dev, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_cmd_begin_retry(cmd);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_u8(uint8_t dev, uint8_t reg, uint8_t *val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_cmd_begin_retry(cmd);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ------------------------ Sensor reading ------------------------

static esp_err_t read_axis_2b(uint8_t msb_reg, int16_t *out)
{
    uint8_t hi = 0, lo = 0;
    esp_err_t r = i2c_read_u8(s_icm_addr, msb_reg, &hi);
    if (r != ESP_OK) return r;

    r = i2c_read_u8(s_icm_addr, msb_reg + 1, &lo);
    if (r != ESP_OK) return r;

    *out = (int16_t)((hi << 8) | lo);
    return ESP_OK;
}

static esp_err_t read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    esp_err_t r;
    r = read_axis_2b(REG_ACCEL_X1, ax); if (r != ESP_OK) return r;
    r = read_axis_2b(REG_ACCEL_Y1, ay); if (r != ESP_OK) return r;
    r = read_axis_2b(REG_ACCEL_Z1, az); if (r != ESP_OK) return r;
    return ESP_OK;
}

// ------------------------ IMU helpers ------------------------

static bool probe_addr(uint8_t addr)
{
    uint8_t who = 0;
    if (i2c_read_u8(addr, REG_WHO_AM_I, &who) == ESP_OK && who == WHOAMI_EXPECT) {
        s_icm_addr = addr;
        ESP_LOGI(TAG, "ICM-42670 at 0x%02X (WHO_AM_I=0x%02X)", addr, who);
        return true;
    }
    return false;
}

static esp_err_t icm_init(void)
{
    uint8_t who, pwr, acc;

    ESP_LOGI(TAG, "Scanning for ICM-42670...");
    if (!probe_addr(0x68) && !probe_addr(0x69)) {
        ESP_LOGE(TAG, "ICM-42670 not found");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Soft reset...");
    i2c_write_u8(s_icm_addr, REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    i2c_read_u8(s_icm_addr, REG_WHO_AM_I, &who);
    if (who != WHOAMI_EXPECT) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: 0x%02X", who);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WHO_AM_I OK (0x%02X)", who);

    ESP_LOGI(TAG, "Enabling accelerometer...");
    i2c_write_u8(s_icm_addr, REG_PWR_MGMT0, 0x03);
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_write_u8(s_icm_addr, REG_ACCEL_CONFIG0, 0x69);
    vTaskDelay(pdMS_TO_TICKS(10));

    if (i2c_read_u8(s_icm_addr, REG_PWR_MGMT0, &pwr) == ESP_OK)
        ESP_LOGI(TAG, "PWR_MGMT0 = 0x%02X", pwr);
    if (i2c_read_u8(s_icm_addr, REG_ACCEL_CONFIG0, &acc) == ESP_OK)
        ESP_LOGI(TAG, "ACCEL_CONFIG0 = 0x%02X", acc);

    ESP_LOGI(TAG, "Sensor ready");
    return ESP_OK;
}

// ------------------------ Direction printing ------------------------

static void print_direction(int16_t ax, int16_t ay, int16_t az)
{
    static int dbg = 0;

    if ((dbg++ % 20) == 0) {
        ESP_LOGI(TAG, "Raw: X=%d, Y=%d, Z=%d", ax, ay, az);
    }

    if (ax > -DEADZONE_LSB && ax < DEADZONE_LSB) ax = 0;
    if (ay > -DEADZONE_LSB && ay < DEADZONE_LSB) ay = 0;

    char msg[24] = {0};
    bool first = true;

    if (ay > TILT_THRESHOLD_LSB)       { strcat(msg, "UP"); first = false; }
    else if (ay < -TILT_THRESHOLD_LSB){ strcat(msg, "DOWN"); first = false; }

    if (ax < -TILT_THRESHOLD_LSB)       { if (!first) strcat(msg, " "); strcat(msg, "RIGHT"); first = false; }
    else if (ax > TILT_THRESHOLD_LSB){ if (!first) strcat(msg, " "); strcat(msg, "LEFT"); first = false; }

    if (!first) ESP_LOGI(TAG, "%s", msg);
}

// ------------------------ Main ------------------------

void app_main(void)
{
    ESP_LOGI(TAG, "Lab 4.1: Board Movement Detection (ICM-42670-P)");
    ESP_LOGI(TAG, "I2C: SDA=GPIO%d, SCL=GPIO%d @ %d Hz",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");

    if (icm_init() != ESP_OK) {
        ESP_LOGE(TAG, "Init failed");
        return;
    }

    ESP_LOGI(TAG, "Warming up...");
    for (int i = 0; i < 10; i++) {
        int16_t ax, ay, az;
        read_accel(&ax, &ay, &az);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Starting detection. Tilt the board...");
    int error_count = 0;

    while (1) {
        int16_t ax = 0, ay = 0, az = 0;

        if (read_accel(&ax, &ay, &az) == ESP_OK) {
            error_count = 0;
            print_direction(ax, ay, az);
        } else {
            if ((++error_count % 10) == 1) {
                ESP_LOGW(TAG, "Read error (count=%d)", error_count);
            }
            if (error_count > 50) {
                ESP_LOGW(TAG, "Reinit sensor");
                icm_init();
                error_count = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL_MS));
    }
}

