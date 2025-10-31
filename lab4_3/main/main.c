#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#define TAG "IMU-BLE-Mouse"

// ==================== I2C & ICM-42670-P Configuration ====================
#define I2C_MASTER_SCL_IO   8
#define I2C_MASTER_SDA_IO   10
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000

// ==================== Button Configuration ====================
#define BUTTON_GPIO         9           // GPIO9 - BOOT button on ESP32-C3
#define BUTTON_ACTIVE_LEVEL 0           // Button pressed = LOW (with pull-up)
#define MOUSE_LEFT_BUTTON   0x01        // Left mouse button bit mask

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

// ==================== Inclination Thresholds ====================
#define DEADZONE_LSB         1000
#define A_BIT_THRESHOLD      3000   // Light inclination
#define A_LOT_THRESHOLD      8000   // Strong inclination

// ==================== Mouse Movement Configuration ====================
#define READ_INTERVAL_MS     10     // Fast polling for smooth movement
#define BASE_SPEED_BIT       2      // Base speed for "a bit" inclined
#define BASE_SPEED_LOT       5      // Base speed for "a lot" inclined

// Acceleration timing thresholds (in milliseconds)
#define ACCEL_LEVEL_1_MS     0      // a=1 (immediate)
#define ACCEL_LEVEL_2_MS     10     // a=2 (after 10ms)
#define ACCEL_LEVEL_3_MS     50     // a=3 (after 50ms)

// ==================== Inclination States ====================
typedef enum {
    INCL_NEUTRAL = 0,
    INCL_A_BIT_LEFT,
    INCL_A_LOT_LEFT,
    INCL_A_BIT_RIGHT,
    INCL_A_LOT_RIGHT,
    INCL_A_BIT_UP,
    INCL_A_LOT_UP,
    INCL_A_BIT_DOWN,
    INCL_A_LOT_DOWN
} inclination_state_t;

// ==================== Global Variables ====================
static uint8_t s_icm_addr = 0x68;
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

// Tracking for acceleration
static inclination_state_t prev_x_state = INCL_NEUTRAL;
static inclination_state_t prev_y_state = INCL_NEUTRAL;
static int64_t x_incl_start_time = 0;
static int64_t y_incl_start_time = 0;

#define HIDD_DEVICE_NAME "IMU-Mouse-ESP32C3"

// ==================== I2C Helper Functions ====================
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

// ==================== Sensor Reading Functions ====================
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

// ==================== IMU Initialization ====================
static esp_err_t button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Button initialized on GPIO%d", BUTTON_GPIO);
    }
    return ret;
}

static bool probe_addr(uint8_t addr)
{
    uint8_t who = 0;
    if (i2c_read_u8(addr, REG_WHO_AM_I, &who) == ESP_OK && who == WHOAMI_EXPECT) {
        s_icm_addr = addr;
        ESP_LOGI(TAG, "ICM-42670 found at 0x%02X (WHO_AM_I=0x%02X)", addr, who);
        return true;
    }
    return false;
}

static esp_err_t icm_init(void)
{
    ESP_LOGI(TAG, "Scanning for ICM-42670...");
    if (!probe_addr(0x68) && !probe_addr(0x69)) {
        ESP_LOGE(TAG, "ICM-42670 not found");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Soft reset...");
    i2c_write_u8(s_icm_addr, REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t who;
    i2c_read_u8(s_icm_addr, REG_WHO_AM_I, &who);
    if (who != WHOAMI_EXPECT) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: 0x%02X", who);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Enabling accelerometer...");
    i2c_write_u8(s_icm_addr, REG_PWR_MGMT0, 0x03);
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_write_u8(s_icm_addr, REG_ACCEL_CONFIG0, 0x69);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Sensor ready");
    return ESP_OK;
}

// ==================== Inclination Analysis ====================
static inclination_state_t get_x_inclination(int16_t ax)
{
    if (ax > -DEADZONE_LSB && ax < DEADZONE_LSB) {
        return INCL_NEUTRAL;
    } else if (ax < -A_LOT_THRESHOLD) {
        return INCL_A_LOT_RIGHT;
    } else if (ax < -A_BIT_THRESHOLD) {
        return INCL_A_BIT_RIGHT;
    } else if (ax > A_LOT_THRESHOLD) {
        return INCL_A_LOT_LEFT;
    } else if (ax > A_BIT_THRESHOLD) {
        return INCL_A_BIT_LEFT;
    }
    return INCL_NEUTRAL;
}

static inclination_state_t get_y_inclination(int16_t ay)
{
    if (ay > -DEADZONE_LSB && ay < DEADZONE_LSB) {
        return INCL_NEUTRAL;
    } else if (ay > A_LOT_THRESHOLD) {
        return INCL_A_LOT_UP;
    } else if (ay > A_BIT_THRESHOLD) {
        return INCL_A_BIT_UP;
    } else if (ay < -A_LOT_THRESHOLD) {
        return INCL_A_LOT_DOWN;
    } else if (ay < -A_BIT_THRESHOLD) {
        return INCL_A_BIT_DOWN;
    }
    return INCL_NEUTRAL;
}

// ==================== Acceleration Calculation ====================
static int get_acceleration_factor(int64_t incline_duration_ms)
{
    if (incline_duration_ms >= ACCEL_LEVEL_3_MS) {
        return 3;
    } else if (incline_duration_ms >= ACCEL_LEVEL_2_MS) {
        return 2;
    } else {
        return 1;
    }
}

// ==================== Mouse Movement Calculation ====================
static void calculate_mouse_movement(int16_t ax, int16_t ay, int8_t *x_delta, int8_t *y_delta)
{
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds

    // Analyze X-axis inclination
    inclination_state_t x_state = get_x_inclination(ax);
    int base_x_speed = 0;
    
    if (x_state != prev_x_state) {
        // State changed, reset timer
        prev_x_state = x_state;
        x_incl_start_time = current_time;
    }

    // Calculate X movement
    if (x_state != INCL_NEUTRAL) {
        int64_t x_duration = current_time - x_incl_start_time;
        int x_accel = get_acceleration_factor(x_duration);
        
        if (x_state == INCL_A_BIT_LEFT || x_state == INCL_A_BIT_RIGHT) {
            base_x_speed = BASE_SPEED_BIT;
        } else {
            base_x_speed = BASE_SPEED_LOT;
        }
        
        int total_x = x_accel * base_x_speed;
        
        // Apply direction
        if (x_state == INCL_A_BIT_LEFT || x_state == INCL_A_LOT_LEFT) {
            *x_delta = -total_x; // Left is negative
        } else {
            *x_delta = total_x;  // Right is positive
        }
    } else {
        *x_delta = 0;
    }

    // Analyze Y-axis inclination
    inclination_state_t y_state = get_y_inclination(ay);
    int base_y_speed = 0;
    
    if (y_state != prev_y_state) {
        // State changed, reset timer
        prev_y_state = y_state;
        y_incl_start_time = current_time;
    }

    // Calculate Y movement
    if (y_state != INCL_NEUTRAL) {
        int64_t y_duration = current_time - y_incl_start_time;
        int y_accel = get_acceleration_factor(y_duration);
        
        if (y_state == INCL_A_BIT_UP || y_state == INCL_A_BIT_DOWN) {
            base_y_speed = BASE_SPEED_BIT;
        } else {
            base_y_speed = BASE_SPEED_LOT;
        }
        
        int total_y = y_accel * base_y_speed;
        
        // Apply direction (UP is negative, DOWN is positive for mouse coordinates)
        if (y_state == INCL_A_BIT_UP || y_state == INCL_A_LOT_UP) {
            *y_delta = -total_y;
        } else {
            *y_delta = total_y;
        }
    } else {
        *y_delta = 0;
    }
}

// ==================== BLE HID Callbacks ====================
static uint8_t hidd_service_uuid128[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x03c2,       // HID Mouse
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH:
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
            }
            break;
        case ESP_HIDD_EVENT_BLE_CONNECT:
            ESP_LOGI(TAG, "BLE HID Connected");
            hid_conn_id = param->connect.conn_id;
            break;
        case ESP_HIDD_EVENT_BLE_DISCONNECT:
            sec_conn = false;
            ESP_LOGI(TAG, "BLE HID Disconnected");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            sec_conn = true;
            ESP_LOGI(TAG, "BLE pairing successful");
        } else {
            ESP_LOGE(TAG, "Pairing failed, reason = 0x%x",
                     param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

// ==================== Main Mouse Control Task ====================
void imu_mouse_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting IMU mouse control task");
    
    // Warm up sensor
    for (int i = 0; i < 10; i++) {
        int16_t ax, ay, az;
        read_accel(&ax, &ay, &az);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "IMU ready. Waiting for BLE connection...");
    
    int error_count = 0;

    while (1) {
        if (sec_conn) {
            int16_t ax = 0, ay = 0, az = 0;

            if (read_accel(&ax, &ay, &az) == ESP_OK) {
                error_count = 0;
                
                int8_t x_delta = 0, y_delta = 0;
                calculate_mouse_movement(ax, ay, &x_delta, &y_delta);

                // Read button state
                uint8_t buttons = 0;
                if (gpio_get_level(BUTTON_GPIO) == BUTTON_ACTIVE_LEVEL) {
                    buttons = MOUSE_LEFT_BUTTON;
                }

                // Send mouse movement (always send to update button state)
                esp_hidd_send_mouse_value(hid_conn_id, buttons, x_delta, y_delta);
            } else {
                if ((++error_count % 10) == 1) {
                    ESP_LOGW(TAG, "Sensor read error (count=%d)", error_count);
                }
                if (error_count > 50) {
                    ESP_LOGW(TAG, "Reinitializing sensor");
                    icm_init();
                    error_count = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL_MS));
    }
}

// ==================== Main Application ====================
void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "IMU-Controlled BLE Mouse Starting...");
    ESP_LOGI(TAG, "I2C: SDA=GPIO%d, SCL=GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");

    // Initialize Button
    ESP_ERROR_CHECK(button_init());

    // Initialize IMU
    if (icm_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed");
        return;
    }

    // Initialize Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT controller init failed");
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT controller enable failed");
        return;
    }

    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed");
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed");
        return;
    }

    if ((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(TAG, "HID profile init failed");
        return;
    }

    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    // Set BLE security parameters
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    ESP_LOGI(TAG, "System initialized. Connect via Bluetooth to '%s'", HIDD_DEVICE_NAME);
    
    // Start mouse control task
    xTaskCreate(&imu_mouse_task, "imu_mouse_task", 4096, NULL, 5, NULL);
}
