
/*
Some info needed
***************************************************************************
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "led_strip.h"
#include "driver/gpio.h"

#define BOOT_PIN GPIO_NUM_9
#define INVALID_CONN_ID 0xFFFF
static const char* DEMO_TAG = "RAMPA";
led_strip_handle_t led;
led_strip_config_t strip_config = {.strip_gpio_num = GPIO_NUM_8, .max_leds = 1,};
led_strip_rmt_config_t rmt_config = {.resolution_hz = 10 * 1000 * 1000,};

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

static void gattc_cb(
    esp_gattc_cb_event_t event,
    esp_gatt_if_t gattc_if,
    esp_ble_gattc_cb_param_t *param);

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = ESP_BLE_GAP_SCAN_ITVL_MS(50),
    .scan_window            = ESP_BLE_GAP_SCAN_WIN_MS(30),
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min            = 0x20,
    .adv_int_max            = 0x40,
    .adv_type               = ADV_TYPE_IND,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .channel_map            = ADV_CHNL_ALL,
    .adv_filter_policy      = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// static uint8_t last_btn = 1;

const uint8_t elite_uuid[] = {
    0x92, 0xE5, 0x9C, 0x94,
    0xF3, 0x8F,
    0x18, 0x89,
    0x8B, 0x40,
    0x35, 0x76,
    0x01, 0x00, 0x7B, 0x34
};

static bool scan_stopped = false;
static bool elite_ready = false;
static bool ftms_cp_indications_enabled = false;
//----------------------------------------------------------------

enum { IDX_SVC, IDX_CHAR_FEATURE, IDX_CHAR_VAL_FEATURE, 
        IDX_CHAR_STATUS, IDX_CHAR_VAL_STATUS, IDX_CHAR_CFG_STATUS,
        IDX_CHAR_TRAINING, IDX_CHAR_VAL_TRAINING, IDX_CHAR_CFG_TRAINING,
        IDX_CHAR_SPEED_RANGE, IDX_CHAR_VAL_SPEED_RANGE,
        IDX_CHAR_INCLINE_RANGE, IDX_CHAR_VAL_INCLINE_RANGE,
        IDX_CHAR_RESIST_RANGE, IDX_CHAR_VAL_RESIST_RANGE,
        IDX_CHAR_POWER_RANGE, IDX_CHAR_VAL_POWER_RANGE,
        IDX_CHAR_BIKE_DATA, IDX_CHAR_VAL_BIKE_DATA, IDX_CHAR_CFG_BIKE_DATA,
        IDX_CHAR_CP, IDX_CHAR_VAL_CP, IDX_CHAR_CFG_CP, FTMS_IDX_NB
        // HR_IDX_SVC, HR_IDX_CHAR, HR_IDX_VAL, HR_IDX_CFG, HR_IDX_NB
     };

enum { HR_IDX_SVC, HR_IDX_CHAR, HR_IDX_VAL, HR_IDX_CFG, HR_IDX_NB };

static const uint16_t FTMS_SERVICE_UUID             = 0x1826;
static const uint16_t FTMS_FEATURE_UUID             = 0x2ACC;
static const uint16_t FTMS_INDOOR_BIKE_UUID         = 0x2AD2;
static const uint16_t FTMS_CONTROL_POINT_UUID       = 0x2AD9;
static const uint16_t FTMS_STATUS_UUID              = 0x2ADA;
static const uint16_t FTMS_TRAINING_STATUS_UUID     = 0x2AD3;
static const uint16_t FTMS_SPEED_RANGE_UUID         = 0x2AD4;
static const uint16_t FTMS_INCLINE_RANGE_UUID       = 0x2AD5;
static const uint16_t FTMS_RESISTANCE_RANGE_UUID    = 0x2AD6;
static const uint16_t FTMS_POWER_RANGE_UUID         = 0x2AD8;
static const uint16_t HR_SERVICE_UUID      = 0x180D;
static const uint16_t HR_MEASUREMENT_UUID  = 0x2A37;

static const uint8_t char_prop_read                 = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_notify               = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint16_t primary_service_uuid          = ESP_GATT_UUID_PRI_SERVICE;
// static const uint16_t secondary_service_uuid        = ESP_GATT_UUID_SEC_SERVICE;
static const uint16_t character_declaration_uuid    = ESP_GATT_UUID_CHAR_DECLARE;

static uint16_t ftms_handle_table[FTMS_IDX_NB];
static uint16_t hr_handle_table[HR_IDX_NB];

static uint8_t training_status[] = {
    0x00,
    0x01      // Idle
};

static uint8_t machine_status[] = {
    0x04
};

static const uint8_t speed_range[] = {
    0x3C, 0x00,
    0x08, 0x07,
    0x0A, 0x00
};

static const uint8_t incline_range[] = {
    0x00, 0x00,
    0x96, 0x00,
    0x0A, 0x00
};

static const uint8_t resistance_range[] = {
    0x0A, 0x00,
    0x96, 0x00,
    0x0A, 0x00
};

static const uint8_t power_range[] =
{
    0x00, 0x00,     // min = 0 W
    0xD0, 0x07,     // max = 2000 W
    0x01, 0x00      // step = 1 W
};

// static const uint8_t feature_value[8] =
// {
//     0x82, 0x40, 0x00, 0x00,
//     0x0c, 0x00, 0x00, 0x00
// };

static const uint8_t feature_value[8] =
{
    // Fitness Machine Features
    0x82, 0x40, 0x00, 0x00,

    // Target Setting Features
    // bit 2  = resistance
    // bit 3  = power
    // bit 13 = indoor bike simulation
    0x0C, 0x20, 0x00, 0x00
};

static uint8_t bike_data[12] =
{
    0x24, 0x02,  
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00
};

// static uint8_t adv_service_uuid[16] = {
//     0xFB, 0x34, 0x9B, 0x5F,
//     0x80, 0x00,
//     0x00, 0x80,
//     0x00, 0x10,
//     0x00, 0x00,
//     0x26, 0x18,
//     0x00, 0x00
// };

static uint8_t hr_measurement[2] = {
    0x00, // flags
    75    // bpm
};

static uint8_t status_ccc[2] = {0, 0};
static uint8_t training_ccc[2] = {0, 0};
static const uint8_t cp_ccc[2] = {0, 0};
static uint8_t bike_ccc[2] = {0, 0};
static uint8_t hr_ccc[2] = {0,0};

static const uint8_t char_prop_read_notify =
    ESP_GATT_CHAR_PROP_BIT_READ |
    ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t char_prop_notify_only = 
    ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_write_indicate = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_INDICATE;
static uint8_t cp_value[8] = {0, 0, 0, 0, 0, 0, 0, 0};


// static uint8_t byte_1 = 0xc9;
// static uint8_t byte_2 = 0;
volatile uint8_t blue = 0;
volatile uint8_t red = 50;
volatile uint8_t green = 0;

static bool xcadey_open_started = false;
static bool polar_open_started = false;
static uint16_t rampa_service_start = 0;
static uint16_t rampa_service_end = 0;
static uint16_t xcadey_service_start = 0;
static uint16_t xcadey_service_end = 0;
static uint16_t polar_service_start = 0;
static uint16_t polar_service_end = 0;

// static bool is_xcadey_notify = false;
// static bool is_polar_notify = false;

static uint16_t h_347b0010 = 0;
static uint16_t h_347b0011 = 0;

static uint16_t xcadey_power_handle = 0;
static uint16_t polar_hr_handle = 0;
static uint16_t xcadey_cccd_handle = 0;
// static uint16_t xcadey_cp_handle = 0;

static uint16_t rampa_conn_id = INVALID_CONN_ID;
static uint16_t xcadey_conn_id = INVALID_CONN_ID;
static uint16_t polar_conn_id = INVALID_CONN_ID;

static esp_gatt_if_t gattc_if_global = 0;

static bool ftms_bike_notify_enabled = false;
static bool ftms_training_notify_enabled = false;
static bool ftms_status_notify_enabled = false;
// static bool ftms_cp_indications_enabled = false;

static bool found_rampa = false;
static bool rampa_open_started = false;
static bool found_xcadey = false;
static bool found_polar = false;
static bool ftms_started = false;
// static bool polar_open_started = false;

// static bool hr_notify_enabled = false;

static bool rampa_connected = false;
static bool xcadey_connected = false;
static bool polar_connected = false;

static esp_bd_addr_t rampa_addr;
static esp_bd_addr_t xcadey_addr;
static esp_bd_addr_t polar_addr;

static uint8_t rampa_addr_type;
static uint8_t xcadey_addr_type;
static uint8_t polar_addr_type;

static uint16_t ftms_conn_id = INVALID_CONN_ID;
static esp_gatt_if_t ftms_gatts_if = 0;
static bool ftms_connected = false;

static uint16_t polar_hr = 0;

static const esp_gatts_attr_db_t ftms_gatt_db[FTMS_IDX_NB] = {
    [IDX_SVC] =
    {{
        ESP_GATT_AUTO_RSP
    },{
        ESP_UUID_LEN_16,
        (uint8_t *)&primary_service_uuid,
        ESP_GATT_PERM_READ,
        sizeof(uint16_t),
        sizeof(FTMS_SERVICE_UUID),
        (uint8_t *)&FTMS_SERVICE_UUID
    }},

    [IDX_CHAR_FEATURE] =
    {{
        ESP_GATT_AUTO_RSP
    },{
        ESP_UUID_LEN_16,
        (uint8_t *)&character_declaration_uuid,
        ESP_GATT_PERM_READ,
        sizeof(uint8_t),
        sizeof(uint8_t),
        (uint8_t *)&char_prop_read
    }},

    [IDX_CHAR_VAL_FEATURE] =
    {{
        ESP_GATT_AUTO_RSP
    },{
        ESP_UUID_LEN_16,
        (uint8_t *)&FTMS_FEATURE_UUID,
        ESP_GATT_PERM_READ,
        sizeof(feature_value),
        sizeof(feature_value),
        (uint8_t *)feature_value
    }},
[IDX_CHAR_STATUS] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_read_notify
}},

[IDX_CHAR_VAL_STATUS] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&FTMS_STATUS_UUID,
    ESP_GATT_PERM_READ,
    sizeof(machine_status),
    sizeof(machine_status),
    machine_status
}},
[IDX_CHAR_CFG_STATUS] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_client_config_uuid,
    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),
    sizeof(status_ccc),status_ccc
}},

[IDX_CHAR_TRAINING] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_read_notify
}},

[IDX_CHAR_VAL_TRAINING] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&FTMS_TRAINING_STATUS_UUID,
    ESP_GATT_PERM_READ,
    sizeof(training_status),
    sizeof(training_status), training_status
}},

[IDX_CHAR_CFG_TRAINING] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_client_config_uuid,
    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),
    sizeof(training_ccc), training_ccc
}},

[IDX_CHAR_SPEED_RANGE] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_read
}},

[IDX_CHAR_VAL_SPEED_RANGE] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&FTMS_SPEED_RANGE_UUID,
    ESP_GATT_PERM_READ,
    sizeof(speed_range),
    sizeof(speed_range),
    (uint8_t *)speed_range
}},

[IDX_CHAR_INCLINE_RANGE] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_read
}},

[IDX_CHAR_VAL_INCLINE_RANGE] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&FTMS_INCLINE_RANGE_UUID,
    ESP_GATT_PERM_READ,
    sizeof(incline_range),
    sizeof(incline_range),
    (uint8_t *)incline_range
}},

[IDX_CHAR_RESIST_RANGE] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_read
}},

[IDX_CHAR_VAL_RESIST_RANGE] =
{{
   ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&FTMS_RESISTANCE_RANGE_UUID,
    ESP_GATT_PERM_READ,
    sizeof(resistance_range),
    sizeof(resistance_range),
    (uint8_t *)resistance_range
}},

[IDX_CHAR_POWER_RANGE] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_read
}},

[IDX_CHAR_VAL_POWER_RANGE] =
{{
   ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&FTMS_POWER_RANGE_UUID,
    ESP_GATT_PERM_READ,
    sizeof(power_range),
    sizeof(power_range),
    (uint8_t *)power_range
}},

[IDX_CHAR_BIKE_DATA] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_notify
}},

[IDX_CHAR_VAL_BIKE_DATA] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&FTMS_INDOOR_BIKE_UUID,
    ESP_GATT_PERM_READ,
    sizeof(bike_data),
    sizeof(bike_data),
    bike_data
}},

[IDX_CHAR_CFG_BIKE_DATA] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_client_config_uuid,
    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),
    sizeof(bike_ccc),
    bike_ccc
}},

[IDX_CHAR_CP] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_write_indicate
}},

[IDX_CHAR_VAL_CP] =
{{
    ESP_GATT_RSP_BY_APP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&FTMS_CONTROL_POINT_UUID,
    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(cp_value),
    sizeof(cp_value),
    cp_value
}},

[IDX_CHAR_CFG_CP] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_client_config_uuid,
    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),
    sizeof(cp_ccc),
    (uint8_t *)cp_ccc
}},    

};


static const esp_gatts_attr_db_t hr_gatt_db[HR_IDX_NB] = {

[HR_IDX_SVC] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&primary_service_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint16_t),
    sizeof(HR_SERVICE_UUID),
    (uint8_t *)&HR_SERVICE_UUID
}},

[HR_IDX_CHAR] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_declaration_uuid,
    ESP_GATT_PERM_READ,
    sizeof(uint8_t),
    sizeof(uint8_t),
    (uint8_t *)&char_prop_notify_only
}},

[HR_IDX_VAL] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&HR_MEASUREMENT_UUID,
    ESP_GATT_PERM_READ,
    sizeof(hr_measurement),
    sizeof(hr_measurement),
    hr_measurement
}},

[HR_IDX_CFG] =
{{
    ESP_GATT_AUTO_RSP
},{
    ESP_UUID_LEN_16,
    (uint8_t *)&character_client_config_uuid,
    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),
    sizeof(hr_ccc),
    hr_ccc
}},
};

// static esp_ble_adv_data_t adv_data = {
//     .set_scan_rsp = false,
//     .include_name = false,
//     .include_txpower = false,
//     .service_uuid_len = 16, //sizeof(adv_service_uuid), // sizeof(FTMS_SERVICE_UUID),
//     .p_service_uuid = adv_service_uuid,
//     .flag = ESP_BLE_ADV_FLAG_GEN_DISC |
//             ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
// };

static uint8_t raw_adv_data[] = {

    // Flags
    0x02,               // długość
    0x01,               // Flags
    0x06,               // General Discoverable + BR/EDR not supported

    // Complete List of 16-bit Service UUIDs
    0x03,               // długość: type + 2 bajty UUID
    0x03,               // Complete List of 16-bit Service Class UUIDs
    0x26, 0x18,         // FTMS = 0x1826

    // Complete Local Name "ESP32-FTMS"
    0x0B,               // 1 byte type + 10 znaków
    0x09,               // Complete Local Name
    'E','S','P','3','2','-','F','T','M','S'
};

typedef struct {
    uint16_t power;
    uint16_t cadence;
    TickType_t timestamp;
} trainer_data_t;

volatile trainer_data_t trainer;

void LEDTask(void *arg) {
    while (1) {
        led_strip_set_pixel(led, 0, red, green, blue);
        led_strip_refresh(led);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void elite_send(uint8_t b1, uint8_t b2) {
    
    uint8_t data[3] = {0x0, b1, b2};
    
    if (!elite_ready)
        return;

    esp_ble_gattc_write_char(
        gattc_if_global,
        rampa_conn_id,
        h_347b0010,
        sizeof(data),
        data,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE);
}

void rouvy_send(uint16_t xpower, uint16_t xcadence)
{
    if (!ftms_connected)
        return;

    if (!ftms_started)
        return;

    if (!ftms_bike_notify_enabled)
        return;

    bike_data[0] = 0x64;
    bike_data[1] = 0x02;

    xcadence *= 2;

    bike_data[4] = xcadence & 0xff;
    bike_data[5] = xcadence >> 8;

    bike_data[8] = xpower & 0xff;
    bike_data[9] = xpower >> 8;

    esp_err_t err = esp_ble_gatts_send_indicate(
        ftms_gatts_if,
        ftms_conn_id,
        ftms_handle_table[IDX_CHAR_VAL_BIKE_DATA],
        sizeof(bike_data),
        bike_data,
        false
    );

    if (err != ESP_OK) {
        printf("BIKE notify err=%d conn=%u if=%u started=%d notify=%d\n",
               err,
               ftms_conn_id,
               ftms_gatts_if,
               ftms_started,
               ftms_bike_notify_enabled);
    }
}



// void rouvy_send(uint16_t xpower, uint16_t xcadence) {


//     if (!ftms_connected)
//         return;
//     bike_data[0] = 0x64;
//     bike_data[1] = 0x02;
//     xcadence *= 2;
//     bike_data[4] = xcadence & 0xff;
//     bike_data[5] = xcadence >> 8;
//     bike_data[8] = xpower & 0xff;
//     bike_data[9] = xpower >> 8;

//     //esp_err_t err = 
//     esp_ble_gatts_send_indicate(
//         ftms_gatts_if, 
//         ftms_conn_id, 
//         ftms_handle_table[IDX_CHAR_VAL_BIKE_DATA], 
//         sizeof(bike_data), 
//         bike_data, 
//         false);
//     training_status[0] = 0x00;
//     training_status[1] = 0x0c;   // Running

//     esp_ble_gatts_send_indicate(
//         ftms_gatts_if,
//         ftms_conn_id,
//         ftms_handle_table[IDX_CHAR_VAL_TRAINING],
//         sizeof(training_status),
//         training_status,
//         false);

//     hr_measurement[1] = polar_hr;

//     // printf("FTMS HR=%u\n", polar_hr);

//     if(hr_notify_enabled) {
//         esp_ble_gatts_send_indicate(
//             ftms_gatts_if,
//             ftms_conn_id,
//             hr_handle_table[HR_IDX_VAL],
//             sizeof(hr_measurement),
//             hr_measurement,
//             false);
//     }
// }

void FTMSTask(void *) {
    while(1) {
        TickType_t age = xTaskGetTickCount() - trainer.timestamp;

        uint16_t power;
        uint16_t cadence;

        if(age > pdMS_TO_TICKS(2500)) {
            power = 0;
            cadence = 0;
        }
        else {
            power = trainer.power;
            cadence = trainer.cadence;
        }
        rouvy_send(power, cadence);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

}

void ScanTimeoutTask(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(60000));
    if (!scan_stopped) {
        scan_stopped = true;
        printf("Scan timeout\n");
        esp_ble_gap_stop_scanning();
    }
    vTaskDelete(NULL);
}

void esp_eddystone_appRegister(void) {
    
    esp_err_t status;

    // ESP_LOGI(DEMO_TAG,"Register callback");
    /*<! register the scan callback function to the gap module */
    if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG,"gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void esp_eddystone_init(void) {
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    esp_bluedroid_init_with_cfg(&cfg);
    esp_bluedroid_enable();
    esp_eddystone_appRegister();
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {

    // esp_err_t err;
    uint8_t len = 0;

    switch(event) {
case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:

    printf("ADV RAW configured status=%d\n",
           param->adv_data_raw_cmpl.status);

    esp_err_t err =
        esp_ble_gap_start_advertising(&adv_params);

    printf("Advertising start: %s\n",
           esp_err_to_name(err));

break;
        
        // case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        //     printf("ADV configured\n");
        //     esp_ble_gap_start_advertising(&adv_params);
        // break;        
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: 
            // printf("scan param set \n");
            // the unit of the duration is second, 0 means scan permanently
            esp_ble_gap_start_scanning(0);
            scan_stopped = false;
            xTaskCreate(ScanTimeoutTask, "scan_timeout", 2048, NULL, 5, NULL);
        break;        
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: 
            err = param->scan_start_cmpl.status;
            printf("Scan started %s\n", esp_err_to_name(err));
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) 
                ESP_LOGE(DEMO_TAG,"Scan start failed: %s", esp_err_to_name(err));
            // else             
            //     ESP_LOGI(DEMO_TAG,"Start scanning...");
        break;        
        case ESP_GAP_BLE_SCAN_RESULT_EVT: 
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            switch(scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: 
                    uint8_t *name = esp_ble_resolve_adv_data(
                                    scan_result->scan_rst.ble_adv,
                                    ESP_BLE_AD_TYPE_NAME_CMPL, &len);
                    char devname[32] = {0};
                    if(name && len > 0) {
                        if(len > sizeof(devname)-1)
                            len = sizeof(devname)-1;
                        memcpy(devname, name, len);
                        devname[len] = 0;
                        // printf("FOUND: %s\n", devname);
                    }
                  
                    if (name && !found_rampa && len == 2 && name[0] == 'R' && name[1] == 'M') {
                        // printf("Rampa found!\n");
                        memcpy(rampa_addr, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                        rampa_addr_type = scan_result->scan_rst.ble_addr_type;
                        found_rampa = true;
                    }
                    if (!found_xcadey && strstr(devname, "XPOWER-L")) {
                        // printf("XCadey found!\n");
                        memcpy(xcadey_addr, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                        xcadey_addr_type = scan_result->scan_rst.ble_addr_type;
                        found_xcadey = true;
                    }
                    if (!found_polar && strstr(devname, "Polar H10")) {
                        memcpy(polar_addr, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                        polar_addr_type = scan_result->scan_rst.ble_addr_type;
                        found_polar = true;     
                        printf("Polar found\n");                  
                    }
                if (!scan_stopped && found_rampa && found_xcadey && found_polar) {
                        scan_stopped = true;
                        printf("Devices found\n");
                        esp_ble_gap_stop_scanning();
                }                
                break;                                
                default:
                ;
                break;
            }
        break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
if (found_rampa && !rampa_open_started) {
    rampa_open_started = true;

    printf("Connecting Rampa...\n");

    esp_ble_gattc_open(
        gattc_if_global,
        rampa_addr,
        rampa_addr_type,
        true
    );
}


//             if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) 
//                 ESP_LOGE(DEMO_TAG,"Scan stop failed: %s", esp_err_to_name(err));
//             else 
//                 ESP_LOGI(DEMO_TAG,"Stop scan successfully");
// /*polar connection*/
//             if(found_polar && !polar_open_started && !polar_connected) {
//                 polar_open_started = true;
//                 uint8_t status = esp_ble_gattc_open(gattc_if_global, polar_addr, polar_addr_type, true);
//                 printf("Connecting Polar... status=%d\n", status);             
//             }
            
//             printf("Scan stopped\n");

//             if(found_rampa) {
//                 printf("Connecting Rampa...\n");
//                 esp_ble_gattc_open(gattc_if_global, rampa_addr, rampa_addr_type, true);
//             }
        break;      
        default:
            ;
        break;
    }
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {

    switch(event) {
        case ESP_GATTC_OPEN_EVT:
        printf("OPEN status=%d conn_id=%u\n", param->open.status, param->open.conn_id);
            if (param->open.status == ESP_GATT_OK) {
                printf("OPEN OK conn_id=%u\n", param->open.conn_id);
                if(memcmp(param->open.remote_bda, rampa_addr, 6) == 0) {
                    rampa_conn_id = param->open.conn_id;
                    rampa_connected = true;
                    printf("RAMPA CONNECTED\n");
                }
                if(memcmp(param->open.remote_bda, xcadey_addr, 6) == 0) {
                    xcadey_conn_id = param->open.conn_id;
                    xcadey_connected = true;
                    printf("XCADEY CONNECTED\n");
                }
                if(memcmp(param->open.remote_bda, polar_addr, 6) == 0) {
                    polar_conn_id = param->open.conn_id;
                    polar_connected = true;
                    printf("POLAR CONNECTED %u\n", param->open.conn_id);
                }
                uint16_t conn_id = param->open.conn_id;
                esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
                gattc_if_global = gattc_if;
            }
        break;
case ESP_GATTC_REG_FOR_NOTIFY_EVT: {

    printf("REG_FOR_NOTIFY status=%d handle=%u\n",
           param->reg_for_notify.status,
           param->reg_for_notify.handle);

    if (param->reg_for_notify.status != ESP_GATT_OK) {
        break;
    }

    uint16_t count = 0;
    uint16_t char_handle = param->reg_for_notify.handle;
    uint16_t conn_id;
    uint16_t start_handle;
    uint16_t end_handle;

    bool is_xcadey = false;
    bool is_polar  = false;

    if (char_handle == xcadey_power_handle) {

        conn_id      = xcadey_conn_id;
        start_handle = xcadey_service_start;
        end_handle   = xcadey_service_end;
        is_xcadey    = true;
    }
    else if (char_handle == polar_hr_handle) {

        conn_id      = polar_conn_id;
        start_handle = polar_service_start;
        end_handle   = polar_service_end;
        is_polar     = true;
    }
    else {
        break;
    }

    esp_gatt_status_t st = esp_ble_gattc_get_attr_count(
        gattc_if,
        conn_id,
        ESP_GATT_DB_DESCRIPTOR,
        start_handle,
        end_handle,
        char_handle,
        &count
    );

    if (st != ESP_GATT_OK || count == 0) {
        printf("No descriptors for handle=%u status=%d count=%u\n",
               char_handle, st, count);
        break;
    }

    esp_gattc_descr_elem_t *descr =
        malloc(sizeof(esp_gattc_descr_elem_t) * count);

    if (!descr) {
        break;
    }

    st = esp_ble_gattc_get_all_descr(
        gattc_if,
        conn_id,
        char_handle,
        descr,
        &count,
        0
    );

    if (st != ESP_GATT_OK) {
        free(descr);
        break;
    }

    for (int i = 0; i < count; i++) {

        if (descr[i].uuid.len != ESP_UUID_LEN_16) {
            continue;
        }

        if (descr[i].uuid.uuid.uuid16 ==
            ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {

            printf("CCCD handle=%u conn=%u\n",
                   descr[i].handle,
                   conn_id);

            /*
             * Zapamiętujemy CCCD XCadey.
             * Po WRITE_DESCR tego konkretnego CCCD
             * uruchomimy Polara.
             */
            if (is_xcadey) {
                xcadey_cccd_handle = descr[i].handle;
                printf("XCADEY CCCD=%u\n",
                       xcadey_cccd_handle);
            }

            uint16_t notify_en = 1;

            esp_err_t err = esp_ble_gattc_write_char_descr(
                gattc_if,
                conn_id,
                descr[i].handle,
                sizeof(notify_en),
                (uint8_t *)&notify_en,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE
            );

            printf("%s CCCD write err=%d\n",
                   is_xcadey ? "XCADEY" :
                   is_polar  ? "POLAR" : "?",
                   err);

            break;
        }
    }

    free(descr);

    break;
}                               
        break;
        case ESP_GATTC_SEARCH_CMPL_EVT: {
            // printf("SEARCH COMPLETE status=%d conn_id=%u\n", param->search_cmpl.status, param->search_cmpl.conn_id);
            uint16_t current_conn = param->search_cmpl.conn_id;
            uint16_t start_handle = 0;
            uint16_t end_handle   = 0;
            bool is_rampa = false;
            bool is_polar = false;
            if (current_conn == rampa_conn_id) {
                start_handle = rampa_service_start;
                end_handle   = rampa_service_end;
                is_rampa = true;
                printf("SEARCH COMPLETE: RAMPA\n");
            }
            else if (current_conn == xcadey_conn_id) {
                start_handle = xcadey_service_start;
                end_handle   = xcadey_service_end;
                printf("SEARCH COMPLETE: XCADEY\n");                                  
            }
            else if (current_conn == polar_conn_id) {
                start_handle = polar_service_start;
                end_handle   = polar_service_end;
                is_polar = true;
                printf("SEARCH COMPLETE: POLAR\n");
            }                      
            else {
                printf("Unknown conn_id=%u\n", current_conn);
                break;
            }
            uint16_t count = 0;

            // esp_gatt_status_t status =
            esp_ble_gattc_get_attr_count(
                gattc_if,
                current_conn,
                ESP_GATT_DB_CHARACTERISTIC,
                start_handle,
                end_handle,
                0,
                &count);
            // printf("exposed=%d status=%d\n", count, status);
            if (count == 0)
                break;
            esp_gattc_char_elem_t *chars = malloc(sizeof(esp_gattc_char_elem_t) * count);
            if (!chars)
                break;
            esp_ble_gattc_get_all_char(gattc_if, current_conn, start_handle, end_handle, chars, &count,0);
            for (int i = 0; i < count; i++) {
                if (is_rampa) {
                    if (chars[i].uuid.len == ESP_UUID_LEN_128) {
                        uint8_t *u = chars[i].uuid.uuid.uuid128;

                        if (u[13] == 0x00 && u[12] == 0x10) {
                            h_347b0010 = chars[i].char_handle;
                            printf("347B0010 handle=%u\n", h_347b0010);
                        }
                        if (u[13] == 0x00 && u[12] == 0x11) {
                            h_347b0011 = chars[i].char_handle;
                            printf("347B0011 handle=%u\n", h_347b0011);
                        }
                    }
                }
                else if (is_polar) {
                        if (chars[i].uuid.len == ESP_UUID_LEN_16) {
                            uint16_t uuid16 = chars[i].uuid.uuid.uuid16;
                            // printf("Polar CHAR UUID=%04X handle=%u\n", uuid16, chars[i].char_handle);
                            if (uuid16 == 0x2A37) {
                                polar_hr_handle = chars[i].char_handle;
                                printf("POLAR handle=%u\n", polar_hr_handle);
                                // esp_err_t err = 
                                esp_ble_gattc_register_for_notify(
                                        gattc_if,
                                        polar_addr,
                                        polar_hr_handle);
                                // printf("register notify err=%d\n", err);
                        }                        
                    }
                }
                else {
                    if (chars[i].uuid.len == ESP_UUID_LEN_16) {
                        uint16_t uuid16 = chars[i].uuid.uuid.uuid16;
                        // printf("XCADEY CHAR UUID=%04X handle=%u\n", uuid16, chars[i].char_handle);
                        if (uuid16 == 0x2A63) {
                            xcadey_power_handle = chars[i].char_handle;
                            printf("CYCLING POWER handle=%u\n", xcadey_power_handle);
                             // esp_err_t err = 
                             esp_ble_gattc_register_for_notify(
                                    gattc_if,
                                    xcadey_addr,
                                    xcadey_power_handle);
                            // printf("register notify err=%d\n", err);
                        }                        
                    }
                }
            }
            if (is_rampa && h_347b0010 && h_347b0011) {          //&& is_polar    
                elite_ready = true;    
                green = 50;
                red = 0;
                elite_send(0x50, 0x00);
                if(found_xcadey && !xcadey_open_started) {
                    xcadey_open_started = true;
                    esp_ble_gattc_open(gattc_if_global, xcadey_addr, xcadey_addr_type, true);
                }  
            }
            free(chars);
        }
        break;
        case ESP_GATTC_NOTIFY_EVT:
            // printf("notify, handle %d\n",param->notify.handle);
            if(param->notify.handle == polar_hr_handle) {
                uint8_t flags = param->notify.value[0];
                if(flags & 0x01) {
                    polar_hr = param->notify.value[1] | (param->notify.value[2] << 8);                    
                }
                else {
                    polar_hr = param->notify.value[1];
                }
                printf("HR=%u\r", polar_hr);
                hr_measurement[1] = polar_hr;
            }
            if (param->notify.handle == xcadey_power_handle) {

                static uint16_t xcadey_cadence = 0;
                static uint16_t xcadey_power = 0;
                static uint16_t last_rev = 0;
                static uint16_t last_evt = 0;

                xcadey_power = param->notify.value[2] | (param->notify.value[3] << 8);
                printf("XCADEY POWER=%u\n", xcadey_power);

                uint16_t rev = param->notify.value[7] | (param->notify.value[8] << 8);                
                uint16_t evt = param->notify.value[9] | (param->notify.value[10] << 8);

                if (last_evt) {
                    uint16_t d_rev = rev - last_rev;
                    uint16_t d_evt = evt - last_evt;
                    if (d_evt) { //(d_rev > 0 && d_evt > 0) {
                        float cadence =(float)d_rev * 60.0f * 1024.0f / (float)d_evt;
                            xcadey_cadence = (uint16_t)(cadence + 0.5f);
                            printf("\ncadence %d\n", xcadey_cadence);
                    }
                }
                last_rev = rev;
                last_evt = evt;
                trainer.power = xcadey_power;
                trainer.cadence = xcadey_cadence;
                trainer.timestamp = xTaskGetTickCount();
            }
        break;

case ESP_GATTC_WRITE_DESCR_EVT: {

    printf("WRITE_DESCR status=%d conn=%u handle=%u\n",
           param->write.status,
           param->write.conn_id,
           param->write.handle);

    if (param->write.status != ESP_GATT_OK) {
        printf("WRITE_DESCR failed\n");
        break;
    }

    /*
     * XCadey ma już działające notifications.
     * Dopiero teraz zaczynamy połączenie z Polarem.
     */
    if (param->write.conn_id == xcadey_conn_id &&
        param->write.handle == xcadey_cccd_handle) {

        printf("XCADEY READY\n");

        // if (found_polar &&
        //     !polar_open_started &&
        //     !polar_connected) {

        //     polar_open_started = true;

        //     printf("Connecting Polar...\n");

        //     esp_err_t err = esp_ble_gattc_open(
        //         gattc_if_global,
        //         polar_addr,
        //         polar_addr_type,
        //         true
        //     );

        //     printf("Connecting Polar... status=%d\n", err);

        //     if (err != ESP_OK) {
        //         polar_open_started = false;
        //     }
        // }
    }

    /*
     * Dla Polara nic więcej nie otwieramy.
     * Tylko informacyjnie potwierdzamy CCCD.
     */
    if (param->write.conn_id == polar_conn_id) {
        printf("POLAR CCCD READY\n");
    }

    break;
}
case ESP_GATTC_DISCONNECT_EVT:

    printf("GATTC DISCONNECT conn=%u reason=%u\n",
           param->disconnect.conn_id,
           param->disconnect.reason);

    if (rampa_connected &&
        param->disconnect.conn_id == rampa_conn_id) {

        printf("RAMPA DISCONNECTED\n");

        rampa_connected = false;
        rampa_open_started = false;

        elite_ready = false;

        h_347b0010 = 0;
        h_347b0011 = 0;

        rampa_conn_id = INVALID_CONN_ID;

        red = 50;
        green = 0;
        blue = 0;
    }

    else if (xcadey_connected &&
             param->disconnect.conn_id == xcadey_conn_id) {

        printf("XCADEY DISCONNECTED\n");

        xcadey_connected = false;
        xcadey_open_started = false;

        xcadey_power_handle = 0;
        xcadey_cccd_handle = 0;
        xcadey_conn_id = INVALID_CONN_ID;
    }

    else if (polar_connected &&
             param->disconnect.conn_id == polar_conn_id) {

        printf("POLAR DISCONNECTED\n");

        polar_connected = false;
        polar_open_started = false;

        polar_hr_handle = 0;
        polar_hr = 0;

        polar_conn_id = INVALID_CONN_ID;
    }

    else {

        /*
         * To może być np. połączenie MyWhoosh/Windows.
         * GATTC callback dostaje również zdarzenia
         * dla fizycznych linków używanych przez GATTS.
         */
        printf("GATTC DISCONNECT ignored - unknown/non-client conn\n");
    }

    break;     

        case ESP_GATTC_REG_EVT:
            printf("REGISTER OK IF=%d\n", gattc_if);
            gattc_if_global = gattc_if;
        break;            
        case ESP_GATTC_CONNECT_EVT:
            printf("GATTC CONNECTED!\n");        
        break;
        case ESP_GATTC_SEARCH_RES_EVT:
            // printf("SEARCH RESULT conn_id=%u\n", param->search_res.conn_id);
            if(param->search_res.conn_id == rampa_conn_id)
            {
                if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
                    uint8_t *u = param->search_res.srvc_id.uuid.uuid.uuid128;
                    if(memcmp(u, elite_uuid, 16) == 0) {
                        rampa_service_start = param->search_res.start_handle;
                        rampa_service_end = param->search_res.end_handle;
                        printf("RAMPA SERVICE %u-%u\n", rampa_service_start, rampa_service_end);
                    }
                }
            }
            else if(param->search_res.conn_id == xcadey_conn_id) {
                if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
                    if (param->search_res.srvc_id.uuid.uuid.uuid16 == 0x1818) {
                        xcadey_service_start = param->search_res.start_handle;
                        xcadey_service_end = param->search_res.end_handle;
                        printf("XCadey CPS %u-%u\n", xcadey_service_start, xcadey_service_end);
                    }
                    uint16_t uuid16 = param->search_res.srvc_id.uuid.uuid.uuid16;
                    printf("XCADEY UUID16=%04X\n", uuid16);
                }
                // printf("XCadey service found\n");
            }
            else if(param->search_res.conn_id == polar_conn_id) {
                if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
                    if (param->search_res.srvc_id.uuid.uuid.uuid16 == 0x180d) {
                        polar_service_start = param->search_res.start_handle;
                        polar_service_end = param->search_res.end_handle;
                        printf("Polar service %u-%u\n", polar_service_start, polar_service_end);
                    }
                    uint16_t uuid16 = param->search_res.srvc_id.uuid.uuid.uuid16;
                    printf("Polar UUID16=%04X\n", uuid16);
                }
            }                
        break;
        default:
            ;
        break;
    }
}

static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    switch(event) {
        case ESP_GATTS_START_EVT:
            printf("SERVICE STARTED\n");
        break;        
case ESP_GATTS_REG_EVT: {

    printf("GATTS REG status=%d app_id=%u if=%d\n",
           param->reg.status,
           param->reg.app_id,
           gatts_if);

    if (param->reg.status != ESP_GATT_OK) {
        printf("GATTS REG FAILED\n");
        break;
    }

    esp_err_t err = esp_ble_gatts_create_attr_tab(
        ftms_gatt_db,
        gatts_if,
        FTMS_IDX_NB,
        0
    );

    printf("CREATE FTMS TABLE ret=%d (%s)\n",
           err, esp_err_to_name(err));

    break;
}
        // case ESP_GATTS_CREAT_ATTR_TAB_EVT:

        //     printf("ATTR TAB num=%u\n", param->add_attr_tab.num_handle);

        //     if (param->add_attr_tab.num_handle == FTMS_IDX_NB) {

        //         memcpy(ftms_handle_table,
        //             param->add_attr_tab.handles,
        //             sizeof(ftms_handle_table));

        //         esp_ble_gatts_start_service(
        //             ftms_handle_table[IDX_SVC]
        //         );

        //         printf("FTMS START\n");
        //     }
        //     else if (param->add_attr_tab.num_handle == HR_IDX_NB) {

        //         memcpy(hr_handle_table,
        //             param->add_attr_tab.handles,
        //             sizeof(hr_handle_table));

        //         esp_ble_gatts_start_service(
        //             hr_handle_table[HR_IDX_SVC]
        //         );

        //         printf("HR START\n");
        //     }

        // break;

/**        
            printf("ATTR TAB num=%u\n",param->add_attr_tab.num_handle);
            memcpy(ftms_handle_table, param->add_attr_tab.handles, sizeof(ftms_handle_table));
            esp_ble_gatts_start_service(ftms_handle_table[IDX_SVC]);
            memcpy(hr_handle_table, param->add_attr_tab.handles, sizeof(hr_handle_table));
            esp_ble_gatts_start_service(hr_handle_table[HR_IDX_SVC]);
                        
            // printf("svc=%u\n", ftms_handle_table[IDX_SVC]);
            // printf("char=%u\n", ftms_handle_table[IDX_CHAR_VAL_FEATURE]);
            // printf("ATTR TABLE CREATED\n");
            // printf("svc=%u\n", ftms_handle_table[IDX_SVC]);
            // printf("feature=%u\n", ftms_handle_table[IDX_CHAR_VAL_FEATURE]);
            // printf("bike=%u\n", ftms_handle_table[IDX_CHAR_VAL_BIKE_DATA]);
            // printf("cp=%u\n", ftms_handle_table[IDX_CHAR_VAL_CP]);      
 

            if(param->add_attr_tab.num_handle == FTMS_IDX_NB) {
                memcpy(ftms_handle_table, param->add_attr_tab.handles, sizeof(ftms_handle_table));
                esp_ble_gatts_start_service(ftms_handle_table[IDX_SVC]);
                printf("FTMS START\n");
            }
            else if(param->add_attr_tab.num_handle == HR_IDX_NB) {
                memcpy(hr_handle_table, param->add_attr_tab.handles, sizeof(hr_handle_table));
                esp_ble_gatts_start_service(hr_handle_table[HR_IDX_SVC]);
                printf("HR START\n");
            }
                
        break;
*/        
case ESP_GATTS_CREAT_ATTR_TAB_EVT: {

    printf("ATTR TAB status=%d num=%u\n",
           param->add_attr_tab.status,
           param->add_attr_tab.num_handle);

    if (param->add_attr_tab.status != ESP_GATT_OK) {
        printf("ATTR TABLE FAILED status=%d\n",
               param->add_attr_tab.status);
        break;
    }

    /*
     * Najpierw FTMS
     */
    if (param->add_attr_tab.num_handle == FTMS_IDX_NB) {

        memcpy(
            ftms_handle_table,
            param->add_attr_tab.handles,
            sizeof(ftms_handle_table)
        );

        printf("FTMS TABLE OK\n");

        for (int i = 0; i < FTMS_IDX_NB; i++) {
            printf("FTMS handle[%d]=%u\n",
                   i,
                   ftms_handle_table[i]);
        }

        esp_err_t err =
            esp_ble_gatts_start_service(
                ftms_handle_table[IDX_SVC]
            );

        printf("FTMS START ret=%d (%s), svc=%u\n",
               err,
               esp_err_to_name(err),
               ftms_handle_table[IDX_SVC]);

        /*
         * Dopiero po utworzeniu FTMS tworzymy HR
         */
        err = esp_ble_gatts_create_attr_tab(
            hr_gatt_db,
            gatts_if,
            HR_IDX_NB,
            1
        );

        printf("CREATE HR TABLE ret=%d (%s)\n",
               err,
               esp_err_to_name(err));
    }

    /*
     * Potem HR
     */
    else if (param->add_attr_tab.num_handle == HR_IDX_NB) {

        memcpy(
            hr_handle_table,
            param->add_attr_tab.handles,
            sizeof(hr_handle_table)
        );

        printf("HR TABLE OK\n");

        for (int i = 0; i < HR_IDX_NB; i++) {
            printf("HR handle[%d]=%u\n",
                   i,
                   hr_handle_table[i]);
        }

        esp_err_t err =
            esp_ble_gatts_start_service(
                hr_handle_table[HR_IDX_SVC]
            );

        printf("HR START ret=%d (%s), svc=%u\n",
               err,
               esp_err_to_name(err),
               hr_handle_table[HR_IDX_SVC]);
    }

    else {
        printf("UNKNOWN ATTR TABLE num=%u\n",
               param->add_attr_tab.num_handle);
    }

    break;
}
case ESP_GATTS_CONNECT_EVT:

    printf("GATTS CONNECT conn=%u role=%u "
           "%02X:%02X:%02X:%02X:%02X:%02X\n",
           param->connect.conn_id,
           param->connect.link_role,
           param->connect.remote_bda[0],
           param->connect.remote_bda[1],
           param->connect.remote_bda[2],
           param->connect.remote_bda[3],
           param->connect.remote_bda[4],
           param->connect.remote_bda[5]);

    /*
     * role=1 -> ESP32 jest peripheral,
     * czyli to PC / MyWhoosh.
     */

if (param->connect.link_role == 1) {

    ftms_connected = true;
    ftms_conn_id = param->connect.conn_id;
    ftms_gatts_if = gatts_if;

    ftms_bike_notify_enabled = false;
    ftms_training_notify_enabled = false;
    ftms_status_notify_enabled = false;
    ftms_cp_indications_enabled = false;

    ftms_started = false;

    printf("FTMS CLIENT CONNECTED conn=%u gatts_if=%u\n",
           ftms_conn_id,
           ftms_gatts_if);
}

    break;

case ESP_GATTS_DISCONNECT_EVT:

    printf("GATTS DISCONNECT conn=%u reason=%u\n",
           param->disconnect.conn_id,
           param->disconnect.reason);


    /*
     * Bluedroid generuje GATTS disconnect także dla
     * połączeń wykonanych przez GATTC.
     *
     * Czyścimy FTMS tylko wtedy, gdy rozłączył się
     * faktyczny klient FTMS.
     */
    if (ftms_connected &&
        param->disconnect.conn_id == ftms_conn_id) {

        printf("FTMS CLIENT DISCONNECTED conn=%u\n",
               ftms_conn_id);
ftms_bike_notify_enabled = false;
ftms_training_notify_enabled = false;
ftms_status_notify_enabled = false;
ftms_cp_indications_enabled = false;
ftms_started = false;
        ftms_connected = false;
        ftms_conn_id = INVALID_CONN_ID;
        ftms_gatts_if = ESP_GATT_IF_NONE;

        

        esp_err_t err =
            esp_ble_gap_start_advertising(&adv_params);

        printf("Advertising restart: %s\n",
               esp_err_to_name(err));
    }
    else {

        printf("GATTS DISCONNECT ignored - not FTMS client\n");
    }

    break;

case ESP_GATTS_WRITE_EVT: 

if (param->write.len == 2) {

    uint16_t cccd =
        param->write.value[0] |
        (param->write.value[1] << 8);

    if (param->write.handle ==
        ftms_handle_table[IDX_CHAR_CFG_BIKE_DATA]) {

        ftms_bike_notify_enabled =
            (cccd & 0x0001) != 0;

        printf("BIKE CCCD=%04X notify=%d\n",
               cccd,
               ftms_bike_notify_enabled);
    }

    else if (param->write.handle ==
             ftms_handle_table[IDX_CHAR_CFG_TRAINING]) {

        ftms_training_notify_enabled =
            (cccd & 0x0001) != 0;

        printf("TRAINING CCCD=%04X notify=%d\n",
               cccd,
               ftms_training_notify_enabled);
    }

    else if (param->write.handle ==
             ftms_handle_table[IDX_CHAR_CFG_STATUS]) {

        ftms_status_notify_enabled =
            (cccd & 0x0001) != 0;

        printf("STATUS CCCD=%04X notify=%d\n",
               cccd,
               ftms_status_notify_enabled);
    }

    else if (param->write.handle ==
             ftms_handle_table[IDX_CHAR_CFG_CP]) {

        ftms_cp_indications_enabled =
            (cccd & 0x0002) != 0;

        printf("CP CCCD=%04X indications=%d\n",
               cccd,
               ftms_cp_indications_enabled);
    }
}

    /*
     * CCCD Control Point
     */
    if (param->write.handle ==
        ftms_handle_table[IDX_CHAR_CFG_CP]) {

        if (param->write.len >= 2) {

            uint16_t cccd =
                param->write.value[0] |
                (param->write.value[1] << 8);

            ftms_cp_indications_enabled =
                (cccd & 0x0002) != 0;

            printf("CP CCCD=%04X indications=%d\n",
                   cccd,
                   ftms_cp_indications_enabled);
        }
    }

    /*
     * Control Point
     */
if (param->write.handle ==
        ftms_handle_table[IDX_CHAR_VAL_CP] &&
    param->write.len > 0) {

    uint8_t opcode = param->write.value[0];

    printf("FTMS CP WRITE conn=%u opcode=%02X len=%u\n",
           param->write.conn_id,
           opcode,
           param->write.len);

    printf("CP DATA:");
    for (int i = 0; i < param->write.len; i++)
        printf(" %02X", param->write.value[i]);
    printf("\n");

    // Start / Resume
if (opcode == 0x07) {
    ftms_started = true;
    printf("FTMS STARTED\n");
}
else if (opcode == 0x08) {
    ftms_started = false;
    printf("FTMS STOPPED\n");
}
    // sterowanie mocą ERG
    if (opcode == 0x05 && param->write.len >= 3) {
        uint16_t power =
            param->write.value[1] |
            (param->write.value[2] << 8);

        printf("SET TARGET POWER=%u\n", power);
                    // blink LED to indicate power change
                    if(green > 0) {
                        blue = 0x32;
                        green = 0;
                        red = 0;
                    }   
                    else {
                        blue = 0;
                        green = 0x32;
                        red = 0;
                    }
              if (power < 0xa0) {    //<160W 
                        if (power < 0x78)  //<120W
                            power += 0x08;
                        power += 0x10;
                    }
                    else
                        power += 0x05;
                    uint8_t byte_1 = power & 0xff;
                    uint8_t byte_2 = power >> 8;
                    elite_send(byte_1, byte_2);
                }
                uint8_t resp[3] = {0x80, opcode, 0x01 };
if (ftms_cp_indications_enabled) {
    esp_ble_gatts_send_indicate(
        gatts_if,
        param->write.conn_id,
        ftms_handle_table[IDX_CHAR_VAL_CP],
        sizeof(resp),
        resp,
        true
    );                
               
                // printf("FTMS CP opcode=%02X\n", opcode);

        // ...Twoja dalsza obsługa...
    }

    // odpowiedź Control Point
    // uint8_t resp[3] = {0x80, opcode, 0x01};

    // ...
    }

    /*
     * Zwykłe zapisy
     */
    if (param->write.need_rsp) {

        esp_ble_gatts_send_response(
            gatts_if,
            param->write.conn_id,
            param->write.trans_id,
            ESP_GATT_OK,
            NULL
        );
    }

    break;       

        case ESP_GATTS_READ_EVT:
            // printf("READ handle=%u\n", param->read.handle);
        break;     
case ESP_GATTS_CONF_EVT:

    if (param->conf.handle ==
        ftms_handle_table[IDX_CHAR_VAL_CP]) {

        printf("FTMS CP CONF status=%d\n",
               param->conf.status);
    }

    break;

    break;
        default:
            ;
        break; 
    }
}

void app_main(void) {

    static TickType_t btn_press_time;
    static int8_t btn, last_btn = 1;
    static uint16_t resistance = 0x80;

    // gpio_config_t io_conf = {.pin_bit_mask = (1ULL << BOOT_PIN), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE,};
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << BOOT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_cfg);
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led));
    xTaskCreate(LEDTask, "LED", 4096, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(FTMSTask, "FTMS", 4096, NULL, 5, NULL);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_eddystone_init();
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));
    printf("\n\nReceiver started.\n\n");
    esp_ble_gap_set_device_name("ESP32-FTMS");    
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_cb));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(1));    

esp_err_t err = esp_ble_gap_config_adv_data_raw(
    raw_adv_data,
    sizeof(raw_adv_data)
);

printf("ADV RAW config: %s\n", esp_err_to_name(err));    
    //esp_err_t ret;
    //ret = 
    // esp_ble_gap_config_adv_data(&adv_data);
    // printf("config_adv_data ret=%d %s\n", ret, esp_err_to_name(ret));

    //ret = 
    esp_ble_gap_set_scan_params(&ble_scan_params);
    // printf("set_scan_params ret=%d\n", ret);   
    
    printf("\n\nSender FTMS started.\n\n");
    
    while (1) {
        if (elite_ready) {     //elite_ready
            btn = gpio_get_level(BOOT_PIN);
            if (btn == 0 && last_btn == 1) 
                btn_press_time = xTaskGetTickCount();
            if (btn == 1 && last_btn == 0) {                
                TickType_t duration = xTaskGetTickCount() - btn_press_time;
                if (duration > pdMS_TO_TICKS(3000)) {
                    resistance = 0x80;
                    red = 0;
                    // printf("res=%d r=%d, g=%d, b=,%d\n", resistance, red, green, blue);
                }                
                else {
                    if (duration > pdMS_TO_TICKS(1000)) {             //100
                        resistance += 0x10;
                        red += 0x0a;
                        // printf("res=%d r=%d, g=%d, b=,%d\n", resistance, red, green, blue);
                        if (resistance >= 0x200) {
                            resistance = 0x80;
                            red = 0x00;
                        } 
                    }
                    else {
                        if (resistance > 0x80) {
                            resistance -= 0x10;
                            red -= 0x0a;
                            // printf("res=%d r=%d, g=%d, b=,%d\n", resistance, red, green, blue);
                        }
                    }
                }
                uint8_t byte_1 = resistance & 0xff;
                uint8_t byte_2 = resistance  >> 8;
                elite_send(byte_1, byte_2);   
            }
            last_btn = btn;    
        }        
        vTaskDelay(pdMS_TO_TICKS(50));         
    // vTaskDelete(NULL);
    }
}

