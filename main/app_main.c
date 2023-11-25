#include "string.h"
#include "esp_log.h"
#include "modbus_params.h" // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "tusb_msc.h"

#define MB_PORT_NUM 1   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED 9600 // The communication speed of the UART
#define MB_UART_RXD 18
#define MB_UART_TXD 17
#define MB_UART_RTS UART_PIN_NO_CHANGE
#define MB_COMM_MODE MB_COMM_MODE_RTU

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 30

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS (5000)
#define UPDATE_CIDS_TIMEOUT_TICS (UPDATE_CIDS_TIMEOUT_MS / portTICK_PERIOD_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS (1)
#define POLL_TIMEOUT_TICS (POLL_TIMEOUT_MS / portTICK_PERIOD_MS)

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char *)(fieldname))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val)                   \
    {                                                      \
        .opt1 = min_val, .opt2 = max_val, .opt3 = step_val \
    }

static const char *TAG = "app";

// Enumeration of modbus device addresses accessed by master device
enum
{
    MB_DEVICE_ADDR1 = 1 // Only one slave device used for the test (add other slave addresses here)
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum
{
    CID_HOL_DATA_0 = 0,
    CID_COUNT
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    {CID_HOL_DATA_0, STR("Voltage"), STR("Volts"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0, 2,
     HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, PARAM_SIZE_U16, OPTS(0, 400, 1), PAR_PERMS_READ_WRITE_TRIGGER}};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters) / sizeof(device_parameters[0]));

// The function to get pointer to parameter storage (instance) according to parameter description table
static void *master_get_param_data(const mb_parameter_descriptor_t *param_descriptor)
{
    assert(param_descriptor != NULL);
    void *instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
        switch (param_descriptor->mb_param_type) {
            case MB_PARAM_HOLDING:
                instance_ptr = ((void *)&holding_reg_params + param_descriptor->param_offset - 1);
                break;
            case MB_PARAM_INPUT:
                instance_ptr = ((void *)&input_reg_params + param_descriptor->param_offset - 1);
                break;
            case MB_PARAM_COIL:
                instance_ptr = ((void *)&coil_reg_params + param_descriptor->param_offset - 1);
                break;
            case MB_PARAM_DISCRETE:
                instance_ptr = ((void *)&discrete_reg_params + param_descriptor->param_offset - 1);
                break;
            default:
                instance_ptr = NULL;
                break;
        }
    } else {
        ESP_LOGE(TAG, "Wrong parameter offset for CID #%d", param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// User operation function to read slave values and check alarm
static void master_task(void *arg)
{
    esp_err_t err = ESP_OK;
    const mb_parameter_descriptor_t *param_descriptor = NULL;

    while (1) {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++) {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                void *temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;
                if (param_descriptor->param_type == PARAM_TYPE_ASCII) {
                    // Check for long array of registers of type PARAM_TYPE_ASCII
                    if (mbc_master_get_parameter(cid, (char *)param_descriptor->param_key, (uint8_t *)temp_data_ptr, &type) == ESP_OK) {
                        ESP_LOGI(TAG, "characteristic #%d %s (%s) value = (0x%08x) read successful.",
                                 param_descriptor->cid,
                                 (char *)param_descriptor->param_key,
                                 (char *)param_descriptor->param_units,
                                 *(uint32_t *)temp_data_ptr);
                    } else {
                        ESP_LOGE(TAG, "characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                 param_descriptor->cid,
                                 (char *)param_descriptor->param_key,
                                 (int)err,
                                 (char *)esp_err_to_name(err));
                    }
                } else {
                    if (mbc_master_get_parameter(cid, (char *)param_descriptor->param_key, (uint8_t *)temp_data_ptr, &type) == ESP_OK) {
                        if ((param_descriptor->mb_param_type == MB_PARAM_HOLDING) || (param_descriptor->mb_param_type == MB_PARAM_INPUT)) {
                            uint16_t value = *(uint16_t *)temp_data_ptr;
                            ESP_LOGI(TAG, "characteristic #%d %s (%s) value = %d (0x%" PRIx32 ") read successful.",
                                     param_descriptor->cid,
                                     (char *)param_descriptor->param_key,
                                     (char *)param_descriptor->param_units,
                                     value,
                                     *(uint32_t *)temp_data_ptr);
                            append_data(value);
                            
                        } else {
                            uint16_t state = *(uint16_t *)temp_data_ptr;
                            const char *rw_str = (state & param_descriptor->param_opts.opt1) ? "ON" : "OFF";
                            ESP_LOGI(TAG, "characteristic #%d %s (%s) value = %s (0x%x) read successful.",
                                     param_descriptor->cid,
                                     (char *)param_descriptor->param_key,
                                     (char *)param_descriptor->param_units,
                                     (const char *)rw_str,
                                     *(uint16_t *)temp_data_ptr);
                        }
                    } else {
                        ESP_LOGE(TAG, "characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                 param_descriptor->cid,
                                 (char *)param_descriptor->param_key,
                                 (int)err,
                                 (char *)esp_err_to_name(err));
                    }
                }
                vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
            }
        }
        vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS); //
    }
}

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
        .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
        .mode = MB_MODE_RTU,
#endif
        .baudrate = MB_DEV_SPEED,
        .parity = MB_PARITY_NONE
    };
    void *master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG, "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller initialization fail, returns(0x%x).", (uint32_t)err);
    err = mbc_master_setup((void *)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller setup fail, returns(0x%x).", (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller start fail, returns(0x%x).", (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller set descriptor fail, returns(0x%x).", (uint32_t)err);
    ESP_LOGI(TAG, "Modbus master stack initialized...");
    return err;
}

void app_main(void)
{
    storage_main();
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);
    xTaskCreate(&master_task, "master_task", 8192, NULL, 5, NULL);
}
