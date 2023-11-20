#include <errno.h>
#include <dirent.h>
#include "esp_console.h"
#include "esp_check.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "tusb_msc.h"
#include <dirent.h>
#include <stdio.h>
#include "led_strip.h"
#include "iot_button.h"

static const char *TAG = "msc";

/* TinyUSB descriptors
   ********************************************************************* */
#define EPNUM_MSC       1
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)
#define BASE_PATH "/data" // base path to mount the partition
#define STORAGE_MAX 14540800
#define AS_FILE_LINE_SIZE 6
#define AS_FILE_LINE_MAX 17280
#define AS_FILE_SIZE 103680
#define AS_FILE_MAX 100
#define BLINK_GPIO 48

enum {
    ITF_NUM_MSC = 0,
    ITF_NUM_TOTAL,
};

enum {
    EDPT_CTRL_OUT = 0x00,
    EDPT_CTRL_IN  = 0x80,

    EDPT_MSC_OUT  = 0x01,
    EDPT_MSC_IN   = 0x81,
};

static led_strip_handle_t led_strip;
static uint16_t file_max = 0;
static uint16_t file_min = UINT16_MAX;
static uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, TUD_OPT_HIGH_SPEED ? 512 : 64),
};
static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A, // This is Espressif VID. This needs to be changed according to Users / Customers
    .idProduct = 0x4002,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};
static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: is supported language is English (0x0409)
    "TinyUSB",                      // 1: Manufacturer
    "TinyUSB Device",               // 2: Product
    "123456",                       // 3: Serials
    "Example MSC",                  // 4. MSC
};
/*********************************************************************** TinyUSB descriptors*/

// callback that is delivered when storage is mounted/unmounted by application.
static void storage_mount_changed_cb(tinyusb_msc_event_t *event)
{
    DIR *d = opendir(BASE_PATH);
    struct dirent *dir;
    if (d) {
        while ((dir = readdir(d)) != NULL) {
            ESP_LOGI(TAG, "found file: %s", dir->d_name);
            uint8_t index = 0;
            if (sscanf(dir->d_name, "%d.txt", &index)) {
                if (index > file_max) file_max = index;
                if (index < file_min) file_min = index;
            }
        }
        closedir(d);
    }

    // ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, event->mount_changed_data.is_mounted ? 0 : 32, 5, 5, 5));
    // ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    ESP_LOGI(TAG, "storage mounted to application: %s, file index[%d, %d]", event->mount_changed_data.is_mounted ? "Yes" : "No", file_min, file_max);
}

static esp_err_t storage_init_spiflash(wl_handle_t *wl_handle)
{
    ESP_LOGI(TAG, "initializing wear levelling");

    const esp_partition_t *data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (data_partition == NULL) {
        ESP_LOGE(TAG, "failed to find fatfs partition. check the partition table.");
        return ESP_ERR_NOT_FOUND;
    }

    return wl_mount(data_partition, wl_handle);
}

static void button_single_click_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "single click");
}

static void button_double_click_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "double click");
}

static void button_long_press_start_cb(void *arg, void *data)
{
    const esp_partition_t *data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (data_partition == NULL) {
        ESP_LOGE(TAG, "failed to find fatfs partition. check the partition table.");
        return;
    }
    ESP_LOGI(TAG, "erasing partition \"%s\" (0x%" PRIx32 " bytes)\n", data_partition->label, data_partition->size);
    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, data_partition->size));
    esp_restart();
}

int append_data(uint16_t value)
{
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        ESP_LOGE(TAG, "storage exposed over usb. application can't write to storage.");
        return -1;
    }
    // read latest file
    char filename[20];
    sprintf(filename, BASE_PATH "/%d.txt", file_max);
    FILE *fd = fopen(filename, "a+");
    uint32_t len = ftell(fd);
    // if lines in latest file exceeds, create new file
    if (len / AS_FILE_LINE_SIZE >= AS_FILE_LINE_MAX) {
        file_max++;
        ESP_LOGI(TAG, "creating file %d, %d", file_max, AS_FILE_MAX);
        // if files in storage exceeds, remove eldest file
        if (file_max >= AS_FILE_MAX) {
            sprintf(filename, BASE_PATH "/%d.txt", file_min);
            ESP_LOGI(TAG, "storage is full, have to delete oldest file: %s", filename);
            int ret = remove(filename);
            if (ret != 0) {
                ESP_LOGI(TAG, "failed to delete file: %d", ret);
            } else {
                file_min++;
            }
        }
        // close last file
        fclose(fd);
        // open new file
        sprintf(filename, BASE_PATH "/%d.txt", file_max);
        fd = fopen(filename, "a+");
        len = ftell(fd);
    }
    if (!fd) {
        ESP_LOGW(TAG, "failed to open %s, skip writing.", filename);
        return -1;
    }
    ESP_LOGI(TAG, "writing value %d to file %s at line %lu", value, filename, len/AS_FILE_LINE_SIZE);
    fprintf(fd, "%.1f\n", (float)value / 10);
    fclose(fd);
    return 0;
}

void storage_main(void)
{
    // led_strip_config_t strip_config = {
    //     .strip_gpio_num = BLINK_GPIO, // The GPIO that connected to the LED strip's data line
    //     .max_leds = 1, // The number of LEDs in the strip,
    //     .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
    //     .led_model = LED_MODEL_WS2812, // LED strip model
    //     .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
    // };

    // led_strip_rmt_config_t rmt_config = {
    //     .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
    //     .resolution_hz = 10 * 1000 * 1000, // 10MHz
    //     .flags.with_dma = false, // whether to enable the DMA feature
    // };
    // ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "initializing buttons...");
    button_config_t cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 3000,
        .short_press_time = 180,
        .gpio_button_config = {
            .gpio_num = GPIO_NUM_0,
            .active_level = 0,
        },
    };
    button_handle_t handle = iot_button_create(&cfg);
    iot_button_register_cb(handle, BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(handle, BUTTON_DOUBLE_CLICK, button_double_click_cb, NULL);
    iot_button_register_cb(handle, BUTTON_LONG_PRESS_START, button_long_press_start_cb, NULL);

    ESP_LOGI(TAG, "initializing storage...");

    static wl_handle_t wl_handle = WL_INVALID_HANDLE;
    ESP_ERROR_CHECK(storage_init_spiflash(&wl_handle));

    const tinyusb_msc_spiflash_config_t config_spi = {
        .wl_handle = wl_handle,
        .callback_mount_changed = storage_mount_changed_cb,  /* First way to register the callback. This is while initializing the storage. */
        .mount_config.max_files = 5,
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_spiflash(&config_spi));
    ESP_ERROR_CHECK(tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, storage_mount_changed_cb)); /* Other way to register the callback i.e. registering using separate API. If the callback had been already registered, it will be overwritten. */

    //mounted in the app by default
    ESP_ERROR_CHECK(tinyusb_msc_storage_mount(BASE_PATH));

    ESP_LOGI(TAG, "usb msc initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_config,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "usb msc initialization done");
}