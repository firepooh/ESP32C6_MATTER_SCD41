/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_log.h>
#include <esp_matter.h>
#include <sdkconfig.h>

#include <app/icd/server/ICDNotifier.h>

#include <app_priv.h>
#include <iot_button.h>
#include <button_gpio.h>
#include <scd4x.h>

#define CONFIG_EXAMPLE_I2C_MASTER_SDA       GPIO_NUM_3
#define CONFIG_EXAMPLE_I2C_MASTER_SCL       GPIO_NUM_2
#define I2C_MASTER_NUM                      I2C_NUM_0

static constexpr char *TAG_SENSOR = "scd4x";
static i2c_dev_t dev;

void sensor_init( void )
{
    ESP_ERROR_CHECK(i2cdev_init());    

    ESP_ERROR_CHECK(scd4x_init_desc(&dev, I2C_MASTER_NUM, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    ESP_LOGI(TAG_SENSOR, "Initializing sensor...");
    ESP_ERROR_CHECK(scd4x_wake_up(&dev));
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&dev));
    ESP_ERROR_CHECK(scd4x_reinit(&dev));
    ESP_LOGI(TAG_SENSOR, "Sensor initialized");

    uint16_t serial[3];
    ESP_ERROR_CHECK(scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2));
    ESP_LOGI(TAG_SENSOR, "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    ESP_ERROR_CHECK(scd4x_start_periodic_measurement(&dev));
    ESP_LOGI(TAG_SENSOR, "Periodic measurements started");
}

void sensor_get( void ) 
{
    uint16_t co2;
    float temperature, humidity;    

    esp_err_t res = scd4x_read_measurement(&dev, &co2, &temperature, &humidity);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG_SENSOR, "Error reading results %d (%s)", res, esp_err_to_name(res));
    }

    ESP_LOGI(TAG_SENSOR, "CO2: %u ppm", co2);
    ESP_LOGI(TAG_SENSOR, "Temperature: %.2f °C", temperature);
    ESP_LOGI(TAG_SENSOR, "Humidity: %.2f %%", humidity);    
}


#ifdef CONFIG_ENABLE_USER_ACTIVE_MODE_TRIGGER_BUTTON
using namespace chip::app::Clusters;
using namespace esp_matter;

static constexpr char *TAG = "app_driver";

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    // The device will stay active mode for Active Mode Threshold
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
        chip::app::ICDNotifier::GetInstance().NotifyNetworkActivityNotification();
    });
}

app_driver_handle_t app_driver_button_init()
{
    /* Initialize button */
    button_handle_t handle = NULL;
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = CONFIG_USER_ACTIVE_MODE_TRIGGER_BUTTON_PIN,
        .active_level = 0,
        .enable_power_save = true,
    };

    if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button device");
        return NULL;
    }

    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
    return (app_driver_handle_t)handle;
}

#endif // CONFIG_ENABLE_USER_ACTIVE_MODE_TRIGGER_BUTTON
