/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
  SCD4x patch : https://github.com/UncleRus/esp-idf-lib/pull/700

  rm -rf managed_components/
  rm -f dependencies.lock
  idf.py fullclean
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <esp_matter.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#include <app-common/zap-generated/cluster-objects.h>  // enum 정의 필요 시

static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;
using namespace chip::app;
namespace CDCM = chip::app::Clusters::CarbonDioxideConcentrationMeasurement;
namespace NM = chip::app::Clusters::CarbonDioxideConcentrationMeasurement;

extern void sensor_init( void );
extern esp_err_t sensor_get( float *temp, float *humidity, uint16_t *co2 );


// CO2(ppm) → AirQuality(enum) 간단 매핑
static uint8_t map_co2_to_air_quality_enum(float ppm) {
    using AQ = Clusters::AirQuality::AirQualityEnum;
    if (ppm < 600)   return (uint8_t)AQ::kGood;
    if (ppm < 1000)  return (uint8_t)AQ::kFair;
    if (ppm < 1500)  return (uint8_t)AQ::kModerate;
    if (ppm < 2000)  return (uint8_t)AQ::kPoor;
    if (ppm < 5000)  return (uint8_t)AQ::kVeryPoor;
    return (uint8_t)AQ::kExtremelyPoor;
}

using scd4x_sensor_cb_t = void (*)(uint16_t endpoint_id, float value, void *user_data);

typedef struct {
    struct {
        // This callback functon will be called periodically to report the temperature.
        scd4x_sensor_cb_t cb = NULL;
        // endpoint_id associated with temperature sensor
        uint16_t endpoint_id;
    } temperature;

    struct {
        // This callback functon will be called periodically to report the humidity.
        scd4x_sensor_cb_t cb = NULL;
        // endpoint_id associated with humidity sensor
        uint16_t endpoint_id;
    } humidity;

    struct {
        // This callback functon will be called periodically to report the humidity.
        scd4x_sensor_cb_t cb = NULL;
        // endpoint_id associated with humidity sensor
        uint16_t endpoint_id;
    } co2;

    // user data
    void *user_data = NULL;

    // polling interval in milliseconds, defaults to 5000 ms
    uint32_t interval_ms = 10000;
} scd4x_sensor_config_t;

// Application cluster specification, 7.18.2.11. Temperature
// represents a temperature on the Celsius scale with a resolution of 0.01°C.
// temp = (temperature in °C) x 100
static void temp_sensor_notification(uint16_t endpoint_id, float temp, void *user_data)
{
    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, temp]() {
        attribute_t * attribute = attribute::get(endpoint_id,
                                                 TemperatureMeasurement::Id,
                                                 TemperatureMeasurement::Attributes::MeasuredValue::Id);

        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.i16 = static_cast<int16_t>(temp * 100);

        attribute::update(endpoint_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
    });
}


// Application cluster specification, 2.6.4.1. MeasuredValue Attribute
// represents the humidity in percent.
// humidity = (humidity in %) x 100
static void humidity_sensor_notification(uint16_t endpoint_id, float humidity, void *user_data)
{
    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, humidity]() {
        attribute_t * attribute = attribute::get(endpoint_id,
                                                 RelativeHumidityMeasurement::Id,
                                                 RelativeHumidityMeasurement::Attributes::MeasuredValue::Id);

        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.u16 = static_cast<uint16_t>(humidity * 100);

        attribute::update(endpoint_id, RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
    });
}

// CO2 concentration measurement
// represents CO2 concentration in parts per million (ppm)
static void co2_sensor_notification(uint16_t endpoint_id, float co2_ppm, void *user_data)
{
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, co2_ppm]() {
        // 1) CO2 MeasuredValue(float) 갱신
        if (auto *attr = attribute::get(endpoint_id,
                Clusters::CarbonDioxideConcentrationMeasurement::Id,
                Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id)) {

            esp_matter_attr_val_t v = esp_matter_invalid(NULL);
            attribute::get_val(attr, &v);
            v.val.f = co2_ppm;                 // 타입 지정 불필요(속성 타입 유지)
            attribute::update(endpoint_id,
                Clusters::CarbonDioxideConcentrationMeasurement::Id,
                Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id, &v);
        }

        // 2) AirQuality(enum) 갱신 → HA의 "Air quality" 채워짐
        if (auto *attr_aq = attribute::get(endpoint_id,
                Clusters::AirQuality::Id,
                Clusters::AirQuality::Attributes::AirQuality::Id)) {

            esp_matter_attr_val_t v = esp_matter_invalid(NULL);
            attribute::get_val(attr_aq, &v);
            v.val.u8 = map_co2_to_air_quality_enum(co2_ppm);  // enum 값
            attribute::update(endpoint_id,
                Clusters::AirQuality::Id,
                Clusters::AirQuality::Attributes::AirQuality::Id, &v);
        }
    });
}

constexpr auto k_timeout_seconds = 300;

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        /* Driver update */
    }

    return err;
}

typedef struct {
    scd4x_sensor_config_t *config;
    esp_timer_handle_t timer;
    bool is_initialized = false;
} scd4x_sensor_ctx_t;

static scd4x_sensor_ctx_t s_ctx;

static void timer_cb_internal(void *arg)
{
    auto *ctx = (scd4x_sensor_ctx_t *) arg;
    if (!(ctx && ctx->config)) {
        return;
    }

    float temp, humidity;
    uint16_t co2;
    esp_err_t err = sensor_get(&temp, &humidity, &co2);
    if (err != ESP_OK) {
        return;
    }
    if (ctx->config->temperature.cb) {
        ctx->config->temperature.cb(ctx->config->temperature.endpoint_id, temp, ctx->config->user_data);
    }
    if (ctx->config->humidity.cb) {
        ctx->config->humidity.cb(ctx->config->humidity.endpoint_id, humidity, ctx->config->user_data);
    }
    if (ctx->config->co2.cb) {
        ctx->config->co2.cb(ctx->config->co2.endpoint_id, (float)co2, ctx->config->user_data);
    }    
}

static esp_err_t sensor_timer_init( scd4x_sensor_config_t *config )
{
    esp_err_t err = ESP_OK;

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // we need at least one callback so that we can start notifying application layer
    if (config->temperature.cb == NULL || config->humidity.cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // keep the pointer to config
    s_ctx.config = config;    

    esp_timer_create_args_t args = {
        .callback = timer_cb_internal,
        .arg = &s_ctx,
    };

    err = esp_timer_create(&args, &s_ctx.timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed, err:%d", err);
        return err;
    }

    #if 1
    err = esp_timer_start_periodic(s_ctx.timer, config->interval_ms * 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", err);
        return err;
    }
    #endif


    s_ctx.is_initialized = true;
    ESP_LOGI(TAG, "shtc3 initialized successfully");
    
    return ESP_OK;
}


extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    err = esp_pm_configure(&pm_config);
#endif
#ifdef CONFIG_ENABLE_USER_ACTIVE_MODE_TRIGGER_BUTTON
    app_driver_button_init();
#endif

    sensor_init();
#if 0
    float temp, humidity;
    uint16_t co2;

    for( int i = 0; i < 100000; i++ ) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        sensor_get(&temp, &humidity, &co2);
    }
#endif
    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

#if 0
    endpoint::on_off_light::config_t endpoint_config;
    endpoint_t *app_endpoint = endpoint::on_off_light::create(node, &endpoint_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(app_endpoint != nullptr, ESP_LOGE(TAG, "Failed to create on off light endpoint"));
#endif    

    // add temperature sensor device
    temperature_sensor::config_t temp_sensor_config;
    endpoint_t * temp_sensor_ep = temperature_sensor::create(node, &temp_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));

    // add the humidity sensor device
    humidity_sensor::config_t humidity_sensor_config;
    endpoint_t * humidity_sensor_ep = humidity_sensor::create(node, &humidity_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(humidity_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create humidity_sensor endpoint"));

    // add CO2 sensor device
    air_quality_sensor::config_t co2_sensor_config;
    endpoint_t * co2_sensor_ep = air_quality_sensor::create(node, &co2_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(co2_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create CO2 sensor endpoint"));

    // [ADD] Air Quality 엔드포인트에 CO2 농도 측정 클러스터(서버) 붙이기
    cluster_t *co2_cluster = cluster::create(co2_sensor_ep,
    CarbonDioxideConcentrationMeasurement::Id, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(co2_cluster != nullptr, ESP_LOGE(TAG, "Failed to create CO2 cluster"));    

{
    using namespace chip::app::Clusters;

    // 1) MeasuredValue (float = 0.0)
    if (!attribute::get(co2_cluster,
        Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id)) {
        esp_matter_attr_val_t v = esp_matter_invalid(NULL);
        v.type  = ESP_MATTER_VAL_TYPE_FLOAT;
        v.val.f = 0.0f;
        attribute::create(co2_cluster,
            Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id,
            /*flags*/0, /*value*/ v);
    }

    // 2) MeasurementUnit (uint8 = ppm)
    if (!attribute::get(co2_cluster,
        Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MeasurementUnit::Id)) {
        esp_matter_attr_val_t v = esp_matter_invalid(NULL);
        v.type   = ESP_MATTER_VAL_TYPE_UINT8;
        v.val.u8 = (uint8_t)Clusters::CarbonDioxideConcentrationMeasurement::MeasurementUnitEnum::kPpm;
        attribute::create(co2_cluster,
            Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MeasurementUnit::Id,
            0, v);
    }

    // 3) MinMeasuredValue (float = 0.0)
    if (!attribute::get(co2_cluster,
        Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MinMeasuredValue::Id)) {
        esp_matter_attr_val_t v = esp_matter_invalid(NULL);
        v.type  = ESP_MATTER_VAL_TYPE_FLOAT;
        v.val.f = 0.0f;
        attribute::create(co2_cluster,
            Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MinMeasuredValue::Id,
            0, v);
    }

    // 4) MaxMeasuredValue (float = 40000.0)
    if (!attribute::get(co2_cluster,
        Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MaxMeasuredValue::Id)) {
        esp_matter_attr_val_t v = esp_matter_invalid(NULL);
        v.type  = ESP_MATTER_VAL_TYPE_FLOAT;
        v.val.f = 40000.0f;
        attribute::create(co2_cluster,
            Clusters::CarbonDioxideConcentrationMeasurement::Attributes::MaxMeasuredValue::Id,
            0, v);
    }
}


    static scd4x_sensor_config_t scd4x_config = {
        .temperature = {
            .cb = temp_sensor_notification,
            .endpoint_id = endpoint::get_id(temp_sensor_ep),
        },
        .humidity = {
            .cb = humidity_sensor_notification,
            .endpoint_id = endpoint::get_id(humidity_sensor_ep),
        },
        .co2 = {
            .cb = co2_sensor_notification,
            .endpoint_id = endpoint::get_id(co2_sensor_ep),
        },        
    };    

    err = sensor_timer_init( &scd4x_config );
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialize temperature sensor driver"));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));
}
