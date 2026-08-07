#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <nvs_flash.h>

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/QRCodeSetupPayloadGenerator.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#include <esp_openthread_types.h>

// The default OpenThread platform config macros are provided by the examples,
// not the SDK. ESP32-C6 has a native 802.15.4 radio (RADIO_MODE_NATIVE).
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
    {                                                                                   \
        .radio_mode = RADIO_MODE_NATIVE,                                                \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
    {                                                                                   \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "heat_meter_cluster.h"
#include "mbus.h"
#include "mbus_parser.h"

#include <math.h>

static const char *TAG = "Main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static uint16_t flow_endpoint_id = 0;       // standard Flow Measurement (0x0404)
static uint16_t heat_meter_endpoint_id = 0; // custom high-precision cluster

#define MBUS_POLL_INTERVAL_MS 10000

// Bring-up mode. Work up the ladder as each layer is proven:
//   NKE_ONLY -> is the bus wired right and does the meter ACK?
//   TEST     -> does a telegram arrive, and what is actually in it?
//   NORMAL   -> parse into heat_meter_data_t and publish to Matter.
#define MBUS_MODE_NORMAL   0
#define MBUS_MODE_NKE_ONLY 1
#define MBUS_MODE_TEST     2

#ifndef MBUS_MODE
#define MBUS_MODE MBUS_MODE_TEST
#endif

#define MBUS_NKE_TEST_INTERVAL_MS 2000
#define MBUS_TEST_INTERVAL_MS 3000

#define ABORT_APP_ON_FAILURE(x, ...)               \
    do                                             \
    {                                              \
        if (!(unlikely(x)))                        \
        {                                          \
            __VA_ARGS__;                           \
            vTaskDelay(5000 / portTICK_PERIOD_MS); \
            abort();                               \
        }                                          \
    } while (0)

static void open_commissioning_window_if_necessary()
{
    VerifyOrReturn(chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);

    chip::CommissioningWindowManager &mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    VerifyOrReturn(mgr.IsCommissioningWindowOpen() == false);

    CHIP_ERROR err = mgr.OpenBasicCommissioningWindow(
        chip::System::Clock::Seconds16(300),
        chip::CommissioningWindowAdvertisement::kDnssdOnly);
    if (err != CHIP_NO_ERROR)
    {
        ESP_LOGE(TAG, "Failed to open commissioning window: %" CHIP_ERROR_FORMAT, err.Format());
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type)
    {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        open_commissioning_window_if_necessary();
        break;
    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized");
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id,
                                       uint8_t effect_id, uint8_t effect_variant, void *priv_data)
{
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type,
                                         uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val,
                                         void *priv_data)
{
    return ESP_OK;
}

// --- Pushing meter values into the Matter data model -----------------------
// Called on the Matter/CHIP thread (via ScheduleLambda) so attribute::update is
// safe to call. Currently unreferenced while the publish call below is commented
// out for bench testing.
__attribute__((unused))
static void publish_meter_data(const heat_meter_data_t &d)
{
    if (d.has_flow)
    {
        nullable<float> v; v = d.flow_m3h;
        esp_matter_attr_val_t val = esp_matter_nullable_float(v);
        attribute::update(heat_meter_endpoint_id, HEAT_METER_CLUSTER_ID, HM_ATTR_FLOW_ID, &val);
    }
    if (d.has_flow_temp)
    {
        nullable<int32_t> v; v = (int32_t)lround(d.flow_temp_c * 100.0f); // 0.01 degC
        esp_matter_attr_val_t val = esp_matter_nullable_int32(v);
        attribute::update(heat_meter_endpoint_id, HEAT_METER_CLUSTER_ID, HM_ATTR_FLOW_TEMP_ID, &val);
    }
    if (d.has_return_temp)
    {
        nullable<int32_t> v; v = (int32_t)lround(d.return_temp_c * 100.0f); // 0.01 degC
        esp_matter_attr_val_t val = esp_matter_nullable_int32(v);
        attribute::update(heat_meter_endpoint_id, HEAT_METER_CLUSTER_ID, HM_ATTR_RETURN_TEMP_ID, &val);
    }
    if (d.has_power)
    {
        nullable<int64_t> v; v = (int64_t)llround(d.power_w * 1000.0f); // W -> mW
        esp_matter_attr_val_t val = esp_matter_nullable_int64(v);
        attribute::update(heat_meter_endpoint_id, HEAT_METER_CLUSTER_ID, HM_ATTR_POWER_ID, &val);
    }
}

static void mbus_poll_task(void *arg)
{
#if MBUS_MODE == MBUS_MODE_NKE_ONLY

    // Link bring-up: send SND_NKE and report whether the meter ACKs. No data
    // request, no parsing, nothing published to the Matter data model.
    while (true)
    {
        esp_err_t err = mbus_send_nke(MBUS_PRIMARY_ADDRESS);
        ESP_LOGI(TAG, "SND_NKE to 0x%02X -> %s", MBUS_PRIMARY_ADDRESS, esp_err_to_name(err));

        vTaskDelay(pdMS_TO_TICKS(MBUS_NKE_TEST_INTERVAL_MS));
    }

#else

    static uint8_t user[256];

    while (true)
    {
        size_t user_len = 0;
        esp_err_t err = mbus_request_data(MBUS_PRIMARY_ADDRESS, user, sizeof(user), &user_len);
        if (err == ESP_OK)
        {
#if MBUS_MODE == MBUS_MODE_TEST
            // Bench mode: dump the user block and log every data record we can
            // decode, whatever quantity it is. Nothing reaches Matter.
            ESP_LOG_BUFFER_HEXDUMP(TAG, user, user_len, ESP_LOG_INFO);
            mbus_parse_test(user, user_len);
#else
            heat_meter_data_t data;
            if (mbus_parse(user, user_len, &data) == ESP_OK)
            {
                ESP_LOGI(TAG,
                         "flow=%.3f m3/h energy=%.0f Wh volume=%.4f m3 Tflow=%.2f Tret=%.2f power=%.1f W",
                         data.has_flow ? data.flow_m3h : NAN,
                         data.has_energy ? data.energy_wh : NAN,
                         data.has_volume ? data.volume_m3 : NAN,
                         data.has_flow_temp ? data.flow_temp_c : NAN,
                         data.has_return_temp ? data.return_temp_c : NAN,
                         data.has_power ? data.power_w : NAN);

                // TODO: re-enable once bench testing against the slave HAT is
                // done. ScheduleLambda only stores a small closure (<= 24
                // bytes), so pass the snapshot by heap pointer, not by value.
                //
                // heat_meter_data_t *snapshot = new heat_meter_data_t(data);
                // chip::DeviceLayer::SystemLayer().ScheduleLambda([snapshot]() {
                //     publish_meter_data(*snapshot);
                //     delete snapshot;
                // });
            }
            else
            {
                ESP_LOGW(TAG, "Failed to parse M-Bus telegram");
            }
#endif
        }
        else
        {
            ESP_LOGW(TAG, "REQ_UD2 failed: %s", esp_err_to_name(err));
        }

#if MBUS_MODE == MBUS_MODE_TEST
        vTaskDelay(pdMS_TO_TICKS(MBUS_TEST_INTERVAL_MS));
#else
        vTaskDelay(pdMS_TO_TICKS(MBUS_POLL_INTERVAL_MS));
#endif
    }

#endif // MBUS_MODE
}

// --- Custom cluster construction -------------------------------------------
static void create_heat_meter_endpoint(node_t *node)
{
    endpoint_t *ep = endpoint::create(node, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(ep != nullptr, ESP_LOGE(TAG, "Failed to create heat meter endpoint"));
    endpoint::add_device_type(ep, HEAT_METER_DEVICE_TYPE_ID, HEAT_METER_DEVICE_TYPE_VER);

    cluster_t *hm = cluster::create(ep, HEAT_METER_CLUSTER_ID, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(hm != nullptr, ESP_LOGE(TAG, "Failed to create heat meter cluster"));

    uint16_t flags = ATTRIBUTE_FLAG_NULLABLE;
    attribute::create(hm, HM_ATTR_FLOW_ID, flags, esp_matter_nullable_float(nullable<float>()));
    attribute::create(hm, HM_ATTR_FLOW_TEMP_ID, flags, esp_matter_nullable_int32(nullable<int32_t>()));
    attribute::create(hm, HM_ATTR_RETURN_TEMP_ID, flags, esp_matter_nullable_int32(nullable<int32_t>()));
    attribute::create(hm, HM_ATTR_POWER_ID, flags, esp_matter_nullable_int64(nullable<int64_t>()));

    heat_meter_endpoint_id = endpoint::get_id(ep);
    ESP_LOGI(TAG, "Heat meter endpoint created: id %d", heat_meter_endpoint_id);
}

extern "C" void app_main()
{
    nvs_flash_init();

    mbus_uart_init();
    ESP_LOGI(TAG, "M-Bus UART initialized");

    // Root node (endpoint 0)
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Custom heat meter cluster.
    create_heat_meter_endpoint(node);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    esp_err_t err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    xTaskCreate(mbus_poll_task, "mbus_poll", 4096, NULL, 5, NULL);
}
