#include <esp_err.h>
#include <esp_log.h>
#include <inttypes.h>
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
#include "freertos/event_groups.h"
#include "freertos/task.h"

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_ieee802154.h"
#endif

#include "heat_meter_cluster.h"
#include "mbus.h"
#include "mbus_parser.h"
#include "reset_button.h"

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

// Matter's Flow Measurement cluster (0x0404) carries MeasuredValue as a uint16
// where MeasuredValue = 10 x flow in m^3/h -- i.e. one count is 0.1 m^3/h, or
// 100 l/h. The meter reports l/h, so this view of the reading loses three
// significant figures; HM_ATTR_FLOW_ID on the custom cluster keeps the accurate
// one. The standard cluster exists so an off-the-shelf controller can read the
// flow at all without knowing anything about the custom cluster.
#define FLOW_MEASUREMENT_PER_M3H 10.0f

// Range advertised by MinMeasuredValue/MaxMeasuredValue, in m^3/h. Sized for a
// residential MULTICAL 403 (qp 1.5, qs 3.0 m^3/h); raise the max for a larger
// meter variant, or readings above it will be clamped.
#define FLOW_SENSOR_MIN_M3H 0.0f
#define FLOW_SENSOR_MAX_M3H 3.0f

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

// 802.15.4 transmit power. The C6 will go to +20 dBm, which is far more than a
// domestic Thread mesh needs and more than this board's 3V3 LDO (U3, MCP1700,
// 250 mA) has headroom for -- the board brownout-resets when the radio comes up
// while BLE is still connected. IDF 5.5 exposes no Kconfig for this, so it is
// clamped at runtime. Raise it if the mesh turns out to need the range.
#define THREAD_TX_POWER_DBM 9

// Gate for the M-Bus poll task. Set only while talking to the meter is both
// safe and useful: a fabric exists and no commissioning is in flight. The M-Bus
// master's 36 V boost (U2) is hard-enabled and its input current is drawn from
// the same USB +5V rail as the ESP32's regulator, so polling the meter during
// commissioning piles load onto the supply at exactly the moment the radio
// needs it. There is also nothing to publish to before a fabric exists.
#define APP_EVENT_MBUS_ENABLED BIT0

static EventGroupHandle_t s_app_events;

// Inputs to the gate. Every writer runs on the Matter thread -- app_event_cb or
// a ScheduleLambda -- so they need no locking.
static bool s_commissioned = false;
static bool s_window_open = false;
static bool s_session_in_flight = false;

// Recompute APP_EVENT_MBUS_ENABLED from the three inputs above. Matter thread.
static void update_mbus_gate()
{
    const bool enabled = s_commissioned && !s_window_open && !s_session_in_flight;
    const bool was_enabled = (xEventGroupGetBits(s_app_events) & APP_EVENT_MBUS_ENABLED) != 0;
    if (enabled == was_enabled)
    {
        return;
    }

    if (enabled)
    {
        ESP_LOGI(TAG, "M-Bus polling enabled");
        xEventGroupSetBits(s_app_events, APP_EVENT_MBUS_ENABLED);
    }
    else
    {
        ESP_LOGI(TAG, "M-Bus polling suspended (window=%d session=%d fabric=%d)",
                 s_window_open, s_session_in_flight, s_commissioned);
        xEventGroupClearBits(s_app_events, APP_EVENT_MBUS_ENABLED);
    }
}

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

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD  
static void apply_thread_tx_power()
{
    ESP_LOGI(TAG, "802.15.4 TX power before clamp: %d dBm", esp_ieee802154_get_txpower());

    esp_err_t err = esp_ieee802154_set_txpower(THREAD_TX_POWER_DBM);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to set 802.15.4 TX power: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "802.15.4 TX power now: %d dBm", esp_ieee802154_get_txpower());
}
#endif

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type)
    {
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        s_window_open = true;
        update_mbus_gate();
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        // Careful: this also fires the instant PASE is established, with the
        // whole of AddNOC/CASE/CommissioningComplete still to come. Resuming on
        // it alone would put the bus back under load at the worst moment, which
        // is what s_session_in_flight below is for.
        ESP_LOGI(TAG, "Commissioning window closed");
        s_window_open = false;
        update_mbus_gate();
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        s_session_in_flight = true;
        update_mbus_gate();
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        s_session_in_flight = false;
        update_mbus_gate();
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        s_commissioned = true;
        s_session_in_flight = false;
        update_mbus_gate();
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        // Commissioning gave up part-way. The SDK reopens the window for a
        // retry, so the gate normally stays shut on the window flag alone.
        ESP_LOGW(TAG, "Fail-safe timer expired");
        s_session_in_flight = false;
        update_mbus_gate();
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        s_commissioned = chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
        update_mbus_gate();
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

// Scale a flow rate in m^3/h to the Flow Measurement cluster's 0.1 m^3/h
// counts, clamped to the range the cluster advertises. A reading outside that
// range means FLOW_SENSOR_MAX_M3H is wrong for the meter on the bus, so say so
// rather than silently reporting a wrong number.
static uint16_t flow_to_measured_value(float flow_m3h)
{
    if (flow_m3h < FLOW_SENSOR_MIN_M3H || flow_m3h > FLOW_SENSOR_MAX_M3H)
    {
        ESP_LOGW(TAG, "flow %.3f m3/h is outside the advertised range %.1f..%.1f -- clamping",
                 flow_m3h, FLOW_SENSOR_MIN_M3H, FLOW_SENSOR_MAX_M3H);
        flow_m3h = (flow_m3h < FLOW_SENSOR_MIN_M3H) ? FLOW_SENSOR_MIN_M3H : FLOW_SENSOR_MAX_M3H;
    }
    return (uint16_t)lroundf(flow_m3h * FLOW_MEASUREMENT_PER_M3H);
}

// Called on the Matter/CHIP thread (via ScheduleLambda) so attribute::update is
// safe to call.
static void publish_meter_data(const heat_meter_data_t &d)
{
    // Flow goes to both endpoints: full precision on the custom cluster, and
    // the standard Flow Measurement cluster for interoperability.
    if (d.has_flow)
    {
        nullable<float> v; v = d.flow_m3h;
        esp_matter_attr_val_t val = esp_matter_nullable_float(v);
        attribute::update(heat_meter_endpoint_id, HEAT_METER_CLUSTER_ID, HM_ATTR_FLOW_ID, &val);

        nullable<uint16_t> mv; mv = flow_to_measured_value(d.flow_m3h);
        esp_matter_attr_val_t mval = esp_matter_nullable_uint16(mv);
        attribute::update(flow_endpoint_id, FlowMeasurement::Id,
                          FlowMeasurement::Attributes::MeasuredValue::Id, &mval);
    }
    else
    {
        // Null is the cluster's "unknown", and the right answer: leaving the
        // last reading in place would have a controller showing a flow rate the
        // meter has stopped reporting.
        esp_matter_attr_val_t mval = esp_matter_nullable_uint16(nullable<uint16_t>());
        attribute::update(flow_endpoint_id, FlowMeasurement::Id,
                          FlowMeasurement::Attributes::MeasuredValue::Id, &mval);
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

// Block until the gate is open. Called at the top of every poll iteration, so a
// commissioning window opening mid-run parks the bus until commissioning is
// finished rather than only gating the first poll.
static void mbus_wait_until_enabled()
{
    if (xEventGroupGetBits(s_app_events) & APP_EVENT_MBUS_ENABLED)
    {
        return;
    }

    ESP_LOGI(TAG, "M-Bus poll task waiting on the gate");
    xEventGroupWaitBits(s_app_events, APP_EVENT_MBUS_ENABLED, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "M-Bus poll task released");
}

static void mbus_poll_task(void *arg)
{
    // Hold off until commissioned and idle -- see APP_EVENT_MBUS_ENABLED.
    mbus_wait_until_enabled();

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    // Re-assert after commissioning: if OpenThread pushed its own value while
    // joining, this logs it and puts the clamp back.
    apply_thread_tx_power();
#endif

#if MBUS_MODE == MBUS_MODE_NKE_ONLY

    // Link bring-up: send SND_NKE and report whether the meter ACKs. No data
    // request, no parsing, nothing published to the Matter data model.
    while (true)
    {
        mbus_wait_until_enabled();

        esp_err_t err = mbus_send_nke(MBUS_PRIMARY_ADDRESS);
        ESP_LOGI(TAG, "SND_NKE to 0x%02X -> %s", MBUS_PRIMARY_ADDRESS, esp_err_to_name(err));

        vTaskDelay(pdMS_TO_TICKS(MBUS_NKE_TEST_INTERVAL_MS));
    }

#else

    static uint8_t user[256];

    while (true)
    {
        mbus_wait_until_enabled();

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

                // ScheduleLambda only stores a small closure (<= 24 bytes), so
                // pass the snapshot by heap pointer, not by value.
                heat_meter_data_t *snapshot = new heat_meter_data_t(data);
                chip::DeviceLayer::SystemLayer().ScheduleLambda([snapshot]() {
                    publish_meter_data(*snapshot);
                    delete snapshot;
                });
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

    // Descriptor (0x001D) is mandatory on every endpoint, and its DeviceTypeList
    // attribute is the only thing that tells a controller what device type the
    // endpoint is. add_device_type() below only records the id in esp-matter's
    // own endpoint struct -- the Descriptor server is what publishes it.
    //
    // The device-type helpers (flow_sensor::create and the rest) get this for
    // free because they go through common::create(), which creates the
    // descriptor cluster first. endpoint::create() is the raw call and does not,
    // so a custom endpoint has to add it by hand.
    cluster::descriptor::config_t descriptor_config;
    cluster_t *descriptor = cluster::descriptor::create(ep, &descriptor_config, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(descriptor != nullptr, ESP_LOGE(TAG, "Failed to create descriptor cluster"));

    esp_err_t dt_err = endpoint::add_device_type(ep, HEAT_METER_DEVICE_TYPE_ID, HEAT_METER_DEVICE_TYPE_VER);
    ABORT_APP_ON_FAILURE(dt_err == ESP_OK, ESP_LOGE(TAG, "Failed to add heat meter device type"));

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

// --- Standard Flow Measurement endpoint -------------------------------------
// Matter Flow Sensor (device type 0x0306), which brings cluster 0x0404 with it.
// This is the interoperable view of the meter's flow rate: any controller can
// read it with no knowledge of the custom Heat Meter cluster, at the cost of
// 0.1 m^3/h resolution.
static void create_flow_sensor_endpoint(node_t *node)
{
    flow_sensor::config_t config;
    config.flow_measurement.min_measured_value =
        (uint16_t)lroundf(FLOW_SENSOR_MIN_M3H * FLOW_MEASUREMENT_PER_M3H);
    config.flow_measurement.max_measured_value =
        (uint16_t)lroundf(FLOW_SENSOR_MAX_M3H * FLOW_MEASUREMENT_PER_M3H);
    // measured_value is left null until the first telegram is decoded.

    endpoint_t *ep = flow_sensor::create(node, &config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(ep != nullptr, ESP_LOGE(TAG, "Failed to create flow sensor endpoint"));

    flow_endpoint_id = endpoint::get_id(ep);
    ESP_LOGI(TAG, "Flow sensor endpoint created: id %d", flow_endpoint_id);
}

extern "C" void app_main()
{
    nvs_flash_init();

    // Must exist before esp_matter::start(), which can deliver events, and
    // before the poll task is created.
    s_app_events = xEventGroupCreate();
    ABORT_APP_ON_FAILURE(s_app_events != nullptr, ESP_LOGE(TAG, "Failed to create event group"));

    mbus_uart_init();
    ESP_LOGI(TAG, "M-Bus UART initialized");

    // Root node (endpoint 0)
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Custom heat meter cluster.
    create_heat_meter_endpoint(node);

    // Standard Flow Measurement. Created after the heat meter endpoint on
    // purpose: endpoint ids are handed out in creation order, so adding this
    // first would shift the heat meter endpoint and break controllers that are
    // already commissioned against it.
    create_flow_sensor_endpoint(node);

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

    // Not fatal: a dead button should not stop the meter from being read.
    err = reset_button_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init reset button: %s", esp_err_to_name(err));
    }

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    apply_thread_tx_power();
#endif

    // A node that is already commissioned gets no kCommissioningComplete on a
    // reboot, so seed the gate from the live state here instead. The window may
    // already have been opened by esp_matter::start() -- re-reading it rather
    // than assuming closed covers a re-commissioning of a node that still holds
    // an older fabric. Run on the Matter thread because it owns both objects.
    chip::DeviceLayer::SystemLayer().ScheduleLambda([]() {
        s_commissioned = chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
        s_window_open = chip::Server::GetInstance().GetCommissioningWindowManager().IsCommissioningWindowOpen();
        ESP_LOGI(TAG, "Startup gate: fabric=%d window=%d", s_commissioned, s_window_open);
        update_mbus_gate();
    });

    xTaskCreate(mbus_poll_task, "mbus_poll", 4096, NULL, 5, NULL);
}
