/*
 * Copyright (C) 2024 Matthias Ringwald
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/clusters/diagnostic-logs-server/diagnostic-logs-server.h>
#include <diagnostic-logs-provider-delegate-impl.h>
#include <app/server/Server.h>

#include <platform/CHIPDeviceLayer.h>

#include "projector_control.h"

static const char *TAG = "app_main";
static uint16_t light_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;

using namespace chip;

using namespace chip::app::Clusters;

static void projector_control_timer(System::Layer * systemLayer, void * data);

constexpr auto k_timeout_seconds = 300;

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

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

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}


// check before updating state
static void app_projector_update_state(bool on){
    uint16_t endpoint_id  = light_endpoint_id;
    uint32_t cluster_id   = OnOff::Id;
    uint32_t attribute_id = OnOff::Attributes::OnOff::Id;

    node_t *node = node::get();
    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
    cluster_t *cluster = cluster::get(endpoint, cluster_id);
    attribute_t *attribute = attribute::get(cluster, attribute_id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);

    // ESP_LOGI(TAG, "Update state triggered by projector, current state %u, new state %u", val.val.b, (int) on);

    if (val.val.b != on){
        val.val.b = on;
        ESP_LOGI(TAG, "Update state triggered by projector, new state %u", (int) on);
        attribute::update(endpoint_id, cluster_id, attribute_id, &val);
    }
}

static void app_projector_set_state(bool on){
    if (on){
        projector_control_set_requested_state(PROJECTOR_STATE_ON);
    } else {
        projector_control_set_requested_state(PROJECTOR_STATE_OFF);
    }
}


static void projector_control_timer(System::Layer * systemLayer, void * data){
    // update projector statemachine
    projector_control_poll();

    // dump state
    // projector_control_dump_state();

    // update state upon transitions triggered via projector
    if (projector_control_get_requested_state() == PROJECTOR_STATE_UNKNOWN){
        switch (projector_control_get_current_state()){
            case PROJECTOR_STATE_STOPPING:
                app_projector_update_state(false);
                break;
            case PROJECTOR_STATE_STARTING:
                app_projector_update_state(true);
                break;
            default:
                break;
        }
    }

    // schedule next poll
    const uint32_t poll_interval_ms = projector_control_get_poll_interval_ms();
    (void) DeviceLayer::SystemLayer().StartTimer(System::Clock::Milliseconds32(poll_interval_ms), projector_control_timer, data);

}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;
    if (type == PRE_UPDATE) {
        if (endpoint_id == light_endpoint_id) {
            if (cluster_id == OnOff::Id) {
                if (attribute_id == OnOff::Attributes::OnOff::Id) {
                    if (val->val.b){
                        ESP_LOGI(TAG, "Request ON");
                        app_projector_set_state(true);
                    } else {
                        ESP_LOGI(TAG, "Request OFF");
                        app_projector_set_state(false);
                    }
                }
           }
        }
    }

    return err;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize projector control driver */
    projector_control_init();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;

    // node handle can be used to add/modify other endpoints.
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // add diagnostic logs cluster on root endpoint
    cluster::diagnostic_logs::config_t diag_logs_config;
    endpoint_t *root_ep = endpoint::get(node, 0); // get the root node ep
    cluster::diagnostic_logs::create(root_ep, &diag_logs_config, CLUSTER_FLAG_SERVER);

    on_off_light::config_t light_config;
    // endpoint handles can be used to add/modify clusters.
    endpoint_t *endpoint = on_off_light::create(node, &light_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create on/off endpoint"));

    light_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Light created with endpoint_id %d", light_endpoint_id);

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

    /** Start Timer - requires locking */
    const uint32_t poll_interval_ms = projector_control_get_poll_interval_ms();
    lock::chip_stack_lock(portMAX_DELAY);
    (void) DeviceLayer::SystemLayer().StartTimer(System::Clock::Milliseconds32(poll_interval_ms), projector_control_timer, NULL);
    lock::chip_stack_unlock();

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif
}

using namespace chip::app::Clusters::DiagnosticLogs;
void emberAfDiagnosticLogsClusterInitCallback(chip::EndpointId endpoint)
{
    auto & logProvider = LogProvider::GetInstance();
    DiagnosticLogsServer::Instance().SetDiagnosticLogsProviderDelegate(endpoint, &logProvider);
}
