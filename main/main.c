#include "esp_check.h"
#include "esp_err.h"
#include "string.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "main.h"
#include "esp_ieee802154.h"
#include "esp_mac.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

static const char *TAG = "ESP_ZB_SPACEBALLS";

// Spaceballs - The Switch!
static char modelid[] = {24, 'S', 'p', 'a', 'c', 'e', 'b', 'a', 'l', 'l', 's', ' ', '-', ' ', 'T', 'h', 'e', ' ', 'S', 'w', 'i', 't', 'c', 'h', '!'};
static char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

#define RESET_GPIO_PIN 23

#define SB_ADC_CHANNEL ADC_CHANNEL_6
#define SB_ADC_ATTEN   ADC_ATTEN_DB_12

#define INDEPENDENT_SWITCHES SPACEBALLS_INDEPENDENT_SWITCHES
#define BUTTON_1        (1<<0) // Light
#define BUTTON_2        (1<<1) // Ridiculous
#define BUTTON_3        (1<<2) // Ludicrous
#define BUTTON_4        (1<<3) // Plaid
#define BUTTON_5        (1<<4) // Go
#define BUTTON_6        (1<<5) // Additional switch
#define BUTTON_GPIO_5   20     // Go button GPIO

trigger_range trigger_ranges[] = {
    {
        .min = 2600,
        .max = 3200,
        .button = BUTTON_1, // Light
        .name = "Light",
    },
    {
        .min = 2100,
        .max = 2599,
        .button = BUTTON_2, // Ridiculous
        .name = "Ridiculous",
    },
    {
        .min = 1200,
        .max = 2099,
        .button = BUTTON_3, // Ludicrous
        .name = "Ludicrous",
    },
    {
        .min = 0,
        .max = 1199,
        .button = BUTTON_4, // Plaid
        .name = "Plaid",
    },
};

#if CONFIG_SPACEBALLS_ADDITIONAL_SWITCH
#define TOTAL_BUTTONS           ((sizeof(trigger_ranges) / sizeof(trigger_range)) + 2)
#define GO_BUTTON_NUM           TOTAL_BUTTONS - 2
#define ADDITIONAL_BUTTON_NUM   TOTAL_BUTTONS - 1
#else
#define TOTAL_BUTTONS   ((sizeof(trigger_ranges) / sizeof(trigger_range)) + 1)
#define GO_BUTTON_NUM           TOTAL_BUTTONS - 1
#endif

// Internal button state
static uint8_t button_state = 0, previous_state = 0;
static uint8_t button_attrs[TOTAL_BUTTONS];
static uint8_t button_attrs_previous[TOTAL_BUTTONS];
static bool zigbee_initialized = false;

// Send updates of attribute changes
static void send_reports(uint8_t force_report)
{
    // Report status of the switches
    // Technically attribute reporting should take care of this, but to minimize
    // latency, we're just sending the updates ourselves.
    esp_zb_lock_acquire(portMAX_DELAY);
    for (int endpoint_id = 0; endpoint_id < TOTAL_BUTTONS; endpoint_id++) {
        if (force_report || button_attrs_previous[endpoint_id] != button_attrs[endpoint_id]) {
            esp_zb_zcl_report_attr_cmd_t report_attr_cmd;

            ESP_LOGI(TAG, "Reporting status change for endpoint ID %d\n", endpoint_id + 1);
            report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
            report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
            report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
            report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            report_attr_cmd.zcl_basic_cmd.src_endpoint = endpoint_id + 1;

            esp_zb_zcl_status_t state = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
            if (state != ESP_ZB_ZCL_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Failed to report on/off status for endpoint %d!", endpoint_id + 1);
            }
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    }
    esp_zb_lock_release();

    for (int endpoint_id = 0; endpoint_id < TOTAL_BUTTONS; endpoint_id++) {
        button_attrs_previous[endpoint_id] = button_attrs[endpoint_id];
    }
}

// This function updates the state of the switches in the on/off clusters
static void on_off_state_handler(uint8_t button_state)
{
    // Just wait until endpoints are initialized - probably not the best way
    // but I'm not sure which signal is the best
    for (int endpoint_id = 0; endpoint_id < TOTAL_BUTTONS; endpoint_id++) {
        // Handle go button
#if CONFIG_SPACEBALLS_ADDITIONAL_SWITCH
        if (endpoint_id == ADDITIONAL_BUTTON_NUM) {
            button_attrs[endpoint_id] = button_state & (1 << endpoint_id) ? 1 : 0;
            continue;
        }
#endif
        if (endpoint_id == GO_BUTTON_NUM) {
            button_attrs[endpoint_id] = button_state & (1 << endpoint_id) ? 1 : 0;
        } else {
            // Keep all previous buttons on for the pot
            if (button_state & (1 << endpoint_id)) {
#if INDEPENDENT_SWITCHES
                for (int i = endpoint_id; i >= 0; i--) {
                    button_attrs[i] = 1;
                }
#else
                button_attrs[endpoint_id] = 1;
#endif
            } else {
                button_attrs[endpoint_id] = 0;
            }
        }
    }

    // Update endpoint on/off attributes
    esp_zb_lock_acquire(portMAX_DELAY);
    for (int endpoint_id = 0; endpoint_id < TOTAL_BUTTONS; endpoint_id++) {
        ESP_LOGI(TAG, "Endpoint %d, on/off status: %d", endpoint_id + 1, button_attrs[endpoint_id]);
        if (button_attrs_previous[endpoint_id] != button_attrs[endpoint_id]) {
            esp_zb_zcl_status_t state = esp_zb_zcl_set_attribute_val(endpoint_id + 1,
                                                                     ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                                     ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                                                     &button_attrs[endpoint_id],
                                                                     false);

            if (state != ESP_ZB_ZCL_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Failed to update on/off status for endpoint %d!", endpoint_id + 1);
            }
        }
    }
    esp_zb_lock_release();

    if (zigbee_initialized) {
        send_reports((uint8_t)false);
    }
}

static void handle_button_state_change(uint8_t previous_state, uint8_t button_state)
{
    ESP_LOGI(TAG, "Scheduling attribute update...");
    esp_zb_scheduler_alarm((esp_zb_callback_t)on_off_state_handler, button_state, 100);
}

// Handles the rotary potentiometer and the throw switch
static void esp_controls_task(void *pvParameters)
{
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    // Initialize button attributes to zero
    for (int endpoint_id = 0; endpoint_id < TOTAL_BUTTONS; endpoint_id++) {
        button_attrs[endpoint_id] = 0;
        button_attrs_previous[endpoint_id] = 0;
    }

    ESP_LOGI(TAG, "Initializing GO switch GPIO %d", BUTTON_GPIO_5);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO_5);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (gpio_get_level(BUTTON_GPIO_5) == 1) {
        button_state |= BUTTON_5;
        ESP_LOGI(TAG, "GO button is ON! (Button state %d)", button_state);
    } else {
        ESP_LOGI(TAG, "GO button is OFF");
    }

#if CONFIG_SPACEBALLS_ADDITIONAL_SWITCH
    if (gpio_get_level(CONFIG_SPACEBALLS_ADDITIONAL_SWITCH_GPIO) == 1) {
        button_state |= BUTTON_6;
        ESP_LOGI(TAG, "Additional button is ON! (Button state %d)", button_state);
    } else {
        ESP_LOGI(TAG, "Additional button is OFF");
    }
#endif

    ESP_LOGI(TAG, "Initializing ADC, bits=%d, channel=%d, atten=%d", ADC_BITWIDTH_DEFAULT, SB_ADC_CHANNEL, SB_ADC_ATTEN);
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    int gpio = 0;
    adc_oneshot_channel_to_io(ADC_UNIT_1, SB_ADC_CHANNEL, &gpio);
    ESP_LOGI(TAG, "Using ADC connected to GPIO %d", gpio);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = SB_ADC_ATTEN,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SB_ADC_CHANNEL, &config));

    int data = 0;
    for (int i = 0; i < 5; i++) {
        int init_data = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SB_ADC_CHANNEL, &init_data));
        data += init_data;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    data = data / 5;
    for (int i = 0; i < sizeof(trigger_ranges) / sizeof(trigger_range); i++) {
        if (data >= trigger_ranges[i].min && data <= trigger_ranges[i].max) {
            button_state |= trigger_ranges[i].button;
        }
    }

    if (gpio_get_level(BUTTON_GPIO_5) == 1) {
            button_state |= BUTTON_5;
    }

#if CONFIG_SPACEBALLS_ADDITIONAL_SWITCH
    if (gpio_get_level(CONFIG_SPACEBALLS_ADDITIONAL_SWITCH_GPIO) == 1) {
        button_state |= BUTTON_6;
    }
#endif

    ESP_LOGI(TAG, "Initial button state is: %s", trigger_ranges[i].name);
    
    while (1) {
        if (button_state != previous_state) {
            ESP_LOGI(TAG, "Button state changed to %d.", button_state);
            handle_button_state_change(previous_state, button_state);
        }
        previous_state = button_state;

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SB_ADC_CHANNEL, &data));

        if (gpio_get_level(BUTTON_GPIO_5) == 1) {
            button_state |= BUTTON_5;
        } else {
            button_state = button_state & ~(BUTTON_5);
        }

#if CONFIG_SPACEBALLS_ADDITIONAL_SWITCH
        if (gpio_get_level(CONFIG_SPACEBALLS_ADDITIONAL_SWITCH_GPIO) == 1) {
            button_state |= BUTTON_6;
        } else {
            button_state = button_state & ~(BUTTON_6);
        }
#endif

        for (int i = 0; i < sizeof(trigger_ranges) / sizeof(trigger_range); i++) {
            if (data >= trigger_ranges[i].min && data <= trigger_ranges[i].max) {
                button_state |= trigger_ranges[i].button;
            } else {
                button_state = button_state & ~(trigger_ranges[i].button);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,, TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED:
        ESP_LOGI(TAG, "Zigbee: ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED");
        break;
    case ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED");
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING_CANCELLED:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_STEERING_CANCELLED");
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION_CANCELLED:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_FORMATION_CANCELLED");
        break;
    case ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE");
        break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            ESP_LOGI(TAG, "Reset device");
        }
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status != ESP_OK) {
            ESP_LOGW(TAG, "Stack %s failure with %s status, steering", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        } else {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                // Just send an update, in case coordinator is out of sync
                zigbee_initialized = true;
                esp_zb_scheduler_alarm((esp_zb_callback_t)send_reports, (uint8_t)true, 1000);
            }
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zigbee_initialized = true;
            esp_zb_scheduler_alarm((esp_zb_callback_t)send_reports, (uint8_t)true, 1000);

        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? * (uint8_t *)variable->attribute.data.value : 0);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), attribute(0x%x)", message->info.status, message->info.cluster,
                 variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

// This is just an informational function
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback: report attr", callback_id);
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback: read attr", callback_id);
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback: config response", callback_id);
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

// Checks if the reset GPIO is bridged and performs a factory
// reset
void check_reset_gpio(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RESET_GPIO_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    for (int i = 0; i < 5; i++) {
        int level = gpio_get_level(RESET_GPIO_PIN);
        if (level == 1) {
            ESP_LOGW(TAG, "Pin %d is high, performing factory reset...", RESET_GPIO_PIN);
            if (!esp_zb_bdb_is_factory_new()) {
                esp_zb_factory_reset();
            } else {
                ESP_LOGI(TAG, "Skipping factory reset, since we are already factory reset.");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    ESP_LOGI(TAG, "Not factory resetting, continuing startup...");
}

// Main Zigbee setup
static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    uint8_t zcl_version, null_values;
    zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    null_values = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE;

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    // Create 5 endpoints (4 pot "buttons" and the go switch)
    for (int endpoint_id = 0; endpoint_id < TOTAL_BUTTONS; endpoint_id++) {
        // Basic cluster
        esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &null_values));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]));

        // Identify cluster
        esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
        ESP_ERROR_CHECK(esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &null_values));

        // On-off cluster
        esp_zb_on_off_cluster_cfg_t on_off_cfg;
        on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
        esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

        esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

        // On-off clusters are in server role, though we won't accept attribute updates.
        // From Zigbee cluster specifications:
        // Conversely, the command that facilitates dynamic attribute reporting, i.e., the report attribute command
        // is (typically) sent from the server device (as typically this is where the attribute data itself is stored)
        // and sent to the client device that has been bound to the server device

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

        esp_zb_endpoint_config_t endpoint_cfg = {
            .endpoint = (endpoint_id + 1),
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
            .app_device_version = 1,
        };
        esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_cfg);
    }

    esp_zb_device_register(esp_zb_ep_list);

    // Configure reporting for all endpoints
    for (int endpoint_id = 0; endpoint_id < TOTAL_BUTTONS; endpoint_id++) {
        esp_zb_zcl_reporting_info_t reporting_info = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
            .ep = endpoint_id + 1,
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .u.send_info.min_interval = 1,
            .u.send_info.max_interval = 0,
            .u.send_info.def_min_interval = 1,
            .u.send_info.def_max_interval = 0,
            .u.send_info.delta.u16 = 100,
            .attr_id = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };

        ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&reporting_info));
    }
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    // esp_zb_set_secondary_network_channel_set(ESP_ZB_SECONDARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(true));

    check_reset_gpio();

    esp_zb_main_loop_iteration();
}

// Main function
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    uint8_t ieeeMac[8] = { 0 };
    esp_read_mac(ieeeMac, ESP_MAC_IEEE802154);
    ESP_LOGI(TAG, "Zigbee MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", ieeeMac[0], ieeeMac[1], ieeeMac[2], ieeeMac[3], ieeeMac[4], ieeeMac[5], ieeeMac[6], ieeeMac[7]);

    ESP_LOGI(TAG, "nvs_flash_init()");
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(esp_controls_task, "Controls", 4096, NULL, 5, NULL);
}
