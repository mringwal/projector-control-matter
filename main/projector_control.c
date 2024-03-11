/*
 * Project Control
 *
 * Requirements:
 * - On and OFF LEDs with ON = 0 V
 * - Power button that can be triggered by pulling to GND (e.g. with button and pull-up to VCC)
 * - Power Off = pressing power button twice
 * - ESP32-C3, esp-idf v5.x
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

#include "projector_control.h"

const static char *TAG = "Projector";

// sample & control period
#define POLL_INTERVAL_MS  100

// ADC1 GPIOs/Channels
#define ADC_RED      ADC_CHANNEL_3
#define ADC_BLUE     ADC_CHANNEL_4

// Power Button
#define GPIO_BUTTON          GPIO_NUM_5

// LED is considered on if value below this, 1000/4096 = 1/4 * 3.3V = 0.825
#define LED_THRESHOLD 1000

// observe LEDs for this period to decide
#define LED_SAMPLE_PERIOD_MS   1000

// number of samples to observe
#define LED_SAMPLE_COUNT (LED_SAMPLE_PERIOD_MS / POLL_INTERVAL_MS)

// button presss duration
#define BUTTON_PRESS_MS 200

// button time between double press
#define BUTTON_DOUBLE_PRESS_MS 3000

// button hold off - wait after double press
#define BUTTON_HOLD_OFF_MS 10000

// single observation
typedef struct {
    bool blue;
    bool red;
} led_adc_sample_t;

static const char * projector_state_names[] = {
    "UNKNOWN",
    "OFF",
    "STARTING",
    "ON",
    "STOPPING"
};

static enum {
    CONTROL_STATE_IDLE,
    CONTROL_STATE_OFF_W2_RELEASE_BUTTON_1,
    CONTROL_STATE_OFF_W2_PRESS_BUTTON_2,
    CONTROL_STATE_W2_RELEASE_BUTTON,
    CONTROL_STATE_W4_HOLD_OFF,
} control_state;


// globals
static adc_oneshot_unit_handle_t adc1_handle;

static led_adc_sample_t app_monitor_samples[LED_SAMPLE_COUNT];
static char app_monitor_samples_text[LED_SAMPLE_COUNT+1];
static int app_monitor_samples_count;

static projector_state_t requested_state;
static projector_state_t current_state;

static uint32_t control_timer;

// --- ADC

static void app_adc_init(void){
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_11,
        // note: 11 is deprecated, use _12 after esp-idf update
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_RED, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_BLUE, &config));
}

static void app_adc_sample(led_adc_sample_t * result){
    int adc_red;
    int adc_blue;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_RED,  &adc_red));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_BLUE, &adc_blue));
    result->red  = adc_red  < LED_THRESHOLD;
    result->blue = adc_blue < LED_THRESHOLD;
    // ESP_LOGI(TAG, "RED: %u (%4d), BLUE: %u (%4d)", result->red, adc_red, result->blue, adc_blue);
}

// --- MONITOR

static void app_monitor_init(void){
    memset(&app_monitor_samples, 0, sizeof(app_monitor_samples));
    memset(&app_monitor_samples_text, 0, sizeof(app_monitor_samples_text));
    app_monitor_samples_count = 0;
    current_state = PROJECTOR_STATE_UNKNOWN;
}

static void app_monitor_add_sample(const led_adc_sample_t * result){
    int i;
    for (i=0;i<LED_SAMPLE_COUNT - 1;i++){
        app_monitor_samples[i] = app_monitor_samples[i+1];
        app_monitor_samples_text[i] = app_monitor_samples_text[i+1];
    }
    app_monitor_samples[i] = *result;
    if (result->blue && result->red){
        app_monitor_samples_text[i] = '*';
    } else if (result->blue){
        app_monitor_samples_text[i] = '+';
    } else if (result->red){
        app_monitor_samples_text[i] = '-';
    } else {
        app_monitor_samples_text[i] = ' ';
    }
    if (app_monitor_samples_count < LED_SAMPLE_COUNT){
        app_monitor_samples_count++;
    }
}

static projector_state_t app_monitor_get_state(void){
    // not enough data yet
    if (app_monitor_samples_count < LED_SAMPLE_COUNT){
        return PROJECTOR_STATE_UNKNOWN;
    }

    // count RED vs. BLUE samples
    int i;
    int num_blue = 0;
    int num_red  = 0;
    for (i=0;i<LED_SAMPLE_COUNT;i++){
        if (app_monitor_samples[i].blue) {
            num_blue++;
        }
        if (app_monitor_samples[i].red) {
            num_red++;
        }
    }

    // BLUE ON
    if (num_blue == LED_SAMPLE_COUNT){
        return PROJECTOR_STATE_ON;
    }

    // RED ON
    if (num_red == LED_SAMPLE_COUNT){
        return PROJECTOR_STATE_OFF;
    }

    // mixed / unexpected
    if ((num_blue > 0) && (num_red > 0)){
        return PROJECTOR_STATE_UNKNOWN;
    }

    // blue blinking
    if (num_blue > 0){
        return PROJECTOR_STATE_STARTING;
    }

    // red blinking
    if (num_red > 0){
        return PROJECTOR_STATE_STOPPING;
    }

    // Both LEDs off
    return PROJECTOR_STATE_UNKNOWN;
}

static void app_button_press(void){
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL<<GPIO_BUTTON;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_BUTTON, 0);
}

static void app_button_release(void){
    gpio_reset_pin(GPIO_BUTTON);
}

// --- CONTROL

static void app_control_init(void){
    requested_state = PROJECTOR_STATE_UNKNOWN;
}

static void app_control_task(void){
    switch (control_state){
        case CONTROL_STATE_IDLE:
            // only start transition in IDLE mode (projector on or off)
            switch (current_state){
                case PROJECTOR_STATE_OFF:
                    switch (requested_state){
                        case PROJECTOR_STATE_ON:
                            ESP_LOGI(TAG, "Press button once to turn it ON");
                            app_button_press();
                            control_timer = BUTTON_PRESS_MS / POLL_INTERVAL_MS;
                            control_state = CONTROL_STATE_W2_RELEASE_BUTTON;
                            break;
                        case PROJECTOR_STATE_OFF:
                            ESP_LOGI(TAG, "Requested state OFF == current state, ignore");
                            requested_state = PROJECTOR_STATE_UNKNOWN;
                            break;                            
                        default:
                            break;
                    }
                    break;
                case PROJECTOR_STATE_ON:
                    switch (requested_state){
                        case PROJECTOR_STATE_OFF:
                            ESP_LOGI(TAG, "Press button first time to turn it OFF");
                            app_button_press();
                            control_timer = BUTTON_PRESS_MS / POLL_INTERVAL_MS;
                            control_state = CONTROL_STATE_OFF_W2_RELEASE_BUTTON_1;
                            break;
                        case PROJECTOR_STATE_ON:
                            ESP_LOGI(TAG, "Requested state ON == current state, ignore");
                            requested_state = PROJECTOR_STATE_UNKNOWN;
                            break;                            
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;
        case CONTROL_STATE_OFF_W2_RELEASE_BUTTON_1:
            if (control_timer > 0){
                control_timer--;
            } else {
                ESP_LOGI(TAG, "Release button");
                app_button_release();
                ESP_LOGI(TAG, "Wait %u seconds before pressing again", BUTTON_DOUBLE_PRESS_MS / 1000);
                control_timer = BUTTON_DOUBLE_PRESS_MS / POLL_INTERVAL_MS;
                control_state = CONTROL_STATE_OFF_W2_PRESS_BUTTON_2;
            }
            break;
        case CONTROL_STATE_OFF_W2_PRESS_BUTTON_2:
            if (control_timer > 0){
                control_timer--;
            } else {
                ESP_LOGI(TAG, "Press button second time to turn it OFF");
                app_button_press();
                control_timer = BUTTON_PRESS_MS / POLL_INTERVAL_MS;
                control_state = CONTROL_STATE_W2_RELEASE_BUTTON;
            }
            break;
        case CONTROL_STATE_W2_RELEASE_BUTTON:
            if (control_timer > 0){
                control_timer--;
            } else {
                ESP_LOGI(TAG, "Release button");
                app_button_release();
                ESP_LOGI(TAG, "Wait %u seconds before trying again", BUTTON_HOLD_OFF_MS / 1000);
                control_timer = BUTTON_HOLD_OFF_MS / POLL_INTERVAL_MS;
                control_state = CONTROL_STATE_W4_HOLD_OFF;
            }
            break;
        case CONTROL_STATE_W4_HOLD_OFF:
            if (control_timer > 0){
                control_timer--;
            } else {
                ESP_LOGI(TAG, "Hold-off complete");
                control_state = CONTROL_STATE_IDLE;
                requested_state = PROJECTOR_STATE_UNKNOWN;
            }
            break;            
    }
}

// --- API

void projector_control_init(void){
    app_adc_init();
    app_monitor_init();
    app_control_init();
}

uint32_t projector_control_get_poll_interval_ms(void){
    return POLL_INTERVAL_MS;
}

void projector_control_poll(void){
    // get ADC sample and infer state
    led_adc_sample_t sample;
    app_adc_sample(&sample);
    app_monitor_add_sample(&sample);
    current_state = app_monitor_get_state();

    // control projector
    app_control_task();
}

projector_state_t projector_control_get_current_state(void){
    return current_state;
}

void projector_control_set_requested_state(projector_state_t state){
    requested_state = state;
}

projector_state_t projector_control_get_requested_state(void){
    return requested_state;
}

void projector_control_dump_state(void){
    // one line summary
    ESP_LOGI(TAG, "Samples: '%s', requested %s, current: %s",
        app_monitor_samples_text, 
        projector_state_names[(int)requested_state], 
        projector_state_names[(int)current_state]);

}
