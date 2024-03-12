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

#pragma once

#include <esp_err.h>
#include <esp_matter.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

/** Standard max values (used for remapping attributes) */
#define STANDARD_BRIGHTNESS 255
#define STANDARD_HUE 360
#define STANDARD_SATURATION 255
#define STANDARD_TEMPERATURE_FACTOR 1000000

/** Matter max values (used for remapping attributes) */
#define MATTER_BRIGHTNESS 254
#define MATTER_HUE 254
#define MATTER_SATURATION 254
#define MATTER_TEMPERATURE_FACTOR 1000000

/** Default attribute values used during initialization */
#define DEFAULT_POWER true
#define DEFAULT_BRIGHTNESS 64
#define DEFAULT_HUE 128
#define DEFAULT_SATURATION 254

typedef void *app_driver_handle_t;

/** Initialize the light driver
 *
 * This initializes the light driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_light_init();

/** Initialize the button driver
 *
 * This initializes the button driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_button_init();

/** Driver Update
 *
 * This API should be called to update the driver for the attribute being updated.
 * This is usually called from the common `app_attribute_update_cb()`.
 *
 * @param[in] endpoint_id Endpoint ID of the attribute.
 * @param[in] cluster_id Cluster ID of the attribute.
 * @param[in] attribute_id Attribute ID of the attribute.
 * @param[in] val Pointer to `esp_matter_attr_val_t`. Use appropriate elements as per the value type.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val);

/** Set defaults for light driver
 *
 * Set the attribute drivers to their default values from the created data model.
 *
 * @param[in] endpoint_id Endpoint ID of the driver.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t app_driver_light_set_defaults(uint16_t endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
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
