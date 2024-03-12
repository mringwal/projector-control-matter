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

/*
 * Projector Control API
 *
 */
#ifndef PROJECTOR_CONTROL_H
#define PROJECTOR_CONTROL_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

// possible projector power states
typedef enum {
    PROJECTOR_STATE_UNKNOWN = 0,
    PROJECTOR_STATE_OFF,
    PROJECTOR_STATE_STARTING,
    PROJECTOR_STATE_ON,
    PROJECTOR_STATE_STOPPING,
} projector_state_t;


/**
 * Init component
 */
void projector_control_init(void);

/**
 * Get poll interval
 */
uint32_t projector_control_get_poll_interval_ms(void);

/**
 * Update state
 */
void projector_control_poll(void);

/**
 * Get current state
 */
projector_state_t projector_control_get_current_state(void);

/**
 * Set requested state
 */
void projector_control_set_requested_state(projector_state_t state); 

/**
 * Get planned state, returns PROJECTOR_STATE_UNKNOWN if steady
 */
projector_state_t projector_control_get_requested_state(void); 

/**
 * Dump state
 */
void projector_control_dump_state(void);

#if defined __cplusplus
}
#endif

#endif /* PROJECTOR_CONTROL_H */
