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
