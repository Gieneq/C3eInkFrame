#pragma once

#include <stdint.h>
#include <esp_err.h>

#define CLICK_EVENT_SHORT  0
#define CLICK_EVENT_LONG   1

typedef struct click_event_t {
    uint8_t type;
} click_event_t;

typedef enum gframe_sm_state_t {
    GFRAME_SM_STATE_PREIDLE = 0,
    GFRAME_SM_STATE_IDLE,
    GFRAME_SM_STATE_CAPTIVE_PORTAL_OPENING,
    GFRAME_SM_STATE_CAPTIVE_PORTAL_OPENED,
    GFRAME_SM_STATE_CAPTIVE_PORTAL_CLOSING
} gframe_sm_state_t;


esp_err_t gframe_create();

esp_err_t gframe_start();

void gframe_enque_shortclick();