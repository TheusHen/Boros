#pragma once

#include <stdint.h>

typedef enum {
  FLIGHT_STATE_PAD = 0,
  FLIGHT_STATE_ARMED = 1,
  FLIGHT_STATE_BOOST = 2,
  FLIGHT_STATE_COAST = 3,
  FLIGHT_STATE_APOGEE = 4,
  FLIGHT_STATE_DESCENT = 5,
  FLIGHT_STATE_LANDED = 6,
  FLIGHT_STATE_FAULT = 7,
} flight_state_t;

typedef struct {
  flight_state_t state;
  uint32_t state_since_ms;
  uint32_t launch_ms;
  int32_t max_alt_cm;
  uint16_t launch_votes;
  uint16_t coast_votes;
  uint16_t apogee_votes;
  uint16_t landed_votes;
  uint16_t fault_votes;
} flight_fsm_t;

typedef struct {
  uint8_t state_changed;
  uint8_t launch;
  uint8_t apogee;
  uint8_t landed;
} flight_events_t;

void flight_fsm_init(flight_fsm_t* fsm, uint32_t now_ms);
flight_state_t flight_fsm_state(const flight_fsm_t* fsm);
void flight_fsm_step(flight_fsm_t* fsm,
                     uint32_t now_ms,
                     int32_t altitude_cm,
                     int16_t az_mg,
                     int16_t vz_cms,
                     int sensors_ok,
                     int arm_ok,
                     flight_events_t* events);
