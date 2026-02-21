#include "flight_fsm.h"

static int iabs32(int32_t v) {
  return (v < 0) ? (int)(-v) : (int)v;
}

static void transition(flight_fsm_t* fsm, flight_state_t next, uint32_t now_ms, flight_events_t* ev) {
  if (fsm->state == next) {
    return;
  }
  fsm->state = next;
  fsm->state_since_ms = now_ms;
  fsm->launch_votes = 0;
  fsm->coast_votes = 0;
  fsm->apogee_votes = 0;
  fsm->landed_votes = 0;
  ev->state_changed = 1u;
}

void flight_fsm_init(flight_fsm_t* fsm, uint32_t now_ms) {
  if (fsm == 0) {
    return;
  }
  fsm->state = FLIGHT_STATE_PAD;
  fsm->state_since_ms = now_ms;
  fsm->launch_ms = 0;
  fsm->max_alt_cm = 0;
  fsm->launch_votes = 0;
  fsm->coast_votes = 0;
  fsm->apogee_votes = 0;
  fsm->landed_votes = 0;
  fsm->fault_votes = 0;
}

flight_state_t flight_fsm_state(const flight_fsm_t* fsm) {
  if (fsm == 0) {
    return FLIGHT_STATE_FAULT;
  }
  return fsm->state;
}

void flight_fsm_step(flight_fsm_t* fsm,
                     uint32_t now_ms,
                     int32_t altitude_cm,
                     int16_t az_mg,
                     int16_t vz_cms,
                     int sensors_ok,
                     int arm_ok,
                     flight_events_t* events) {
  if ((fsm == 0) || (events == 0)) {
    return;
  }

  events->state_changed = 0u;
  events->launch = 0u;
  events->apogee = 0u;
  events->landed = 0u;

  if (altitude_cm > fsm->max_alt_cm) {
    fsm->max_alt_cm = altitude_cm;
  }

  if (!sensors_ok) {
    if (fsm->fault_votes < 2000u) {
      fsm->fault_votes++;
    }
  } else if (fsm->fault_votes > 0u) {
    fsm->fault_votes--;
  }

  if (fsm->fault_votes > 500u &&
      fsm->state != FLIGHT_STATE_PAD &&
      fsm->state != FLIGHT_STATE_ARMED &&
      fsm->state != FLIGHT_STATE_LANDED) {
    transition(fsm, FLIGHT_STATE_FAULT, now_ms, events);
    return;
  }

  switch (fsm->state) {
    case FLIGHT_STATE_PAD:
      if (arm_ok) {
        transition(fsm, FLIGHT_STATE_ARMED, now_ms, events);
      }
      break;

    case FLIGHT_STATE_ARMED:
      if (!arm_ok) {
        transition(fsm, FLIGHT_STATE_PAD, now_ms, events);
        break;
      }
      if (az_mg > 1700 && vz_cms > 150) {
        if (fsm->launch_votes < 1000u) {
          fsm->launch_votes++;
        }
      } else {
        fsm->launch_votes = 0u;
      }
      if (fsm->launch_votes >= 3u) {
        fsm->launch_ms = now_ms;
        transition(fsm, FLIGHT_STATE_BOOST, now_ms, events);
        events->launch = 1u;
      }
      break;

    case FLIGHT_STATE_BOOST:
      if (az_mg < 300) {
        if (fsm->coast_votes < 1000u) {
          fsm->coast_votes++;
        }
      } else {
        fsm->coast_votes = 0u;
      }

      if (fsm->coast_votes >= 8u || (now_ms - fsm->launch_ms) > 6000u) {
        transition(fsm, FLIGHT_STATE_COAST, now_ms, events);
      }
      break;

    case FLIGHT_STATE_COAST:
      if (vz_cms < -80 && fsm->max_alt_cm > 300) {
        if (fsm->apogee_votes < 1000u) {
          fsm->apogee_votes++;
        }
      } else {
        fsm->apogee_votes = 0u;
      }

      if (fsm->apogee_votes >= 5u) {
        transition(fsm, FLIGHT_STATE_APOGEE, now_ms, events);
        events->apogee = 1u;
      }
      break;

    case FLIGHT_STATE_APOGEE:
      if ((now_ms - fsm->state_since_ms) > 300u) {
        transition(fsm, FLIGHT_STATE_DESCENT, now_ms, events);
      }
      break;

    case FLIGHT_STATE_DESCENT:
      if ((iabs32(vz_cms) < 30) && (altitude_cm < 300)) {
        if (fsm->landed_votes < 2000u) {
          fsm->landed_votes++;
        }
      } else if (fsm->landed_votes > 0u) {
        fsm->landed_votes--;
      }

      if (fsm->landed_votes >= 250u) {
        transition(fsm, FLIGHT_STATE_LANDED, now_ms, events);
        events->landed = 1u;
      }
      break;

    case FLIGHT_STATE_LANDED:
    case FLIGHT_STATE_FAULT:
    default:
      break;
  }
}
