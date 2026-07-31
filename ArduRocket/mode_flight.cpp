#include "ArduRocket.h"

/*
  FLIGHT mode stub. The real body -- driving RocketStateMachine and
  RocketAttitudeControl -- lands in Task 3 of the Phase 2b plan. Entry is
  refused until then so the vehicle cannot pretend to fly.
 */

bool ModeFlight::_enter()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "RKT: FLIGHT not implemented yet");
    return false;
}

void ModeFlight::_exit()
{
    rocket.set_fins(0.0f, 0.0f);
}

void ModeFlight::update()
{
    rocket.set_fins(0.0f, 0.0f);
}

void ModeFlight::announce()
{
    gcs().send_text(MAV_SEVERITY_INFO, "RKT: %s",
                    RocketControl::RocketStateMachine::phase_name(_fsm.phase()));
}
