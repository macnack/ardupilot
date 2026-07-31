#include "ArduRocket.h"

bool AP_Arming_Rocket::pre_arm_checks(bool display_failure)
{
    // rocket-specific pre-arm checks land in Task 5 of the Phase 2b plan
    return AP_Arming::pre_arm_checks(display_failure);
}
