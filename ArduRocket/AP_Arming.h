#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Rocket : public AP_Arming
{
public:
    AP_Arming_Rocket() : AP_Arming() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Rocket);

    bool pre_arm_checks(bool display_failure) override;
};
