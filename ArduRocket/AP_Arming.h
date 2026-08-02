#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Rocket : public AP_Arming
{
public:
    AP_Arming_Rocket() : AP_Arming() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Rocket);

    bool pre_arm_checks(bool display_failure) override;

    // Mirror the arming state into HAL soft-armed. Every vehicle does this
    // itself -- AP_Arming does not (ArduPlane/AP_Arming.cpp:390). Without it
    // hal.util->get_soft_armed() is permanently false, which silently breaks
    // every consumer of it on this vehicle.
    void update_soft_armed();

private:
    // rocket-specific gates: AHRS health, EKF origin, FSM at PAD, pad tilt
    bool rocket_checks(bool display_failure);
};
