#include "ArduRocket.h"

Mode *ArduRocket::mode_from_number(Mode::Number num)
{
    switch (num) {
    case Mode::Number::IDLE:
        return &mode_idle;
    case Mode::Number::FLIGHT:
        return &mode_flight;
    }
    return nullptr;
}

bool ArduRocket::set_mode(Mode::Number num, ModeReason reason)
{
    if (num == control_mode) {
        control_mode_reason = reason;
        return true;
    }

    // Once the rocket has left the pad the flight is committed: dropping to
    // IDLE would neutralize the fins mid-ascent. You cannot un-launch a
    // rocket, so refuse -- loudly, never silently.
    if (control_mode == Mode::Number::FLIGHT &&
        mode_flight.phase() != RocketControl::Phase::PAD) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,
                        "RKT_ERR: mode change refused, in flight (phase %u)",
                        (unsigned)mode_flight.phase());
        return false;
    }

    Mode *new_mode = mode_from_number(num);
    if (new_mode == nullptr) {
        notify_no_such_mode((uint8_t)num);
        return false;
    }

    if (!new_mode->_enter()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "RKT_ERR: mode %s refused",
                        new_mode->name());
        return false;
    }

    flightmode->_exit();
    flightmode = new_mode;
    control_mode = num;
    control_mode_reason = reason;   // AP_Vehicle requires this be set
    gcs().send_text(MAV_SEVERITY_INFO, "RKT: mode %s", new_mode->name());
    return true;
}

bool ArduRocket::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    if (new_mode > (uint8_t)Mode::Number::FLIGHT) {
        notify_no_such_mode(new_mode);
        return false;
    }
    return set_mode((Mode::Number)new_mode, reason);
}
