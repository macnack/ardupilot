#include "ArduRocket.h"

/*
  Home-position handling.

  AP_Vehicle declares set_home()/set_home_to_current_location() as virtuals
  that default to returning false, and schedules nothing to call them: setting
  home is vehicle-owned (cf. Blimp/commands.cpp, ArduPlane/commands.cpp).

  Skipping this is not a cosmetic omission. EKF3 initialises and sets its
  origin, but AP_AHRS never becomes healthy without a home, so arming reports
  "PreArm: AHRS: waiting for home" forever and ModeFlight::_enter() refuses.
 */

void ArduRocket::update_home_from_EKF()
{
    if (ahrs.home_is_set()) {
        return;
    }
    // A rocket has no in-flight home-reset case worth special handling: home
    // is the pad, established before launch. If it is somehow still unset once
    // armed, take the current EKF location -- it is better than no home.
    IGNORE_RETURN(set_home_to_current_location(false));
}

bool ArduRocket::set_home_to_current_location(bool lock)
{
    Location loc;
    if (!ahrs.get_location(loc)) {
        return false;
    }
    return set_home(loc, lock);
}

bool ArduRocket::set_home(const Location &loc, bool lock)
{
    // the EKF origin must exist before home means anything
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }
    if (!ahrs.set_home(loc)) {
        return false;
    }
    if (lock) {
        ahrs.lock_home();
    }
    return true;
}
