#include "ArduRocket.h"

/*
  Vehicle-specific init. AP_Vehicle::setup() has already run the shared init
  (board config, scheduler, INS/AHRS, logger, serial manager) before this is
  called, so only rocket-specific setup belongs here.

  Deliberately absent vs Blimp/Plane: motor allocation, position controllers,
  RC input/output mapping. The rocket has no pilot stick input; fins are
  driven only by AP_RocketControl.
 */
void ArduRocket::init_ardupilot()
{
    // telemetry slots on the serial ports
    gcs().setup_uarts();

    barometer.init();

#if AP_RELAY_ENABLED
    relay.init();       // pyro channel
#endif

    // start in IDLE: fins neutral, pyro untouched
    control_mode = Mode::Number::IDLE;
    flightmode = &mode_idle;
    flightmode->_enter();

    gcs().send_text(MAV_SEVERITY_INFO, "RKT: ArduRocket ready (IDLE)");
}

MAV_TYPE ArduRocket::get_frame_mav_type() const
{
    return MAV_TYPE_ROCKET;
}
