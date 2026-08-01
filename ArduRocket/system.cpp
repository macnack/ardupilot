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

    notify.init();
    battery.init();
    barometer.init();

    // gps.init() is vehicle-owned too (Blimp/system.cpp:63,
    // ArduPlane/system.cpp:75). Skip it and AP_GPS::update_primary()
    // dereferences an uninitialised backend the first time the 50 Hz GPS
    // scheduler task fires -> SIGSEGV mid-flight-loop.
    gps.init();

#if AP_RELAY_ENABLED
    relay.init();       // pyro channel
#endif

    // AHRS + INS ground start. AP_Vehicle does NOT do this: every vehicle
    // calls ins.init() itself (cf. Blimp/system.cpp:119,
    // ArduPlane/system.cpp:434). Without it the INS never produces samples,
    // so scheduler.loop() blocks forever in ins.wait_for_sample() and the
    // vehicle never runs a single loop -- no MAVLink, no logging.
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::FIXED_WING);
    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    barometer.calibrate();

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
