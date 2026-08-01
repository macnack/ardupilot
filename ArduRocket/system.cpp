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
    // ORDERING IS SAFETY-CRITICAL HERE, not cosmetic.
    //
    // Everything that can arm must be initialised BEFORE the GCS starts
    // serving MAVLink. Long init steps (compass.init(), barometer.calibrate(),
    // ins.init()) call hal.scheduler->delay(), and AP_Vehicle's delay callback
    // pumps MAVLink while they block -- so a GCS arm command can be handled
    // part-way through this function. If that happens before gps.init(), the
    // pre-arm GPS check dereferences a not-yet-allocated blended-GPS driver
    // and the flight computer dies on the pad.
    //
    // Sensors first, MAVLink last.
    notify.init();
    battery.init();

    // gps.init() is vehicle-owned (Blimp/system.cpp:63,
    // ArduPlane/system.cpp:75). Skip it and AP_GPS::update_primary()
    // dereferences an uninitialised backend the first time the 50 Hz GPS
    // scheduler task fires -> SIGSEGV mid-flight-loop.
    gps.init();

    // Compass init is vehicle-owned (ArduPlane/system.cpp:65-66). Without it
    // the compass never comes healthy, EKF3 has no yaw source, so it never
    // completes alignment: EKF_STATUS_REPORT stays all-zero and
    // ahrs.healthy() stays false forever -> ModeFlight::_enter() refuses.
    // This one delays, hence it comes after gps.init().
#if AP_COMPASS_ENABLED
    AP::compass().set_log_bit(MASK_LOG_IMU);
    AP::compass().init();
#endif

    barometer.init();

#if AP_RELAY_ENABLED
    relay.init();       // pyro channel
#endif

    // AHRS + INS ground start. AP_Vehicle does NOT do this: every vehicle
    // calls ins.init() itself (cf. Blimp/system.cpp:119,
    // ArduPlane/system.cpp:434). Without it the INS never produces samples,
    // so scheduler.loop() blocks forever in ins.wait_for_sample() and the
    // vehicle never runs a single loop -- no MAVLink, no logging.
    ahrs.init();
    // Match ArduPlane's AHRS setup (ArduPlane/system.cpp:429-432). fly_forward
    // in particular is not cosmetic: it tells EKF3 it may align yaw from the
    // GPS velocity vector. Phase 2b is a differential test against ModeRocket
    // on the plane host, so the estimator configuration must match.
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::FIXED_WING);
    ahrs.set_wind_estimation_enabled(true);
    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    barometer.calibrate();

    // start in IDLE: fins neutral, pyro untouched
    control_mode = Mode::Number::IDLE;
    flightmode = &mode_idle;
    flightmode->_enter();

    // MAVLink comes up LAST, once every subsystem an arm command can touch is
    // initialised. See the ordering note at the top of this function: doing
    // this first opens a window where a GCS arm is serviced against
    // half-initialised sensors.
    gcs().setup_uarts();

    gcs().send_text(MAV_SEVERITY_INFO, "RKT: ArduRocket ready (IDLE)");
}

MAV_TYPE ArduRocket::get_frame_mav_type() const
{
    return MAV_TYPE_ROCKET;
}
