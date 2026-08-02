#include "ArduRocket.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) \
    SCHED_TASK_CLASS(ArduRocket, &rocket, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(ArduRocket, &rocket, func)

/*
  Scheduler table.

  This is the whole point of ArduRocket existing as a vehicle rather than a
  mode inside ArduPlane. Deliberately ABSENT: stabilize() (which in ArduPlane
  ran stabilize_roll/pitch/yaw -- the fixed-wing attitude controllers -- on
  every tick of every rocket flight), navigate(), update_alt(),
  adjust_altitude_target(), calc_airspeed_errors(), update_speed_height(),
  and every other TECS/L1 task. See the Phase 2b design spec S1 and S4.2.
 */
const AP_Scheduler::Task ArduRocket::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &rocket.ins, update),
    // run the EKF. AP_Vehicle does NOT do this for you -- without it the
    // estimator never runs, so there is no attitude/position solution, GPS
    // never reports a usable fix, and arming is refused forever.
    FAST_TASK(read_AHRS),
    // run the rocket controller
    FAST_TASK(update_flight_mode),
    // push fin commands to the outputs
    FAST_TASK(set_servos),

    SCHED_TASK_CLASS(AP_GPS,            &rocket.gps,    update,         50, 200,   3),
    // nothing else reads baro/airspeed/compass; the EKF starves without them
    SCHED_TASK(update_sensors,                                          10, 200,   4),
    // set home once the EKF has an origin; AHRS stays unhealthy without it
    SCHED_TASK(update_home_from_EKF,                                    10,  50,   5),
    SCHED_TASK_CLASS(GCS,  (GCS*)&rocket._gcs,          update_receive, 400, 180,   6),
#if HAL_PARACHUTE_ENABLED
    // AP_Parachute::update() is the only place the release output moves, and it
    // wants ~10 Hz. Priority 7 keeps this table's priorities NON-DECREASING
    // (3,4,5,6,7,9,12,15,18,20) -- see the update_logging10 comment below.
    // Placed after GCS update_receive so a MAV_CMD_DO_PARACHUTE is actuated on
    // the same tick it arrives.
    SCHED_TASK(parachute_check,                                         10, 200,   7),
#endif
    SCHED_TASK_CLASS(GCS,  (GCS*)&rocket._gcs,          update_send,    400, 550,   9),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger,         &rocket.logger, periodic_tasks, 400, 300,  12),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &rocket.ins,    periodic,       400,  50,  15),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,      &rocket.scheduler, update_logging, 0.1, 75, 18),
    // EKF/IMU/baro/GPS logging is vehicle-scheduled: AP::ahrs().Log_Write()
    // is what emits the XKF* records, and nothing calls it for you.
    // MUST stay last: AP_Scheduler.cpp:153-157 requires the vehicle task
    // priorities be non-decreasing and raises INTERNAL_ERROR(flow_of_control)
    // otherwise -- which latches an internal-error flag that then fails every
    // pre-arm check for the rest of the flight.
    SCHED_TASK(update_logging10,                                        10, 300,  20),
#endif
};

void ArduRocket::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                     uint8_t &task_count,
                                     uint32_t &log_bit)
{
    // AP_Vehicle requires ALL passed-in fields be filled (Valgrind otherwise)
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t ArduRocket::_failsafe_priorities[1];

void ArduRocket::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    // report only -- see the rationale in ArduRocket.h
    gcs().send_text(MAV_SEVERITY_CRITICAL, "RKT_ERR: battery failsafe (%s)", type_str);
}

#if HAL_PARACHUTE_ENABLED
void ArduRocket::parachute_check()
{
    // ArduPlane also calls parachute.check_sink_rate() here. ArduRocket does
    // not: nothing on this vehicle calls parachute.set_is_flying(), so
    // check_sink_rate() would return early anyway, and CHUTE_CRT_SINK is 0 by
    // policy -- a rocket in ballistic descent sinks fast by design, so a bare
    // sink-rate trigger would fire the pyro at whatever speed it reached.
    parachute.update();
}
#endif

void ArduRocket::update_sensors()
{
    // Measured, not guessed: without barometer.update() the EKF gets no
    // height observation, XKF4 shows a height-fusion timeout, and the filter
    // dead-reckons the IMU into divergence (VN/VD in the hundreds of m/s,
    // PD ~10 km on the pad) -- which is why NavEKF3_core::healthy() is false.
    // ArduPlane calls this from update_alt() at 10 Hz (ArduPlane.cpp:597).
    barometer.update();
#if AP_AIRSPEED_ENABLED
    // q for the gain schedule comes from EAS; the sensor also needs reading
    airspeed.update();
#endif

    // battery before compass: it may feed compass motor-interference comp
    battery.read();
#if AP_COMPASS_ENABLED
    if (AP::compass().available()) {
        compass.set_voltage(battery.voltage());
        compass.read();
    }
#endif
}

#if HAL_LOGGING_ENABLED
void ArduRocket::update_logging10()
{
    // XKF1..XKF5 / XKQ / XKV* -- the estimator's own state, innovations and
    // variances. Without this the dataflash has no EKF records at all.
    AP::ahrs().Log_Write();
    ahrs.Log_Write_Home_And_Origin();
    ins.Write_IMU();
}
#endif  // HAL_LOGGING_ENABLED

void ArduRocket::read_AHRS()
{
    // Mirror arming state into the HAL every tick, as ArduPlane does from its
    // own ahrs_update() (ArduPlane.cpp:165). AP_Arming does NOT do this for
    // you: without it hal.util->get_soft_armed() is stuck false, so the
    // heartbeat never reports MAV_MODE_FLAG_SAFETY_ARMED (no GCS ever shows
    // this vehicle as armed), vehicle_system_status() reports standby in
    // flight, and the "never reboot an armed rocket" guard in
    // GCS_MAVLINK_Rocket::handle_preflight_reboot silently never fires.
    arming.update_soft_armed();

    // skip the INS update: the FAST_TASK above already ran it this tick
    ahrs.update(true);
}

void ArduRocket::update_flight_mode()
{
    flightmode->update();
}

void ArduRocket::set_fins(float cy, float cz)
{
    // identical mapping to ModeRocket and the Phase 0 Lua sequencer:
    // s1=+cz s2=-cz s3=+cy s4=-cy
    const auto pwm = [](float v) {
        return (uint16_t)(1500.0f + 500.0f * constrain_float(v, -1.0f, 1.0f));
    };
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, pwm(cz));
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting2, pwm(-cz));
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting3, pwm(cy));
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting4, pwm(-cy));
}

ArduRocket::ArduRocket()
    : param_loader(var_info),
      control_mode(Mode::Number::IDLE),
      flightmode(&mode_idle)
{
}

ArduRocket rocket;
AP_Vehicle& vehicle = rocket;

AP_HAL_MAIN_CALLBACKS(&rocket);
