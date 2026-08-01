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
    // nothing else reads these: without a periodic compass.read() the compass
    // never produces samples, so it never reports healthy, so EKF3 has no yaw
    // source and never completes alignment (ahrs.healthy() stays false).
    SCHED_TASK(update_batt_compass,                                     10, 120,   4),
    // set home once the EKF has an origin; AHRS stays unhealthy without it
    SCHED_TASK(update_home_from_EKF,                                    10,  50,   5),
    SCHED_TASK_CLASS(GCS,  (GCS*)&rocket._gcs,          update_receive, 400, 180,   6),
    SCHED_TASK_CLASS(GCS,  (GCS*)&rocket._gcs,          update_send,    400, 550,   9),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger,         &rocket.logger, periodic_tasks, 400, 300,  12),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &rocket.ins,    periodic,       400,  50,  15),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,      &rocket.scheduler, update_logging, 0.1, 75, 18),
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

void ArduRocket::update_batt_compass()
{
    // battery first: it may be used for compass motor-interference compensation
    battery.read();
#if AP_COMPASS_ENABLED
    if (AP::compass().available()) {
        compass.set_voltage(battery.voltage());
        compass.read();
    }
#endif
}

void ArduRocket::read_AHRS()
{
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
