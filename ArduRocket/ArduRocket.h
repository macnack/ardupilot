#pragma once

/*
  ArduRocket -- standalone AP_Vehicle vehicle type for fin-guided rockets.

  All rocket logic lives in libraries/AP_RocketControl (RocketStateMachine +
  RocketAttitudeControl), reused unchanged from the ArduPlane ModeRocket
  Phase 1/2a work. This vehicle is a thin host for it.

  The point of the vehicle (vs a mode inside ArduPlane) is the scheduler
  table: see ArduRocket.cpp. ArduPlane ran stabilize()/navigate()/TECS on
  every tick of every rocket flight because ModeRocket could not opt out.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <SRV_Channel/SRV_Channel.h>

#include "config.h"
#include "defines.h"
#include "Parameters.h"
#include "mode.h"
#include "GCS_Rocket.h"
#include "AP_Arming.h"

class ArduRocket : public AP_Vehicle
{
public:
    friend class GCS_MAVLINK_Rocket;
    friend class GCS_Rocket;
    friend class AP_Arming_Rocket;
    friend class ModeIdle;
    friend class ModeFlight;
    friend class Parameters;

    ArduRocket();

    /* Do not allow copies */
    CLASS_NO_COPY(ArduRocket);

    static const struct AP_Param::Info var_info[];

    // --- AP_Vehicle pure virtuals ---
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    uint8_t get_mode() const override { return (uint8_t)control_mode; }
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;

    // typed overload used internally
    bool set_mode(Mode::Number mode, ModeReason reason);

    MAV_TYPE get_frame_mav_type() const;

#if HAL_LOGGING_ENABLED
    static const struct LogStructure log_structure[];
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    uint8_t get_num_log_structures() const override;
#endif

    // drive the four fin servos; inputs normalized to [-1, 1]
    void set_fins(float cy, float cz);

protected:
    void init_ardupilot() override;
    void load_parameters() override;

private:
    static const AP_Scheduler::Task scheduler_tasks[];

    // setup the var_info table
    AP_Param param_loader;

    Parameters g;

    GCS_Rocket _gcs;
    AP_Arming_Rocket arming;

    // AP_BattMonitor is NOT an AP_Vehicle member: each vehicle declares its
    // own. AP_Arming::pre_arm_checks() calls AP::battery().arming_checks()
    // unconditionally, so without this the first pre-arm check segfaults.
    //
    // There is no useful battery-failsafe ACTION for a rocket: once the motor
    // lights the trajectory is committed and there is nothing to land or
    // return. The handler therefore only reports; recovery stays with the
    // flight-phase FSM and its pyro.
    void handle_battery_failsafe(const char *type_str, const int8_t action);
    static constexpr int8_t _failsafe_priorities[] = {
        -1  // the priority list must end with a sentinel of -1
    };
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&ArduRocket::handle_battery_failsafe,
                                               void, const char *, const int8_t),
                           _failsafe_priorities};

    ModeIdle   mode_idle;
    ModeFlight mode_flight;

    Mode::Number control_mode;
    Mode *flightmode;

    // scheduler task bodies
    void read_AHRS();
    void update_flight_mode();
    void set_servos();

    Mode *mode_from_number(Mode::Number num);
};

extern ArduRocket rocket;
