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

    ModeIdle   mode_idle;
    ModeFlight mode_flight;

    Mode::Number control_mode;
    Mode *flightmode;

    // scheduler task bodies
    void update_flight_mode();
    void set_servos();

    Mode *mode_from_number(Mode::Number num);
};

extern ArduRocket rocket;
