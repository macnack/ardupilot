#include "ArduRocket.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <SITL/SITL.h>

// GSCALAR / GOBJECT / GOBJECTN / PARAM_VEHICLE_INFO / AP_VAREND come from
// AP_Param.h and expand through AP_PARAM_VEHICLE_NAME (see Parameters.h).

/*
  Rocket controller + flight-phase parameters. Values are carried over
  verbatim from ModeRocket::var_info[] (ArduPlane/mode_rocket.cpp), including
  the Phase 2a chatter-fix defaults RKTC_D_FILT=2.0 / RKTC_SLEW=0.0.
 */
const AP_Param::Info ArduRocket::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION", 0),

    // @Param: RKTC_ATT_P
    // @DisplayName: Rocket attitude P gain
    // @Description: Outer-loop body-rate command per radian of tilt error
    // @User: Standard
    GSCALAR(att_p,     "RKTC_ATT_P",    12.0f),

    // @Param: RKTC_RAT_P
    // @DisplayName: Rocket rate P gain at q_ref
    // @User: Standard
    GSCALAR(rate_p,    "RKTC_RAT_P",     1.50f),

    // @Param: RKTC_RAT_I
    // @DisplayName: Rocket rate I gain at q_ref
    // @User: Standard
    GSCALAR(rate_i,    "RKTC_RAT_I",     0.50f),

    // @Param: RKTC_RAT_D
    // @DisplayName: Rocket rate D gain at q_ref
    // @User: Standard
    GSCALAR(rate_d,    "RKTC_RAT_D",     0.08f),

    // @Param: RKTC_Q_REF
    // @DisplayName: Rocket gain-schedule reference dynamic pressure
    // @Units: Pa
    // @User: Standard
    GSCALAR(q_ref,     "RKTC_Q_REF",   800.0f),

    // @Param: RKTC_Q_FLOOR
    // @DisplayName: Rocket gain-schedule dynamic pressure floor
    // @Units: Pa
    // @User: Standard
    GSCALAR(q_floor,   "RKTC_Q_FLOOR",  50.0f),

    // @Param: RKTC_IMAX
    // @DisplayName: Rocket rate integrator clamp
    // @User: Standard
    GSCALAR(imax,      "RKTC_IMAX",      0.30f),

    // @Param: RKTC_D_FILT
    // @DisplayName: Rocket rate D-term filter cutoff
    // @Description: Low-pass cutoff (Hz) on the rate-loop D term; damps the
    // gyro-noise-driven fin chatter measured in Phase 1. 0 disables filtering.
    // @Units: Hz
    // @User: Standard
    GSCALAR(d_filt_hz, "RKTC_D_FILT",    2.0f),

    // @Param: RKTC_SLEW
    // @DisplayName: Rocket fin output slew limit
    // @Description: Max rate of change (normalized units/s) of the fin
    // command. 0 disables the limit. Off by default: the closed-loop sweep
    // found this stacked with RKTC_D_FILT hurts tilt tracking more than
    // either alone -- only enable with a fresh sweep.
    // @User: Standard
    GSCALAR(slew_rate, "RKTC_SLEW",      0.0f),

    // @Param: RKTC_LNCH_ACC
    // @DisplayName: Nose acceleration to declare launch
    // @Units: m/s/s
    // @User: Standard
    GSCALAR(lnch_acc,  "RKTC_LNCH_ACC", 30.0f),

    // @Param: RKTC_BURN_ACC
    // @DisplayName: Nose acceleration below which boost has ended
    // @Units: m/s/s
    // @User: Standard
    GSCALAR(burn_acc,  "RKTC_BURN_ACC",  5.0f),

    // @Param: RKTC_ABRT_TILT
    // @DisplayName: Tilt angle triggering abort
    // @Units: deg
    // @User: Standard
    GSCALAR(abrt_tilt, "RKTC_ABRT_TILT", 60.0f),

    // @Param: RKTC_CHUTE_DLY
    // @DisplayName: Apogee dwell before pyro fires
    // @Units: s
    // @User: Standard
    GSCALAR(chute_dly, "RKTC_CHUTE_DLY",  1.0f),

    // --- library sub-objects. AP_Vehicle's own var_info (registered via
    // PARAM_VEHICLE_INFO below) already covers ARSPD/SCR_/LOG/SERIAL/FENCE_
    // and friends; everything here is vehicle-declared by convention. ---

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming, "ARMING_", AP_Arming),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs, "AHRS_", AP_AHRS),

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins, "INS", AP_InertialSensor),

    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

#if AP_RELAY_ENABLED
    // @Group: RELAY
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay, "RELAY", AP_Relay),
#endif

#if HAL_PARACHUTE_ENABLED
    // @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute, "CHUTE_", AP_Parachute),
#endif

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig, "BRD_", AP_BoardConfig),

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_", AP_Notify),
    GOBJECT(battery, "BATT", AP_BattMonitor),
#if AP_COMPASS_ENABLED
    GOBJECT(compass, "COMPASS_", Compass),
#endif

#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    GGROUP(servo_channels, "SERVO", SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    GGROUP(rc_channels, "RC", RC_Channels_Rocket),

    // @Group: SR0_
    // @Path: ../libraries/GCS_MAVLink/GCS_Param.cpp
    GOBJECTN(_gcs.chan_parameters[0], gcs0, "SR0_", GCS_MAVLINK_Parameters),

#if MAVLINK_COMM_NUM_BUFFERS >= 2
    // @Group: SR1_
    // @Path: ../libraries/GCS_MAVLink/GCS_Param.cpp
    GOBJECTN(_gcs.chan_parameters[1], gcs1, "SR1_", GCS_MAVLINK_Parameters),
#endif

    // AP_Vehicle common block (ARSPD_, SCR_, LOG, SERIAL, ...)
    PARAM_VEHICLE_INFO,

    AP_VAREND
};

void ArduRocket::load_parameters()
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);
}
