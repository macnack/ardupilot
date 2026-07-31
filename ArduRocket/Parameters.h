#pragma once

// must match the name of the global ArduRocket instance (ArduRocket.cpp),
// because AP_Param.h's GSCALAR/GOBJECT macros expand through it
#define AP_PARAM_VEHICLE_NAME rocket

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>

class Parameters
{
public:
    // The version of the layout as described by the parameter enum.
    static const uint16_t k_format_version = 1;

    // WARNING: care must be taken when editing this enumeration, as the
    // AP_Param load/save code depends on these values to identify variables
    // saved in EEPROM. Only append; never renumber existing entries.
    enum {
        k_param_format_version = 0,

        // vehicle sub-objects
        k_param_arming = 10,
        k_param_ahrs,
        k_param_ins,
        k_param_gps,
        k_param_barometer,
        k_param_relay,
        k_param_scheduler,
        k_param_BoardConfig,
        k_param_notify,
        k_param_sitl,
        k_param_NavEKF3,
        k_param_servo_channels,
        k_param_gcs0,
        k_param_gcs1,

        // rocket controller gains (RKTC_ prefix)
        k_param_att_p = 100,
        k_param_rate_p,
        k_param_rate_i,
        k_param_rate_d,
        k_param_q_ref,
        k_param_q_floor,
        k_param_imax,
        k_param_d_filt_hz,
        k_param_slew_rate,

        // flight-phase state machine
        k_param_lnch_acc,
        k_param_burn_acc,
        k_param_abrt_tilt,
        k_param_chute_dly,

        // AP_Vehicle common block
        k_param_vehicle = 257,
    };

    AP_Int16 format_version;

    // controller gains
    AP_Float att_p;
    AP_Float rate_p;
    AP_Float rate_i;
    AP_Float rate_d;
    AP_Float q_ref;
    AP_Float q_floor;
    AP_Float imax;
    AP_Float d_filt_hz;
    AP_Float slew_rate;

    // flight-phase state machine
    AP_Float lnch_acc;
    AP_Float burn_acc;
    AP_Float abrt_tilt;
    AP_Float chute_dly;

    // fin servo output channels (SERVOn_*)
    SRV_Channels servo_channels;

    Parameters() {}
};
