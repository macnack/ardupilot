#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Rocket : public GCS
{
    friend class ArduRocket;

public:

    // expands to chan(ofs) accessors returning GCS_MAVLINK_Rocket*
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Rocket);

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;
    const char *frame_string() const override { return "ArduRocket"; }
    bool vehicle_initialised() const override;
    uint8_t sysid_this_mav() const override;

protected:

    GCS_MAVLINK_Rocket *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                                AP_HAL::UARTDriver &uart) override
    {
        return NEW_NOTHROW GCS_MAVLINK_Rocket(params, uart);
    }
};
