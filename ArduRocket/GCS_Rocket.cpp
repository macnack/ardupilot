#include "ArduRocket.h"

uint32_t GCS_Rocket::custom_mode() const
{
    return (uint32_t)rocket.control_mode;
}

MAV_TYPE GCS_Rocket::frame_type() const
{
    return rocket.get_frame_mav_type();
}

bool GCS_Rocket::vehicle_initialised() const
{
    return true;
}

uint8_t GCS_Rocket::sysid_this_mav() const
{
    return 1;
}

MAV_MODE GCS_MAVLINK_Rocket::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;

    if (hal.util->get_soft_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // we always report a custom mode (IDLE / FLIGHT)
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

MAV_STATE GCS_MAVLINK_Rocket::vehicle_system_status() const
{
    if (!rocket.ahrs.healthy()) {
        return MAV_STATE_CRITICAL;
    }
    if (!hal.util->get_soft_armed()) {
        return MAV_STATE_STANDBY;
    }
    return MAV_STATE_ACTIVE;
}
