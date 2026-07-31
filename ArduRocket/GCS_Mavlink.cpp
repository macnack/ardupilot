#include "ArduRocket.h"

MAV_RESULT GCS_MAVLINK_Rocket::handle_preflight_reboot(const mavlink_command_int_t &packet,
                                                       const mavlink_message_t &msg)
{
    // a live rocket must never reboot its flight computer
    if (hal.util->get_soft_armed()) {
        return MAV_RESULT_FAILED;
    }
    return GCS_MAVLINK::handle_preflight_reboot(packet, msg);
}
