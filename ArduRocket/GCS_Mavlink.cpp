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

MAV_RESULT GCS_MAVLINK_Rocket::handle_command_int_packet(const mavlink_command_int_t &packet,
                                                         const mavlink_message_t &msg)
{
    switch (packet.command) {
#if HAL_PARACHUTE_ENABLED
    case MAV_CMD_DO_PARACHUTE:
        return handle_MAV_CMD_DO_PARACHUTE(packet);
#endif
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

#if HAL_PARACHUTE_ENABLED
MAV_RESULT GCS_MAVLINK_Rocket::handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet)
{
    switch ((uint16_t)packet.param1) {
    case PARACHUTE_DISABLE:
        rocket.parachute.enabled(false);
        return MAV_RESULT_ACCEPTED;
    case PARACHUTE_ENABLE:
        rocket.parachute.enabled(true);
        return MAV_RESULT_ACCEPTED;
    case PARACHUTE_RELEASE:
        // Same gate the flight-phase FSM uses: never fire a pyro on a disarmed
        // vehicle, whoever is asking. There is deliberately no altitude floor
        // here -- Phase 2d ships CHUTE_ALT_MIN 0 by decision, so ArduRocket
        // does not copy Plane::parachute_manual_release()'s check.
        if (!AP::arming().is_armed()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RKT: chute release refused, disarmed");
            return MAV_RESULT_FAILED;
        }
        if (!rocket.parachute.enabled()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RKT: chute not enabled");
            return MAV_RESULT_FAILED;
        }
        if (rocket.parachute.released()) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "RKT: chute already released");
            return MAV_RESULT_FAILED;
        }
        rocket.parachute.release();
        return MAV_RESULT_ACCEPTED;
    default:
        break;
    }
    return MAV_RESULT_FAILED;
}
#endif  // HAL_PARACHUTE_ENABLED
