#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Rocket : public GCS_MAVLINK
{
public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    uint32_t telem_delay() const override { return 0; }
    uint8_t sysid_my_gcs() const override { return 255; }

    MAV_MODE base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    // a rocket has no navigation controller and no tunable PID exposed over
    // MAVLink; these are pure virtual in GCS_MAVLINK so they must exist
    void send_nav_controller_output() const override {}
    void send_pid_tuning() override {}

    // a live rocket must never reboot its flight computer
    MAV_RESULT handle_preflight_reboot(const mavlink_command_int_t &packet,
                                       const mavlink_message_t &msg) override;
};
