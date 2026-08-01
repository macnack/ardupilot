#include "ArduRocket.h"

// Pad tilt beyond this fraction of RKTC_ABRT_TILT is refused at arm time:
// arming a rocket that already sits near its in-flight abort threshold is
// never intentional.
#define ROCKET_PREARM_TILT_FRACTION 0.5f

bool AP_Arming_Rocket::rocket_checks(bool display_failure)
{
    // AHRS must be healthy - the whole control loop keys off attitude
    if (!rocket.ahrs.healthy()) {
        check_failed(display_failure, "RKT: AHRS not healthy");
        return false;
    }

    // EKF origin must be set, else velocity/position are meaningless
    Location origin;
    if (!rocket.ahrs.get_origin(origin)) {
        check_failed(display_failure, "RKT: EKF origin not set");
        return false;
    }

    // the flight-phase machine must be at PAD - never arm mid-sequence
    if (rocket.mode_flight.phase() != RocketControl::Phase::PAD) {
        check_failed(display_failure, "RKT: FSM not in PAD");
        return false;
    }

    // nose must actually be up
    Quaternion att;
    if (!rocket.ahrs.get_quaternion(att)) {
        check_failed(display_failure, "RKT: no attitude solution");
        return false;
    }
    Matrix3f R;
    att.rotation_matrix(R);
    const Vector3f up_body = R.mul_transpose(Vector3f{0.0f, 0.0f, -1.0f});
    const float tilt_deg = degrees(acosf(constrain_float(up_body.x, -1.0f, 1.0f)));
    const float tilt_limit = rocket.g.abrt_tilt * ROCKET_PREARM_TILT_FRACTION;
    if (tilt_deg > tilt_limit) {
        check_failed(display_failure, "RKT: pad tilt %.0f deg > %.0f",
                     (double)tilt_deg, (double)tilt_limit);
        return false;
    }

    return true;
}

bool AP_Arming_Rocket::pre_arm_checks(bool display_failure)
{
    // Bitwise &, not &&: both halves must run so the operator sees EVERY
    // reason arming was refused, not just the first. Copter does the same
    // (AP_Arming_Copter::pre_arm_checks). Short-circuiting would also hide
    // the rocket checks entirely whenever a generic check fails first, which
    // is exactly when knowing the rocket's own state matters most.
    return AP_Arming::pre_arm_checks(display_failure) & rocket_checks(display_failure);
}
