#include "ArduRocket.h"

#include <AP_Arming/AP_Arming.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Parachute/AP_Parachute.h>

/*
  FLIGHT mode: the only mode that flies.

  Direct port of ArduPlane's ModeRocket with the plane plumbing removed. The
  control logic, pyro gating, RKT/RKTC log schema and statustext strings are
  byte-identical to that mode on purpose: AP_RocketControl is shared unchanged
  between the two hosts, so any behavioural delta between them is a
  vehicle-port defect rather than a control-law question. Keep them in sync.
 */

bool ModeFlight::_enter()
{
    if (!rocket.ahrs.healthy()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RKT: refused, AHRS unhealthy");
        return false;
    }
    if (!AP::arming().is_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RKT: refused, not armed");
        return false;
    }
    _fsm.reset();
    _ctrl.reset();
    _pyro_fired = false;
    _chute_fault_sent = false;
    _last_update_ms = AP_HAL::millis();
    announce();
    return true;
}

void ModeFlight::_exit()
{
    rocket.set_fins(0.0f, 0.0f);
}

void ModeFlight::announce()
{
    gcs().send_text(MAV_SEVERITY_INFO, "RKT: %s",
                    RocketControl::RocketStateMachine::phase_name(_fsm.phase()));
}

void ModeFlight::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float dt = MAX((now_ms - _last_update_ms) * 1e-3f, 1e-3f);
    _last_update_ms = now_ms;

    Quaternion att;
    if (!rocket.ahrs.get_quaternion(att)) {
        rocket.set_fins(0.0f, 0.0f);
        return;
    }
    const Vector3f gyro = rocket.ahrs.get_gyro();
    const Vector3f accel = rocket.ahrs.get_accel();
    Vector3f vel;
    const bool have_vel = rocket.ahrs.get_velocity_NED(vel);

    Matrix3f R;
    att.rotation_matrix(R);
    const Vector3f up_body = R.mul_transpose(Vector3f{0.0f, 0.0f, -1.0f});
    const float tilt_deg = degrees(acosf(constrain_float(up_body.x, -1.0f, 1.0f)));

    // --- flight-phase state machine ---
    RocketControl::FsmParams fp;
    fp.launch_accel = rocket.g.lnch_acc;
    fp.burnout_accel = rocket.g.burn_acc;
    fp.abort_tilt_deg = rocket.g.abrt_tilt;
    fp.chute_delay_s = rocket.g.chute_dly;
    RocketControl::FsmInputs fin;
    fin.nose_accel = accel.x;
    fin.tilt_deg = tilt_deg;
    fin.vd = have_vel ? vel.z : 0.0f;
    fin.armed = AP::arming().is_armed();
    fin.dt = dt;
    _fsm.step(fin, fp);
    if (_fsm.phase_changed()) {
        if (_fsm.aborted()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RKT: ABORT tilt %.0f", (double)tilt_deg);
        }
        announce();
    }

    // --- recovery release: AP_Parachute is the mechanism, the gate stays here.
    //     CHUTE_TYPE picks the actuator (10 = servo on k_parachute_release for
    //     the JSON SITL backend, 0-3 = relay for hardware with
    //     RELAY1_FUNCTION 3), so the pyro gate never moves onto the output. ---
    if (_fsm.want_pyro() && !_pyro_fired) {
        if (AP::arming().is_armed()) {
            AP_Parachute *chute = AP::parachute();
            // release() returns silently when CHUTE_ENABLED <= 0, so without
            // this guard a misconfigured vehicle would announce "pyro fired"
            // while nothing happened -- the worst available failure mode.
            if (chute != nullptr && chute->enabled()) {
                chute->release();
                _pyro_fired = true;
                gcs().send_text(MAV_SEVERITY_INFO, "RKT: pyro fired (%s)",
                                _fsm.aborted() ? "abort" : "apogee");
                if (_fsm.phase() == RocketControl::Phase::APOGEE) {
                    _fsm.reset_to_recovery();
                    announce();
                }
            } else if (!_chute_fault_sent) {
                // one-shot: RocketStateMachine::_want_pyro is a latch that
                // never clears, so an unlatched branch would spam at loop rate
                _chute_fault_sent = true;
                gcs().send_text(MAV_SEVERITY_CRITICAL, "RKT_ERR: chute disabled");
            }
        } else {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RKT_ERR: pyro blocked in phase %u",
                            (unsigned)_fsm.phase());
        }
    }

    // --- controller: active in BOOST/COAST, fins neutral otherwise ---
    using RocketControl::Phase;
    const Phase ph = _fsm.phase();
    if (ph == Phase::BOOST || ph == Phase::COAST) {
        RocketControl::CtrlParams cp;
        cp.att_p = rocket.g.att_p;
        cp.rate_p = rocket.g.rate_p;
        cp.rate_i = rocket.g.rate_i;
        cp.rate_d = rocket.g.rate_d;
        cp.q_ref = rocket.g.q_ref;
        cp.q_floor = rocket.g.q_floor;
        cp.imax = rocket.g.imax;
        cp.d_filt_hz = rocket.g.d_filt_hz;
        cp.slew_rate = rocket.g.slew_rate;
        RocketControl::CtrlInputs ci;
        ci.att = att;
        ci.gyro = gyro;
        float eas = 0.0f;
        ci.airspeed_healthy = rocket.ahrs.airspeed_estimate(eas);
        ci.eas = eas;
        ci.integrate = AP::arming().is_armed();
        ci.dt = dt;
        const auto out = _ctrl.update(ci, cp);
        if (out.fault_nonfinite) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RKT_ERR: ctrl non-finite input");
        }
        rocket.set_fins(out.cy, out.cz);

        AP::logger().WriteStreaming("RKTC", "TimeUS,q,GS,RyC,Ry,RzC,Rz,Cy,Cz,Sat",
                                    "Qfffffffff",
                                    AP_HAL::micros64(),
                                    (double)out.q, (double)out.gain_scale,
                                    (double)out.rate_cmd.y, (double)gyro.y,
                                    (double)out.rate_cmd.z, (double)gyro.z,
                                    (double)out.cy, (double)out.cz,
                                    (double)((out.saturated_y || out.saturated_z) ? 1.0f : 0.0f));
    } else {
        rocket.set_fins(0.0f, 0.0f);
    }

    AP::logger().WriteStreaming("RKT", "TimeUS,Ph,Tilt,Ax,Vd", "QBfff",
                                AP_HAL::micros64(), (uint8_t)ph,
                                (double)tilt_deg, (double)accel.x,
                                (double)fin.vd);
}
