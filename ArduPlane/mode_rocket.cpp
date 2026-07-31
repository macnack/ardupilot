#include "mode.h"
#include "Plane.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Arming/AP_Arming.h>

/*
  ModeRocket: thin adapter over AP_RocketControl for the ArduRocket Phase 1
  sim campaign. All rocket logic (flight-phase FSM, cascade attitude control)
  lives in libraries/AP_RocketControl; this mode only feeds it AHRS state and
  routes its outputs to the fin servos (Script1..4 functions, same wiring as
  the Phase 0 Lua sequencer) and the pyro relay.
 */

const AP_Param::GroupInfo ModeRocket::var_info[] = {
    // @Param: ATT_P
    // @DisplayName: Rocket attitude P gain
    // @Description: Outer-loop body-rate command per radian of tilt error
    AP_GROUPINFO("ATT_P", 1, ModeRocket, att_p, 12.0f),
    AP_GROUPINFO("RAT_P", 2, ModeRocket, rate_p, 1.50f),
    AP_GROUPINFO("RAT_I", 3, ModeRocket, rate_i, 0.50f),
    AP_GROUPINFO("RAT_D", 4, ModeRocket, rate_d, 0.08f),
    AP_GROUPINFO("Q_REF", 5, ModeRocket, q_ref, 800.0f),
    AP_GROUPINFO("Q_FLOOR", 6, ModeRocket, q_floor, 50.0f),
    AP_GROUPINFO("IMAX", 7, ModeRocket, imax, 0.30f),
    AP_GROUPINFO("LNCH_ACC", 8, ModeRocket, lnch_acc, 30.0f),
    AP_GROUPINFO("BURN_ACC", 9, ModeRocket, burn_acc, 5.0f),
    AP_GROUPINFO("ABRT_TILT", 10, ModeRocket, abrt_tilt, 60.0f),
    AP_GROUPINFO("CHUTE_DLY", 11, ModeRocket, chute_dly, 1.0f),
    AP_GROUPEND
};

ModeRocket::ModeRocket()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeRocket::_enter()
{
    if (!ahrs.healthy()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RCKT: refused, AHRS unhealthy");
        return false;
    }
    _fsm.reset();
    _ctrl.reset();
    _pyro_fired = false;
    _last_update_ms = AP_HAL::millis();
    gcs().send_text(MAV_SEVERITY_INFO, "RKT: %s",
                    RocketControl::RocketStateMachine::phase_name(_fsm.phase()));
    return true;
}

void ModeRocket::_exit()
{
    set_fins(0, 0);
}

void ModeRocket::set_fins(float cy, float cz)
{
    // identical mapping to the Lua sequencer: s1=+cz s2=-cz s3=+cy s4=-cy
    const auto pwm = [](float v) {
        return (uint16_t)(1500.0f + 500.0f * constrain_float(v, -1.0f, 1.0f));
    };
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, pwm(cz));
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting2, pwm(-cz));
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting3, pwm(cy));
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting4, pwm(-cy));
}

void ModeRocket::announce()
{
    gcs().send_text(MAV_SEVERITY_INFO, "RKT: %s",
                    RocketControl::RocketStateMachine::phase_name(_fsm.phase()));
}

void ModeRocket::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float dt = MAX((now_ms - _last_update_ms) * 1e-3f, 1e-3f);
    _last_update_ms = now_ms;

    Quaternion att;
    if (!ahrs.get_quaternion(att)) {
        set_fins(0, 0);
        return;
    }
    const Vector3f gyro = ahrs.get_gyro();
    const Vector3f accel = ahrs.get_accel();
    Vector3f vel;
    const bool have_vel = ahrs.get_velocity_NED(vel);

    Matrix3f R;
    att.rotation_matrix(R);
    const Vector3f up_body = R.mul_transpose(Vector3f{0.0f, 0.0f, -1.0f});
    const float tilt_deg = degrees(acosf(constrain_float(up_body.x, -1.0f, 1.0f)));

    // --- flight-phase state machine ---
    RocketControl::FsmParams fp;
    fp.launch_accel = lnch_acc;
    fp.burnout_accel = burn_acc;
    fp.abort_tilt_deg = abrt_tilt;
    fp.chute_delay_s = chute_dly;
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

    // --- pyro: sole authority, gated exactly like the Lua ---
    if (_fsm.want_pyro() && !_pyro_fired) {
        if (AP::arming().is_armed()) {
            AP_Relay *relay = AP::relay();
            if (relay != nullptr) {
                relay->on(0);
                _pyro_fired = true;
                gcs().send_text(MAV_SEVERITY_INFO, "RKT: pyro fired (%s)",
                                _fsm.aborted() ? "abort" : "apogee");
                if (_fsm.phase() == RocketControl::Phase::APOGEE) {
                    _fsm.reset_to_recovery();
                    announce();
                }
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
        cp.att_p = att_p;
        cp.rate_p = rate_p;
        cp.rate_i = rate_i;
        cp.rate_d = rate_d;
        cp.q_ref = q_ref;
        cp.q_floor = q_floor;
        cp.imax = imax;
        RocketControl::CtrlInputs ci;
        ci.att = att;
        ci.gyro = gyro;
        float eas = 0.0f;
        ci.airspeed_healthy = ahrs.airspeed_estimate(eas);
        ci.eas = eas;
        ci.integrate = AP::arming().is_armed();
        ci.dt = dt;
        const auto out = _ctrl.update(ci, cp);
        if (out.fault_nonfinite) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RKT_ERR: ctrl non-finite input");
        }
        set_fins(out.cy, out.cz);

        AP::logger().WriteStreaming("RKTC", "TimeUS,q,GS,RyC,Ry,RzC,Rz,Cy,Cz,Sat",
                                    "Qfffffffff",
                                    AP_HAL::micros64(),
                                    (double)out.q, (double)out.gain_scale,
                                    (double)out.rate_cmd.y, (double)gyro.y,
                                    (double)out.rate_cmd.z, (double)gyro.z,
                                    (double)out.cy, (double)out.cz,
                                    (double)((out.saturated_y || out.saturated_z) ? 1.0f : 0.0f));
    } else {
        set_fins(0, 0);
    }

    AP::logger().WriteStreaming("RKT", "TimeUS,Ph,Tilt,Ax,Vd", "QBfff",
                                AP_HAL::micros64(), (uint8_t)ph,
                                (double)tilt_deg, (double)accel.x,
                                (double)fin.vd);
}
