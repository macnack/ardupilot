#include "RocketAttitudeControl.h"

namespace RocketControl {

static constexpr float RHO0 = 1.225f;  // kg/m^3, sea level (EAS convention)

void RocketAttitudeControl::reset()
{
    _iy = _iz = 0.0f;
    _prev_ey = _prev_ez = 0.0f;
    _first_sample = true;
    _held = CtrlOutputs{};
    _have_held = false;
}

CtrlOutputs RocketAttitudeControl::update(const CtrlInputs &in, const CtrlParams &p)
{
    // --- input validation: hold previous output on any non-finite input ---
    const bool finite = !in.att.is_nan() && !in.gyro.is_nan() &&
                        isfinite(in.eas) && isfinite(in.dt) && is_positive(in.dt);
    if (!finite) {
        CtrlOutputs out = _have_held ? _held : CtrlOutputs{};
        out.fault_nonfinite = true;
        return out;
    }

    CtrlOutputs out{};

    // --- dynamic pressure + gain schedule ---
    float q = p.q_floor;
    if (in.airspeed_healthy && is_positive(in.eas)) {
        q = MAX(0.5f * RHO0 * in.eas * in.eas, p.q_floor);
    }
    out.q = q;
    out.gain_scale = p.q_ref / q;

    // --- outer loop: earth-up in body frame, corrective rates about y/z ---
    // up_body = R^T * (0,0,-1) with R = attitude rotation matrix (body->NED).
    // Signs are EMPIRICAL, validated closed-loop against the MuJoCo plant with
    // the full pipeline (quat -> AP frame -> fins -> sim torques): y-axis and
    // z-axis disturbances both converge to <1.5 deg with these signs and
    // tumble to 180 deg with the z sign flipped. Do not re-derive analytically
    // (two prior derivations got the z sign wrong); re-run the closed-loop
    // sweep in dummy_rocket_sim if this ever needs to change.
    Matrix3f R;
    in.att.rotation_matrix(R);
    const Vector3f up_body = R.mul_transpose(Vector3f{0.0f, 0.0f, -1.0f});
    out.rate_cmd.x = 0.0f;
    out.rate_cmd.y = constrain_float(-p.att_p * up_body.z, -p.rate_limit, p.rate_limit);
    out.rate_cmd.z = constrain_float(p.att_p * up_body.y, -p.rate_limit, p.rate_limit);

    // --- inner loop: q-scheduled rate PID on y and z ---
    const float kp = p.rate_p * out.gain_scale;
    const float ki = p.rate_i * out.gain_scale;
    const float kd = p.rate_d * out.gain_scale;

    const float ey = out.rate_cmd.y - in.gyro.y;
    const float ez = out.rate_cmd.z - in.gyro.z;

    const bool int_enabled = in.integrate && (q > p.q_floor);
    if (int_enabled) {
        _iy = constrain_float(_iy + ki * ey * in.dt, -p.imax, p.imax);
        _iz = constrain_float(_iz + ki * ez * in.dt, -p.imax, p.imax);
    }

    // no D on the first sample after reset: _prev_* is stale, the raw
    // difference would be a derivative kick that slams the fins at engage
    const float dy = _first_sample ? 0.0f : (ey - _prev_ey) / in.dt;
    const float dz = _first_sample ? 0.0f : (ez - _prev_ez) / in.dt;
    _first_sample = false;
    _prev_ey = ey;
    _prev_ez = ez;

    const float uy = kp * ey + _iy + kd * dy;   // torque demand about +y_AP
    const float uz = kp * ez + _iz + kd * dz;   // torque demand about +z_AP

    // --- output mapping: cy drives torque about +y directly; the sim-roll
    // fin pair drives torque about x_r = -z_AP, so cz = -uz (the single
    // axis flip in the whole chain, see Phase 0 conventions) ---
    out.cy = constrain_float(uy, -1.0f, 1.0f);
    out.cz = constrain_float(-uz, -1.0f, 1.0f);
    out.saturated_y = fabsf(uy) >= 1.0f;
    out.saturated_z = fabsf(uz) >= 1.0f;

    _held = out;
    _have_held = true;
    return out;
}

} // namespace RocketControl
