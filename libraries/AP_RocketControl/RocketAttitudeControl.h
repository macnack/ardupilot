#pragma once

#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>

namespace RocketControl {

struct CtrlParams {
    // defaults from the closed-loop sim sweep (nominal 0.1 deg, tip-off 1.4 deg,
    // y-disturbance 1.4 deg at q_ref=800/q_floor=50)
    float att_p = 12.0f;      // outer loop: rad/s per rad of tilt error
    float rate_p = 1.50f;     // inner loop gains at q == q_ref
    float rate_i = 0.50f;
    float rate_d = 0.08f;
    float q_ref = 800.0f;     // Pa, tuning point (mid-boost)
    float q_floor = 50.0f;    // Pa, scheduling floor / integrator enable
    float imax = 0.30f;       // integrator output clamp (normalized units)
    float rate_limit = 6.0f;  // rad/s outer-loop command clamp
    // Phase 2a chatter fix (rocket/reports/phase1_results.md Known Limitations):
    // the raw finite-difference D term amplified gyro noise into a rail-to-rail
    // fin limit cycle at ~10 Hz. d_filt_hz low-pass filters the D term below
    // that; slew_rate optionally caps how fast the fin command itself can
    // move. Either <= 0 disables that stage.
    //
    // d_filt_hz=2 was picked from a closed-loop sweep against the MuJoCo
    // plant (tipoff/wind_force/thrust_high scenarios): it cuts fin-command
    // total variation 7-45x with tilt envelopes holding or improving. Do NOT
    // also enable slew_rate at a tight value on top of it — the sweep found
    // the two stacked (e.g. 8 Hz + 20 units/s) nearly triples tilt error on
    // thrust_high (excess combined loop lag), worse than either alone. Kept
    // as an available knob for hardware tuning, default off.
    float d_filt_hz = 2.0f;  // Hz, D-term low-pass cutoff
    float slew_rate = 0.0f;  // normalized units/s, output rate limit (off by default)
};

struct CtrlInputs {
    Quaternion att;           // AP body->NED attitude (AHRS convention)
    Vector3f gyro;            // body rates rad/s
    float eas;                // equivalent airspeed m/s
    bool airspeed_healthy;
    bool integrate;           // armed && phase in {BOOST, COAST}
    float dt;                 // s
};

struct CtrlOutputs {
    float cy;                 // normalized fin cmd, pitch pair
    float cz;                 // normalized fin cmd, sim-roll pair
    float q;                  // dynamic pressure used (Pa)
    float gain_scale;         // q_ref / max(q, q_floor)
    Vector3f rate_cmd;        // outer-loop body-rate command (x always 0)
    bool saturated_y, saturated_z;
    bool fault_nonfinite;     // input rejected, previous output held
};

class RocketAttitudeControl {
public:
    void reset();
    CtrlOutputs update(const CtrlInputs &in, const CtrlParams &p);

private:
    float _iy = 0.0f, _iz = 0.0f;            // integrators (normalized units)
    float _prev_ey = 0.0f, _prev_ez = 0.0f;  // for rate D term
    bool _first_sample = true;               // suppress D-term kick after reset
    LowPassFilterFloat _d_filt_y;            // D-term chatter filter (Phase 2a)
    LowPassFilterFloat _d_filt_z;
    float _prev_cy = 0.0f, _prev_cz = 0.0f;  // for output slew limit (Phase 2a)
    CtrlOutputs _held{};                     // last good output (non-finite hold)
    bool _have_held = false;
};

} // namespace RocketControl
