#pragma once

#include <AP_Math/AP_Math.h>

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
    CtrlOutputs _held{};                     // last good output (non-finite hold)
    bool _have_held = false;
};

} // namespace RocketControl
