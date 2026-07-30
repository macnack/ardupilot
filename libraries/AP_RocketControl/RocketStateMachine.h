#pragma once

#include <stdint.h>

namespace RocketControl {

enum class Phase : uint8_t { PAD = 0, BOOST = 1, COAST = 2, APOGEE = 3, RECOVERY = 4 };

struct FsmParams {
    float launch_accel = 30.0f;    // m/s^2 nose accel to declare launch
    float burnout_accel = 5.0f;    // m/s^2 below which boost ended
    float abort_tilt_deg = 60.0f;  // tilt to abort
    float chute_delay_s = 1.0f;    // APOGEE dwell before pyro
};

struct FsmInputs {
    float nose_accel;   // m/s^2, body-x specific force
    float tilt_deg;     // angle nose vs earth-up
    float vd;           // NED down velocity, m/s
    bool armed;
    float dt;           // seconds since last step
};

class RocketStateMachine {
public:
    void reset();
    void step(const FsmInputs &in, const FsmParams &p);
    Phase phase() const { return _phase; }
    bool phase_changed() const { return _phase_changed; }
    bool aborted() const { return _aborted; }
    bool want_pyro() const { return _want_pyro; }
    // caller-driven APOGEE -> RECOVERY after a successful apogee pyro fire
    // (mirrors the Lua sequencer where try_fire_pyro drives the transition)
    void reset_to_recovery();
    static const char *phase_name(Phase ph);

private:
    Phase _phase = Phase::PAD;
    bool _phase_changed = false;
    bool _aborted = false;
    bool _want_pyro = false;
    uint8_t _launch_count = 0, _burnout_count = 0, _apogee_count = 0;
    float _apogee_elapsed_s = 0.0f;
    void transition(Phase next);
};

} // namespace RocketControl
