#include "RocketStateMachine.h"

namespace RocketControl {

void RocketStateMachine::reset()
{
    _phase = Phase::PAD;
    _phase_changed = false;
    _aborted = false;
    _want_pyro = false;
    _launch_count = _burnout_count = _apogee_count = 0;
    _apogee_elapsed_s = 0.0f;
}

void RocketStateMachine::transition(Phase next)
{
    _phase = next;
    _phase_changed = true;
}

void RocketStateMachine::reset_to_recovery()
{
    if (_phase == Phase::APOGEE) {
        transition(Phase::RECOVERY);
    }
}

const char *RocketStateMachine::phase_name(Phase ph)
{
    switch (ph) {
    case Phase::PAD:      return "PAD";
    case Phase::BOOST:    return "BOOST";
    case Phase::COAST:    return "COAST";
    case Phase::APOGEE:   return "APOGEE";
    case Phase::RECOVERY: return "RECOVERY";
    }
    return "?";
}

void RocketStateMachine::step(const FsmInputs &in, const FsmParams &p)
{
    _phase_changed = false;

    switch (_phase) {
    case Phase::PAD:
        if (in.armed && in.nose_accel > p.launch_accel) {
            if (++_launch_count >= 3) {
                transition(Phase::BOOST);
            }
        } else {
            _launch_count = 0;
        }
        break;

    case Phase::BOOST:
    case Phase::COAST:
        if (in.tilt_deg > p.abort_tilt_deg) {
            _aborted = true;
            _want_pyro = true;   // abort path: fire immediately
            transition(Phase::RECOVERY);
            break;
        }
        if (_phase == Phase::BOOST) {
            if (in.nose_accel < p.burnout_accel) {
                if (++_burnout_count >= 3) {
                    transition(Phase::COAST);
                }
            } else {
                _burnout_count = 0;
            }
        } else {  // COAST
            if (in.vd > 0.5f) {
                if (++_apogee_count >= 3) {
                    transition(Phase::APOGEE);
                }
            } else {
                _apogee_count = 0;
            }
        }
        break;

    case Phase::APOGEE:
        _apogee_elapsed_s += in.dt;
        if (_apogee_elapsed_s > p.chute_delay_s) {
            _want_pyro = true;
        }
        break;

    case Phase::RECOVERY:
        break;
    }
}

} // namespace RocketControl
