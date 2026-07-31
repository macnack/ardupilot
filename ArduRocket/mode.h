#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_RocketControl/RocketStateMachine.h>
#include <AP_RocketControl/RocketAttitudeControl.h>

class ArduRocket;

/*
  Minimal mode framework.

  Vehicle modes express human/GCS intent chosen on the ground; the
  autonomously-detected flight phases (PAD/BOOST/COAST/APOGEE/RECOVERY) live
  entirely inside RocketControl::RocketStateMachine and are deliberately NOT
  modes -- ArduPilot modes are pilot-selectable by design, and flight phases
  must never be externally selectable. See the Phase 2b design spec S5.1.
 */
class Mode
{
public:
    Mode() {}
    virtual ~Mode() {}

    /* Do not allow copies */
    CLASS_NO_COPY(Mode);

    enum class Number : uint8_t {
        IDLE   = 0,
        FLIGHT = 1,
    };

    // called every fast-loop tick while this mode is active
    virtual void update() = 0;

    virtual Number mode_number() const = 0;
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    // entry/exit hooks; _enter() returning false refuses the mode change
    virtual bool _enter() { return true; }
    virtual void _exit() {}
};

class ModeIdle : public Mode
{
public:
    ModeIdle() {}
    CLASS_NO_COPY(ModeIdle);

    Number mode_number() const override { return Number::IDLE; }
    const char *name() const override { return "IDLE"; }
    const char *name4() const override { return "IDLE"; }

    void update() override;
    bool _enter() override;
};

class ModeFlight : public Mode
{
public:
    ModeFlight() {}
    CLASS_NO_COPY(ModeFlight);

    Number mode_number() const override { return Number::FLIGHT; }
    const char *name() const override { return "FLIGHT"; }
    const char *name4() const override { return "FLGT"; }

    void update() override;
    bool _enter() override;
    void _exit() override;

    // exposed so ArduRocket::set_mode() can refuse to leave FLIGHT after
    // launch, and so the pre-arm checks can require phase == PAD
    RocketControl::Phase phase() const { return _fsm.phase(); }

private:
    RocketControl::RocketStateMachine _fsm;
    RocketControl::RocketAttitudeControl _ctrl;
    bool _pyro_fired = false;
    uint32_t _last_update_ms = 0;
    void announce();
};
