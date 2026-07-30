#include <AP_gtest.h>
#include <AP_RocketControl/RocketStateMachine.h>

using namespace RocketControl;

static FsmInputs pad_in() { return FsmInputs{9.81f, 0.5f, 0.0f, true, 0.01f}; }

TEST(RocketStateMachine, StartsInPad)
{
    RocketStateMachine fsm;
    fsm.reset();
    EXPECT_EQ(fsm.phase(), Phase::PAD);
    EXPECT_FALSE(fsm.want_pyro());
}

TEST(RocketStateMachine, LaunchNeedsThreeConsecutiveSamplesAndArmed)
{
    RocketStateMachine fsm;
    FsmParams p;
    fsm.reset();
    FsmInputs in = pad_in();
    in.nose_accel = 35.0f;
    in.armed = false;
    for (int i = 0; i < 10; i++) fsm.step(in, p);
    EXPECT_EQ(fsm.phase(), Phase::PAD);      // never while disarmed
    in.armed = true;
    fsm.step(in, p);
    fsm.step(in, p);
    EXPECT_EQ(fsm.phase(), Phase::PAD);      // 2 samples not enough
    fsm.step(in, p);
    EXPECT_EQ(fsm.phase(), Phase::BOOST);    // 3rd sample transitions
    EXPECT_TRUE(fsm.phase_changed());
    fsm.step(in, p);
    EXPECT_FALSE(fsm.phase_changed());
}

TEST(RocketStateMachine, FullNominalSequenceAndPyroDelay)
{
    RocketStateMachine fsm;
    FsmParams p;
    fsm.reset();
    FsmInputs in = pad_in();
    in.nose_accel = 35.0f;
    for (int i = 0; i < 3; i++) fsm.step(in, p);     // -> BOOST
    in.nose_accel = 1.0f;
    for (int i = 0; i < 3; i++) fsm.step(in, p);     // -> COAST
    EXPECT_EQ(fsm.phase(), Phase::COAST);
    in.vd = 1.0f;
    for (int i = 0; i < 3; i++) fsm.step(in, p);     // -> APOGEE
    EXPECT_EQ(fsm.phase(), Phase::APOGEE);
    EXPECT_FALSE(fsm.want_pyro());                    // delay not elapsed
    in.dt = 0.6f;
    fsm.step(in, p);
    EXPECT_FALSE(fsm.want_pyro());                    // 0.6 s < 1.0 s
    fsm.step(in, p);
    EXPECT_TRUE(fsm.want_pyro());                     // 1.2 s > 1.0 s
}

TEST(RocketStateMachine, AbortOnTiltFiresPyroImmediately)
{
    RocketStateMachine fsm;
    FsmParams p;
    fsm.reset();
    FsmInputs in = pad_in();
    in.nose_accel = 35.0f;
    for (int i = 0; i < 3; i++) fsm.step(in, p);     // -> BOOST
    in.tilt_deg = 75.0f;
    fsm.step(in, p);
    EXPECT_EQ(fsm.phase(), Phase::RECOVERY);
    EXPECT_TRUE(fsm.aborted());
    EXPECT_TRUE(fsm.want_pyro());
}

TEST(RocketStateMachine, PhasesAreMonotonic)
{
    RocketStateMachine fsm;
    FsmParams p;
    fsm.reset();
    FsmInputs in = pad_in();
    in.nose_accel = 35.0f;
    for (int i = 0; i < 3; i++) fsm.step(in, p);
    in.nose_accel = 40.0f;                            // accel returns (no un-burnout path)
    Phase prev = fsm.phase();
    for (int i = 0; i < 50; i++) {
        fsm.step(in, p);
        EXPECT_GE((uint8_t)fsm.phase(), (uint8_t)prev);
        prev = fsm.phase();
    }
}

TEST(RocketStateMachine, ResetToRecoveryOnlyFromApogee)
{
    RocketStateMachine fsm;
    FsmParams p;
    fsm.reset();
    FsmInputs in = pad_in();
    in.nose_accel = 35.0f;
    for (int i = 0; i < 3; i++) fsm.step(in, p);     // BOOST
    in.nose_accel = 1.0f;
    for (int i = 0; i < 3; i++) fsm.step(in, p);     // COAST
    in.vd = 1.0f;
    for (int i = 0; i < 3; i++) fsm.step(in, p);     // APOGEE
    fsm.reset_to_recovery();
    EXPECT_EQ(fsm.phase(), Phase::RECOVERY);
    EXPECT_TRUE(fsm.phase_changed());
}

AP_GTEST_MAIN()
