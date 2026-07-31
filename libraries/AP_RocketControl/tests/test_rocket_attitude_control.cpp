#include <AP_gtest.h>
#include <AP_RocketControl/RocketAttitudeControl.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

using namespace RocketControl;

// nose-up attitude: AP body x aligned with earth-up == pitch +90
static Quaternion nose_up()
{
    Quaternion q;
    q.from_euler(0.0f, radians(90.0f), 0.0f);
    return q;
}

// nose-up then rotated by angle_rad about body z (the controllable sim-roll axis)
static Quaternion nose_up_tilted_z(float angle_rad)
{
    Quaternion q = nose_up();
    Quaternion r;
    r.from_axis_angle(Vector3f{0, 0, 1}, angle_rad);
    return q * r;
}

static CtrlInputs base_in(const Quaternion &att)
{
    CtrlInputs in{};
    in.att = att;
    in.gyro.zero();
    in.eas = 40.0f;           // q = 0.5*1.225*1600 = 980 Pa
    in.airspeed_healthy = true;
    in.integrate = true;
    in.dt = 0.0025f;
    return in;
}

TEST(RocketAttitudeControl, VerticalNoRatesGivesZeroOutput)
{
    RocketAttitudeControl c;
    c.reset();
    CtrlParams p;
    auto out = c.update(base_in(nose_up()), p);
    EXPECT_NEAR(out.cy, 0.0f, 1e-4f);
    EXPECT_NEAR(out.cz, 0.0f, 1e-4f);
    EXPECT_FALSE(out.fault_nonfinite);
}

TEST(RocketAttitudeControl, TiltAboutZCommandsCorrectiveCz)
{
    RocketAttitudeControl c;
    c.reset();
    CtrlParams p;
    auto out = c.update(base_in(nose_up_tilted_z(radians(10.0f))), p);
    // Empirical convention (closed-loop validated against the MuJoCo plant):
    // attitude post-rotated +10 deg about body z gives up_body.y = -sin(10)
    // -> rate_cmd.z = +att_p*up_y < 0 -> uz < 0 -> cz = -uz > 0.
    // The opposite sign tumbles the sim to 180 deg; see RocketAttitudeControl.cpp.
    EXPECT_GT(out.cz, 0.01f);
    EXPECT_NEAR(out.cy, 0.0f, 1e-3f);
    EXPECT_NEAR(out.rate_cmd.x, 0.0f, 1e-6f);   // never command roll axis
}

TEST(RocketAttitudeControl, GainScaleFollowsQAndFloors)
{
    RocketAttitudeControl c;
    CtrlParams p;
    c.reset();
    CtrlInputs in = base_in(nose_up_tilted_z(radians(5.0f)));
    in.eas = 40.0f;                             // q = 980 Pa > q_ref
    auto fast = c.update(in, p);
    c.reset();
    in.eas = 20.0f;                             // q = 245 Pa < q_ref
    auto slow = c.update(in, p);
    EXPECT_GT(slow.gain_scale, fast.gain_scale);        // gains grow as q drops
    c.reset();
    in.eas = 1.0f;                              // q ~ 0.6 Pa -> floored
    auto floored = c.update(in, p);
    EXPECT_NEAR(floored.gain_scale, p.q_ref / p.q_floor, 1e-3f);
    EXPECT_NEAR(floored.q, p.q_floor, 1e-3f);
}

TEST(RocketAttitudeControl, UnhealthyAirspeedUsesFloor)
{
    RocketAttitudeControl c;
    c.reset();
    CtrlParams p;
    CtrlInputs in = base_in(nose_up());
    in.airspeed_healthy = false;
    in.eas = 40.0f;                             // must be ignored
    auto out = c.update(in, p);
    EXPECT_NEAR(out.q, p.q_floor, 1e-3f);
}

TEST(RocketAttitudeControl, IntegratorClampsAndOnlyRunsWhenEnabled)
{
    RocketAttitudeControl c;
    c.reset();
    CtrlParams p;
    CtrlInputs in = base_in(nose_up_tilted_z(radians(20.0f)));
    in.integrate = false;
    // let the Phase 2a D-filter/slew-limit settle to steady state first so
    // this test isolates integrator behavior, not output-shaping dynamics.
    for (int i = 0; i < 400; i++) {
        c.update(in, p);
    }
    auto a = c.update(in, p);
    auto b = c.update(in, p);
    EXPECT_NEAR(a.cz, b.cz, 1e-5f);             // no integration -> identical
    in.integrate = true;
    for (int i = 0; i < 20000; i++) {           // 50 s: must wind up then clamp
        c.update(in, p);
    }
    auto final_out = c.update(in, p);
    EXPECT_GE(final_out.cz, -1.0f);             // output clamped
    // integrator contribution alone bounded by imax (D term ~0 in steady state)
    EXPECT_LE(fabsf(final_out.cz - b.cz), p.imax + 0.15f);
}

// Phase 2a chatter fix: sample-to-sample gyro noise (Nyquist-frequency
// alternation, the worst case for a raw finite-difference D term) must
// produce far less fin-command total variation once low-pass filtered —
// this is the chatter metric from rocket/reports/phase1_results.md Known
// Limitations, applied at unit-test scale.
static float fin_total_variation(RocketAttitudeControl &c, const CtrlParams &p,
                                  const Quaternion &att, float noise_amp, int n)
{
    float tv = 0.0f;
    float prev_cz = 0.0f;
    for (int i = 0; i < n; i++) {
        CtrlInputs in = base_in(att);
        in.gyro.z = (i % 2 == 0) ? noise_amp : -noise_amp;
        auto out = c.update(in, p);
        if (i > 0) {
            tv += fabsf(out.cz - prev_cz);
        }
        prev_cz = out.cz;
    }
    return tv;
}

TEST(RocketAttitudeControl, DTermFilterAttenuatesAlternatingGyroNoise)
{
    CtrlParams p;
    p.slew_rate = 0.0f;  // isolate the D filter from the output slew limit

    RocketAttitudeControl filtered;
    filtered.reset();
    const float tv_filtered = fin_total_variation(filtered, p, nose_up(), 0.01f, 40);

    CtrlParams unfiltered = p;
    unfiltered.d_filt_hz = 0.0f;
    RocketAttitudeControl raw;
    raw.reset();
    const float tv_unfiltered = fin_total_variation(raw, unfiltered, nose_up(), 0.01f, 40);

    EXPECT_GT(tv_unfiltered, 1e-3f);              // sanity: raw D does chatter
    EXPECT_LT(tv_filtered, 0.25f * tv_unfiltered); // filtered chatter well below raw
}

// Phase 2a chatter fix: the fin command cannot move faster than slew_rate,
// even when the controller demand saturates instantly (as it does at max-q,
// see phase1_results.md).
TEST(RocketAttitudeControl, OutputSlewLimitCapsCommandStep)
{
    RocketAttitudeControl c;
    c.reset();
    CtrlParams p;
    p.slew_rate = 4.0f;  // units/s
    CtrlInputs in = base_in(nose_up_tilted_z(radians(20.0f)));  // saturates uz
    auto out = c.update(in, p);
    EXPECT_NEAR(fabsf(out.cz), p.slew_rate * in.dt, 1e-5f);
}

TEST(RocketAttitudeControl, NonFiniteInputHoldsPreviousOutputAndFlags)
{
    RocketAttitudeControl c;
    c.reset();
    CtrlParams p;
    auto good = c.update(base_in(nose_up_tilted_z(radians(10.0f))), p);
    CtrlInputs bad = base_in(nose_up());
    bad.gyro.y = NAN;
    auto held = c.update(bad, p);
    EXPECT_TRUE(held.fault_nonfinite);
    EXPECT_FLOAT_EQ(held.cy, good.cy);
    EXPECT_FLOAT_EQ(held.cz, good.cz);
}

AP_GTEST_MAIN()
