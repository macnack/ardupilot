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
    // body rotated +theta about z_AP -> up_body_y = +sin(theta) -> corrective
    // rate about -z -> sim-roll output cz must be NEGATIVE (Phase 0-validated
    // sign convention: cz = -KP*up_y + KD*gyro_z in the PD limit)
    EXPECT_LT(out.cz, -0.01f);
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
