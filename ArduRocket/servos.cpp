#include "ArduRocket.h"

/*
  Flush commanded PWM to the outputs.

  SRV_Channels is a generic library, but this pump is vehicle-owned:
  AP_Vehicle does not provide it (AP_Vehicle.cpp only calls
  SRV_Channels::zero_rc_outputs() on the failsafe path). Mirrors the tail of
  Plane::set_servos() (ArduPlane/servos.cpp).
 */
void ArduRocket::set_servos()
{
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    // push() is a non-static member; g.servo_channels is our SRV_Channels
    // instance (registered as the SERVOn_ parameter group)
    g.servo_channels.push();
}
