#pragma once

#include <RC_Channel/RC_Channel.h>

/*
  ArduRocket has no pilot stick input -- fins are driven only by
  AP_RocketControl and modes are selected from the GCS. An RC_Channels object
  is required anyway: shared library code assumes the singleton exists. In
  particular GCS::send_textv() -> AP_CRSF_Telem::queue_message() dereferences
  rc() on every statustext, and a null singleton segfaults during
  AP_Vehicle::setup().
 */
class RC_Channel_Rocket : public RC_Channel
{
};

class RC_Channels_Rocket : public RC_Channels
{
public:

    RC_Channel_Rocket obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Rocket *channel(const uint8_t chan) override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    // no mode switch: ArduRocket modes are GCS-selected only
    int8_t flight_mode_channel_number() const override { return -1; }
};
