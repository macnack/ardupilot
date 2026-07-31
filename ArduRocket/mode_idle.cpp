#include "ArduRocket.h"

bool ModeIdle::_enter()
{
    // IDLE is always enterable. The guard against LEAVING flight after
    // launch lives in ArduRocket::set_mode() (Task 6 of the Phase 2b plan).
    return true;
}

void ModeIdle::update()
{
    // fins neutral; pyro relay is never touched from IDLE
    rocket.set_fins(0.0f, 0.0f);
}
