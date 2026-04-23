#include "mode.h"

#include "Tracker.h"

void ModeAuto::update()
{
    if (tracker.vehicle.location_valid) {
        update_auto();
    } else if ((tracker.g.auto_opts.get() & (1 << 0)) != 0) {
        update_scan();
    }
    // якщо ціль невалідна і біт scan не включений:
    // нічого не робимо, серви лишаються на останній команді
}
