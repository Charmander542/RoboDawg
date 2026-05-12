#include "app_modes.h"

#include "gait.h"
#include "state.h"

void appEnterMode(RunMode m) {
    g_state.mode = m;
    g_interpFlag = 0;
    g_previousInterpMillis = millis();
    if (m == MODE_WALK) {
        gait::reset();
    }
}
