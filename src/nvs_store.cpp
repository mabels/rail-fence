#include "globals.h"
#include <Preferences.h>

static Preferences g_prefs;

void nvs_save_position(int32_t pos) {
    g_prefs.begin(NVS_NS, false);
    g_prefs.putInt(NVS_POS, pos);
    g_prefs.end();
}

int32_t nvs_load_position() {
    g_prefs.begin(NVS_NS, true);
    const int32_t pos = g_prefs.getInt(NVS_POS, 0);
    g_prefs.end();
    return pos;
}

void nvs_save_transport(Transport t) {
    g_prefs.begin(NVS_NS, false);
    g_prefs.putUChar(NVS_TRANSPORT, static_cast<uint8_t>(t));
    g_prefs.end();
}

Transport nvs_load_transport() {
    g_prefs.begin(NVS_NS, true);
    const uint8_t v = g_prefs.getUChar(NVS_TRANSPORT, 0);
    g_prefs.end();
    return static_cast<Transport>(v);
}
