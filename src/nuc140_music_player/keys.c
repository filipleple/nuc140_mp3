#include "include/keys.h"

/* ===== Keys (debounced) ===== */
int8_t get_key_press(void)
{
    enum { STABLE_N = 5 };
    static int8_t  last_raw = KEY_NONE;
    static uint8_t stable_cnt = 0;
    static uint8_t pressed_latch = 0;

    int8_t raw = ScanKey();

    if (raw == last_raw) {
        if (stable_cnt < 255) stable_cnt++;
    } else {
        stable_cnt = 0;
        last_raw   = raw;
    }

    if (stable_cnt < STABLE_N) return KEY_NONE;

    if (!pressed_latch && raw != KEY_NONE) { pressed_latch = 1; return raw; }
    if (pressed_latch && raw == KEY_NONE) { pressed_latch = 0; }

    return KEY_NONE;
}
