/* keys.h */
#ifndef KEYS_H
#define KEYS_H

#include "include/player.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Keys (NU-LB-NUC140 3x3) ===== */
#define KEY_NONE        0
#define KEY_VOLUME_UP   2
#define KEY_PREV        4
#define KEY_PLAY_PAUSE  5
#define KEY_NEXT        6
#define KEY_VOLUME_DOWN 8


int8_t get_key_press(void);

#ifdef __cplusplus
}
#endif
#endif
