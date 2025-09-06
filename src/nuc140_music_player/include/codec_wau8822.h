/* codec_wau8822.h */
#ifndef CODEC_WAU8822_H
#define CODEC_WAU8822_H

#include "include/player.h"

#ifdef __cplusplus
extern "C" {
#endif

void wau8822_setup(void);
void codec_apply_vol_pct(uint8_t pct);
void volume_change_pct(int delta_pct);

#ifdef __cplusplus
}
#endif
#endif
