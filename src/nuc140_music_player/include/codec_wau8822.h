/* codec_wau8822.h */
#ifndef CODEC_WAU8822_H
#define CODEC_WAU8822_H
#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include "UART.h"
#include "GPIO.h"
#include "I2C.h"
#include "I2S.h"
#include "SYS.h"
#include "diskio.h"
#include "ff.h"
#include <Audio.h>
#include "adpcm4bit.h"
#include "LCD.h"
#include "Scankey.h"

#include "include/audio_hw.h"
#include "include/codec_wau8822.h"
#include "include/keys.h"
#include "include/player.h"
#include "include/playlist.h"
#include "include/ui.h"
#include "include/utils.h"
#include "include/wav_adpcm.h"
#include "include/player_state.h"

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
