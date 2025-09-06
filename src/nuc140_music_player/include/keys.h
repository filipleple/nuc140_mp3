/* keys.h */
#ifndef KEYS_H
#define KEYS_H
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
