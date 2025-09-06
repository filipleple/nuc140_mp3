/* player_state.h */
#ifndef PLAYER_STATE_H
#define PLAYER_STATE_H
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

/* keep the same constants as in main.c */
#define ADPCM_BLOCK_BYTES    256U
#define ADPCM_BLOCK_SAMPLES  504U
#define PCM_BUFF_SAMPLES     ADPCM_BLOCK_SAMPLES

/* shared playback state — ISR touches these, so keep them volatile */
extern volatile uint32_t g_samples_left;
extern volatile uint32_t g_pcm0_pos;
extern volatile uint32_t g_pcm1_pos;
extern volatile uint8_t  g_vol_pct;             /* 0..100 */
extern volatile uint16_t g_vol_raw_cached;

typedef enum {
    STATE_STOPPED = 0,
    STATE_PLAYING = 1,
    STATE_PAUSED  = 2
} play_state_t;

/* ===== Playback state ===== */
extern volatile play_state_t g_state;


/* PCM ping/pong buffers */
extern int16_t g_pcm0[PCM_BUFF_SAMPLES];
extern int16_t g_pcm1[PCM_BUFF_SAMPLES];

/* Volume mapping: UI 0..100% -> WAU8822 speaker out 0x10..0x3D (safe range) */
enum { VOL_VU_BIT = 0x100, VOL_MUTE_BIT = 0x080, VOL_MASK = 0x07F };
#define VOL_RAW_MIN_SAFE  0x10
#define VOL_RAW_MAX_SAFE  0x3D
#define VOL_UI_STEP_PCT   4

#endif
