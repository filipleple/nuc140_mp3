/* playlist.h */
#ifndef PLAYLIST_H
#define PLAYLIST_H
#include <stdint.h>
#include "ff.h"

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

/* Track list */
#define MAX_TRACKS 32
extern volatile char     g_tracks[MAX_TRACKS][13];   /* 8.3 names */
extern volatile int32_t  g_track_idx;         /* current index in g_tracks */
extern volatile uint32_t g_track_count;

/* File handle for the current track */
extern volatile FIL g_file;

void rebuild_playlist(void);
int  open_track_by_index(int32_t idx);

#ifdef __cplusplus
}
#endif
#endif
