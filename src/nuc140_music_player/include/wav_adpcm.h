/* wav_adpcm.h */
#ifndef WAV_ADPCM_H
#define WAV_ADPCM_H
#include <stdint.h>
#include "ff.h"
#include <Audio.h>

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

/* ===== ADPCM/I2S constants ===== */
#define ADPCM_BLOCK_BYTES    256U        /* input per IMA block */
#define ADPCM_BLOCK_SAMPLES  504U        /* output samples per block */
#define PCM_BLOCK_SAMPLES    ADPCM_BLOCK_SAMPLES

/* WAV header parsed fields (AudioHeader from Nuvoton BSP) */
extern volatile AudioHeader WavFile;

int parse_and_prepare_current_file(void);                     /* uses global g_file/WavFile as w twoim kodzie */
int decode_block_into(int16_t *dest);
int prefill_from_sample(uint32_t sample_pos);

#ifdef __cplusplus
}
#endif
#endif
