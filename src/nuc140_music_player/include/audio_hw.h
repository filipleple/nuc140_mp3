/* audio_hw.h */
#ifndef AUDIO_HW_H
#define AUDIO_HW_H

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

void init_codec_i2s(void);
void Tx_thresholdCallbackfn0(uint32_t status);
void Tx_thresholdCallbackfn1(uint32_t status);

#ifdef __cplusplus
}
#endif
#endif
