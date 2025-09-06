/* audio_hw.h */
#ifndef AUDIO_HW_H
#define AUDIO_HW_H

#include "include/player.h"

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
