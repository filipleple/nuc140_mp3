/* player_state.c */
#include "player_state.h"

volatile uint32_t g_samples_left = 0;
volatile uint32_t g_pcm0_pos = 0;
volatile uint32_t g_pcm1_pos = 0;
volatile uint8_t g_vol_pct = 50;   // 0..100 start
volatile uint16_t g_vol_raw_cached = VOL_VU_BIT | VOL_RAW_MIN_SAFE;

int16_t g_pcm0[PCM_BUFF_SAMPLES];
int16_t g_pcm1[PCM_BUFF_SAMPLES];

volatile play_state_t g_state = STATE_STOPPED;

/* file/header globals */
FIL         g_file;
volatile AudioHeader WavFile;
uint32_t    g_data_start = 0;
