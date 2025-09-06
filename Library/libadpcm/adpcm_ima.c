#include <stdint.h>
#include <stddef.h>
#include "adpcm4bit.h"

typedef struct {
    int16_t predictor;
    int8_t  step_index;
} ima_state_t;

static const int step_table[89] = {
     7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107,118,130,143,
    157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,
    724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,
    2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,
    9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,
    27086,29794,32767
};
static const int8_t index_table[16] =
    { -1,-1,-1,-1, 2, 4, 6, 8, -1,-1,-1,-1, 2, 4, 6, 8 };

static inline int16_t clamp16(int v) {
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static inline void decode_nibble(uint8_t code, ima_state_t *st, int16_t *out)
{
    int step = step_table[st->step_index];
    int diff = step >> 3;
    if (code & 1) diff += step >> 2;
    if (code & 2) diff += step >> 1;
    if (code & 4) diff += step;
    if (code & 8) diff = -diff;

    int pred = st->predictor + diff;
    st->predictor = clamp16(pred);

    st->step_index += index_table[code & 0x0F];
    if (st->step_index < 0) st->step_index = 0;
    if (st->step_index > 88) st->step_index = 88;

    *out = st->predictor;
}

static inline uint8_t encode_sample(int16_t sample, ima_state_t *st)
{
    int step = step_table[st->step_index];
    int diff = sample - st->predictor;
    uint8_t code = 0;
    if (diff < 0) { code |= 8; diff = -diff; }

    int delta = step >> 3;
    if (diff >= step) { code |= 4; diff -= step; delta += step; }
    if (diff >= (step >> 1)) { code |= 2; diff -= (step >> 1); delta += (step >> 1); }
    if (diff >= (step >> 2)) { code |= 1; delta += (step >> 2); }

    if (code & 8) st->predictor = clamp16(st->predictor - delta);
    else          st->predictor = clamp16(st->predictor + delta);

    st->step_index += index_table[code];
    if (st->step_index < 0) st->step_index = 0;
    if (st->step_index > 88) st->step_index = 88;

    return code & 0x0F;
}

/* Encoder
 * ibuff: n 16-bit PCM samples (n % 4 == 0)
 * st: persistent state across calls; packed as (index<<16) | (uint16_t)predictor
 * obuff: output buffer length must be n/2 + 4 bytes
 * Layout: [pred_low][pred_high][index][0] then n/2 bytes of 2 nibbles per byte (low nibble first)
 */
void AdpcmEnc4(short *ibuff, int n, int *st, unsigned char *obuff)
{
    ima_state_t S;
    if (st && *st != 0) {
        S.predictor = (int16_t)(*st & 0xFFFF);
        S.step_index = (int8_t)((*st >> 16) & 0xFF);
        if (S.step_index < 0) S.step_index = 0;
        if (S.step_index > 88) S.step_index = 88;
    } else {
        S.predictor = 0;
        S.step_index = 0;
    }

    obuff[0] = (uint8_t)(S.predictor & 0xFF);
    obuff[1] = (uint8_t)((S.predictor >> 8) & 0xFF);
    obuff[2] = (uint8_t)S.step_index;
    obuff[3] = 0;

    unsigned char *p = obuff + 4;
    for (int i = 0; i < n; i += 2) {
        uint8_t lo = encode_sample(ibuff[i],   &S);
        uint8_t hi = encode_sample(ibuff[i+1], &S);
        *p++ = (unsigned char)((hi << 4) | lo);
    }

    if (st) *st = ((int)S.step_index << 16) | ((uint16_t)S.predictor);
}

/* Decoder
 * ibuff: input bitstream with 4-byte header as written by AdpcmEnc4
 * obuff: output PCM16, n samples
 * n: number of output samples (must match the encoder's n)
 */
void AdpcmDec4(unsigned char *ibuff, short *obuff, int n)
{
    ima_state_t S;
    S.predictor  = (int16_t)( (uint16_t)ibuff[0] | ((uint16_t)ibuff[1] << 8) );
    S.step_index = (int8_t)ibuff[2];
    if (S.step_index < 0) S.step_index = 0;
    if (S.step_index > 88) S.step_index = 88;

    const unsigned char *p = ibuff + 4;
    for (int i = 0; i < n; i += 2) {
        uint8_t b = *p++;
        int16_t s;
        decode_nibble(b & 0x0F, &S, &s);
        obuff[i] = s;
        if (i + 1 < n) {
            decode_nibble((b >> 4) & 0x0F, &S, &s);
            obuff[i+1] = s;
        }
    }
}
