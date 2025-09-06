#include "include/wav_adpcm.h"

/* ADPCM read scratch */
static uint8_t  g_adpcm[ADPCM_BLOCK_BYTES];

/* Absolute byte offset where ADPCM data begins */
static uint32_t g_data_start = 0;

/* ===== WAV parsing for IMA ADPCM mono ===== */
int parse_and_prepare_current_file(void)
{
    UINT br;
    uint8_t hdr[64];

    if (f_lseek(&g_file, 0) != FR_OK) return 0;
    if (f_read(&g_file, hdr, sizeof hdr, &br) != FR_OK || br < 44) return 0;

    if (memcmp(hdr + 0, "RIFF", 4) != 0) return 0;
    if (memcmp(hdr + 8, "WAVE", 4) != 0) return 0;

    /* Expect "fmt " at 12, subchunk1 size at 16 */
    if (memcmp(hdr + 12, "fmt ", 4) != 0) return 0;

    uint32_t sub1 = u32_le(hdr + 16);
    WavFile.u16AudioFormat   = u16_le(hdr + 20);
    WavFile.u16NumChannels   = u16_le(hdr + 22);
    WavFile.u32SampleRate    = u32_le(hdr + 24);
    WavFile.u32ByteRate      = u32_le(hdr + 28);
    WavFile.u16BlockAlign    = u16_le(hdr + 32);
    WavFile.u16BitsPerSample = u16_le(hdr + 34);

    if (WavFile.u16AudioFormat != 0x0011) return 0; /* IMA ADPCM */
    if (WavFile.u16NumChannels != 1)       return 0; /* mono only */

    /* fact chunk follows fmt for ADPCM */
    uint32_t fact_off = 20 + 20; /* RIFF(12) + fmt(8) + fmt body(20) -> 40 */
    if (memcmp(hdr + fact_off, "fact", 4) != 0) return 0;
    WavFile.u32Subchunk1Size = sub1;
    WavFile.u32SampleNumber  = u32_le(hdr + fact_off + 8);

    /* data chunk header after 'fact' (12 bytes) */
    uint32_t data_off = fact_off + 12;
    if (memcmp(hdr + data_off, "data", 4) != 0) return 0;
    WavFile.u32Subchunk2Size = u32_le(hdr + data_off + 4);
    WavFile.u8HeaderStatus   = 1;

    g_samples_left = WavFile.u32SampleNumber;
    g_data_start   = data_off + 8; /* start of ADPCM bytes */

    /* Seek to start of ADPCM */
    if (f_lseek(&g_file, g_data_start) != FR_OK) return 0;

    return 1;
}

/* ===== Decode helpers ===== */
int decode_block_into(int16_t *dest)
{
    UINT br = 0;
    if (f_read(&g_file, g_adpcm, ADPCM_BLOCK_BYTES, &br) != FR_OK) return 0;
    if (br < ADPCM_BLOCK_BYTES) return 0; /* require full block */
    AdpcmDec4(g_adpcm, dest, PCM_BLOCK_SAMPLES);
    return 1;
}

int prefill_from_sample(uint32_t sample_pos)
{
    uint32_t blk = sample_pos / ADPCM_BLOCK_SAMPLES;
    uint32_t ofs = sample_pos % ADPCM_BLOCK_SAMPLES;

    if (f_lseek(&g_file, g_data_start + (DWORD)blk * ADPCM_BLOCK_BYTES) != FR_OK) return 0;

    if (!decode_block_into(g_pcm0)) return 0;
    g_pcm0_pos = ofs; /* start inside block */

    if (decode_block_into(g_pcm1))
        g_pcm1_pos = 0;
    else
        g_pcm1_pos = PCM_BUFF_SAMPLES; /* mark empty */

    return 1;
}
