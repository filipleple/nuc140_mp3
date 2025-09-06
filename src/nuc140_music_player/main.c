/*
 * NUC140 + WAU8822 ADPCM player
 *
 * SD (FAT32) -> mono IMA ADPCM WAV -> decode -> I2S -> WAU8822 speaker out
 * Keypad: play/pause, next, prev, volume up/down
 * LCD: track name (row 0), progress bar (row 1), state/volume (row 2/3)
 *
 * Notes
 *  - Consistent style, dead code removed, single-responsibility helpers
 *  - Robust header parse for RIFF/WAVE with expected 'fmt'+'fact'+'data'
 *  - Double-buffered PCM with simple per-half refills (one ADPCM block = 256 B -> 504 samples)
 *  - Pause resumes at precise sample index
 *  - WAU8822 volume expressed as UI percent, mapped to safe raw range
 */

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

/* ===== Playback state ===== */
typedef enum { STATE_STOPPED = 0, STATE_PLAYING = 1, STATE_PAUSED = 2 } play_state_t;
static volatile play_state_t g_state = STATE_STOPPED;

/* ===== Keys (NU-LB-NUC140 3x3) ===== */
#define KEY_NONE        0
#define KEY_VOLUME_UP   2
#define KEY_PREV        4
#define KEY_PLAY_PAUSE  5
#define KEY_NEXT        6
#define KEY_VOLUME_DOWN 8

/* ===== ADPCM/I2S constants ===== */
#define ADPCM_BLOCK_BYTES    256U        /* input per IMA block */
#define ADPCM_BLOCK_SAMPLES  504U        /* output samples per block */
#define PCM_BLOCK_SAMPLES    ADPCM_BLOCK_SAMPLES

/* LCD */
#define LCD_COLS 16

/* Filesystem */
FATFS FatFs[_DRIVES];

/* WAV header parsed fields (AudioHeader from Nuvoton BSP) */
static AudioHeader WavFile;

/* Track list */
#define MAX_TRACKS 32
static char     g_tracks[MAX_TRACKS][13];   /* 8.3 names */
static uint32_t g_track_count = 0;
static int32_t  g_track_idx   = -1;         /* current index in g_tracks */

/* File handle for the current track */
static FIL g_file;

/* Double PCM buffers */
#define PCM_BUFF_SAMPLES  PCM_BLOCK_SAMPLES
static int16_t  g_pcm0[PCM_BUFF_SAMPLES];
static int16_t  g_pcm1[PCM_BUFF_SAMPLES];
static volatile uint32_t g_pcm0_pos = 0;   /* 0..PCM_BUFF_SAMPLES */
static volatile uint32_t g_pcm1_pos = 0;   /* 0..PCM_BUFF_SAMPLES */

/* ADPCM read scratch */
static uint8_t  g_adpcm[ADPCM_BLOCK_BYTES];

/* Remaining samples in current song; decremented by ISR */
static volatile uint32_t g_samples_left = 0;

/* Absolute byte offset where ADPCM data begins */
static uint32_t g_data_start = 0;

/* Resume position in absolute samples */
static uint32_t g_resume_samples = 0;

/* UI progress redraw throttle */
static uint8_t  g_last_bar_chars = 0xFF;    /* invalid -> force first draw */

/* ===== I2C <-> WAU8822 ===== */
static __IO uint32_t g_i2c_tx_done = 0;
static uint8_t  g_codec_i2c_addr   = 0x1A;  /* 7-bit */
static uint8_t  g_i2c_tx_data[2];
static uint8_t  g_i2c_tx_idx = 0;

/* Volume mapping: UI 0..100% -> WAU8822 speaker out 0x10..0x3D (safe range) */
enum { VOL_VU_BIT = 0x100, VOL_MUTE_BIT = 0x080, VOL_MASK = 0x07F };
#define VOL_RAW_MIN_SAFE  0x10
#define VOL_RAW_MAX_SAFE  0x3D
#define VOL_UI_STEP_PCT   4
static volatile uint8_t  g_vol_pct = 50;             /* 0..100 */
static volatile uint16_t g_vol_raw_cached = VOL_VU_BIT | VOL_RAW_MIN_SAFE;

/* ===== Prototypes ===== */
static void i2c0_callback_tx(uint32_t status);
static void i2c_write_wau8822(uint8_t reg, uint16_t data);
static void wau8822_setup(void);
static void init_codec_i2s(void);
static void start_playback(void);
static void stop_playback(void);
static void pause_playback(void);
static void resume_playback(void);
static void start_playback_from_sample(uint32_t sample_pos);
static int  open_track_by_index(int32_t idx);
static int  parse_and_prepare_current_file(void);
static void rebuild_playlist(void);
static void handle_end_of_track(void);
static void ui_update_progress_bar(void);
static void ui_progress_reset(void);
static void lcd_print_track_name(void);
static int  decode_block_into(int16_t *dest);
static int  prefill_from_sample(uint32_t sample_pos);
static int8_t get_key_press(void);
static void poll_keys_and_dispatch(void);
void Tx_thresholdCallbackfn0(uint32_t status);
void Tx_thresholdCallbackfn1(uint32_t status);

/* ===== Small utils ===== */
static void rough_delay(uint32_t t)
{
    volatile uint32_t d = t;
    while (d-- > 0) { /* busy wait */ }
}

static uint32_t u32_le(const void *p)
{
    const uint8_t *b = (const uint8_t *)p;
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static uint16_t u16_le(const void *p)
{
    const uint8_t *b = (const uint8_t *)p;
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

/* ===== I2C to WAU8822 ===== */
static void i2c0_callback_tx(uint32_t status)
{
    switch (status) {
    case 0x08: /* START tx */
        DrvI2C_WriteData(I2C_PORT0, (g_codec_i2c_addr << 1));
        DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
        break;
    case 0x18: /* SLA+W ACK */
        DrvI2C_WriteData(I2C_PORT0, g_i2c_tx_data[g_i2c_tx_idx++]);
        DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
        break;
    case 0x20: /* SLA+W NACK */
        DrvI2C_Ctrl(I2C_PORT0, 1, 1, 1, 0);
        break;
    case 0x28: /* DATA ACK */
        if (g_i2c_tx_idx < 2) {
            DrvI2C_WriteData(I2C_PORT0, g_i2c_tx_data[g_i2c_tx_idx++]);
            DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
        } else {
            DrvI2C_Ctrl(I2C_PORT0, 0, 1, 1, 0);
            g_i2c_tx_done = 1;
        }
        break;
    default:
        while (1) { /* unexpected state */ }
    }
}

static void i2c_write_wau8822(uint8_t reg, uint16_t data)
{
    g_i2c_tx_idx = 0;
    g_i2c_tx_done = 0;
    g_i2c_tx_data[0] = (uint8_t)((reg << 1) | (data >> 8));
    g_i2c_tx_data[1] = (uint8_t)(data & 0xFF);

    DrvI2C_InstallCallback(I2C_PORT0, I2CFUNC, i2c0_callback_tx);
    DrvI2C_Ctrl(I2C_PORT0, 1, 0, 0, 0); /* START */
    while (!g_i2c_tx_done) { /* wait */ }
}

static inline uint16_t vol_pct_to_raw(uint8_t pct)
{
    if (pct > 100) pct = 100;
    const uint16_t span = (uint16_t)(VOL_RAW_MAX_SAFE - VOL_RAW_MIN_SAFE);
    uint16_t raw = VOL_RAW_MIN_SAFE + (uint16_t)((pct * (uint32_t)span + 50U) / 100U);
    if (raw < VOL_RAW_MIN_SAFE) raw = VOL_RAW_MIN_SAFE;
    if (raw > VOL_RAW_MAX_SAFE) raw = VOL_RAW_MAX_SAFE;
    return (uint16_t)(raw & VOL_MASK);
}

static inline void codec_apply_vol_pct(uint8_t pct)
{
    uint16_t raw = vol_pct_to_raw(pct);
    g_vol_pct = pct;
    g_vol_raw_cached = (uint16_t)(raw | VOL_VU_BIT);
    i2c_write_wau8822(54, g_vol_raw_cached); /* LSPKOUT */
    i2c_write_wau8822(55, g_vol_raw_cached); /* RSPKOUT */
}

static inline void volume_show_ui(void)
{
    char line[16];
    snprintf(line, sizeof line, "Vol:%3u%%", (unsigned)g_vol_pct);
    print_Line(3, line);
}

static inline void volume_change_pct(int delta_pct)
{
    int p = (int)g_vol_pct + delta_pct;
    if (p < 0)   p = 0;
    if (p > 100) p = 100;
    codec_apply_vol_pct((uint8_t)p);
    volume_show_ui();
}

static void wau8822_setup(void)
{
    i2c_write_wau8822(0,  0x000); /* Reset */
    rough_delay(0x200);

    i2c_write_wau8822(1,  0x02F);
    i2c_write_wau8822(2,  0x1B3); /* HP, ADC mix/boost, ADC */
    i2c_write_wau8822(3,  0x07F); /* main mixer, DAC */
    i2c_write_wau8822(4,  0x010); /* 16-bit, I2S, stereo */
    i2c_write_wau8822(5,  0x000); /* no companding/loopback */
    i2c_write_wau8822(6,  0x1ED); /* 8 kHz */
    i2c_write_wau8822(7,  0x00A); /* 8 kHz coeffs */
    i2c_write_wau8822(10, 0x008); /* DAC softmute off, 128x */
    i2c_write_wau8822(14, 0x108); /* ADC HPF off, 128x */
    i2c_write_wau8822(15, 0x1EF); /* ADC L vol */
    i2c_write_wau8822(16, 0x1EF); /* ADC R vol */
    i2c_write_wau8822(43, 0x010);
    i2c_write_wau8822(44, 0x000);
    i2c_write_wau8822(45, 0x150);
    i2c_write_wau8822(46, 0x150);
    i2c_write_wau8822(47, 0x007);
    i2c_write_wau8822(48, 0x007);
    i2c_write_wau8822(49, 0x047);
    i2c_write_wau8822(50, 0x001); /* DAC -> LMIX */
    i2c_write_wau8822(51, 0x000); /* DAC -> RMIX */

    codec_apply_vol_pct(g_vol_pct); /* ensures VU bit write and UI mapping */

    DrvGPIO_Open(E_GPE, 14, E_IO_OUTPUT);
    DrvGPIO_ClrBit(E_GPE, 14);
}

/* ===== I2S ISR (Tx threshold) ===== */
void Tx_thresholdCallbackfn0(uint32_t status)
{
    /* feed up to 4 samples per interrupt from PCM0 */
    for (uint32_t i = 0; i < 4; i++) {
        if (g_samples_left == 0) { DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD); DrvI2S_DisableTx(); return; }
        if (g_pcm0_pos < PCM_BUFF_SAMPLES) {
            int32_t s = ((int32_t)g_pcm0[g_pcm0_pos++]) << 16;
            _DRVI2S_WRITE_TX_FIFO(s);
            g_samples_left--;
        } else {
            _DRVI2S_WRITE_TX_FIFO(0);
        }
    }
    if (g_pcm0_pos >= PCM_BUFF_SAMPLES) {
        DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn1);
    }
}

void Tx_thresholdCallbackfn1(uint32_t status)
{
    /* feed up to 4 samples per interrupt from PCM1 */
    for (uint32_t i = 0; i < 4; i++) {
        if (g_samples_left == 0) { DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD); DrvI2S_DisableTx(); return; }
        if (g_pcm1_pos < PCM_BUFF_SAMPLES) {
            int32_t s = ((int32_t)g_pcm1[g_pcm1_pos++]) << 16;
            _DRVI2S_WRITE_TX_FIFO(s);
            g_samples_left--;
        } else {
            _DRVI2S_WRITE_TX_FIFO(0);
        }
    }
    if (g_pcm1_pos >= PCM_BUFF_SAMPLES) {
        DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn0);
    }
}

/* ===== Filesystem helpers ===== */
static void put_rc(FRESULT rc)
{
    static const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
    UINT i;
    const TCHAR *q = p;
    for (i = 0; (i != (UINT)rc) && *q; i++) { while (*q++) { } }
    printf("rc=%u FR_%s\n", (UINT)rc, q);
}

/* ===== Codec + I2S init (8 kHz, 16-bit, I2S, slave) ===== */
static void init_codec_i2s(void)
{
    /* Tri-state FS and BCLK while configuring codec */
    DrvGPIO_Open(E_GPC, 0, E_IO_OPENDRAIN); /* FS */
    DrvGPIO_Open(E_GPC, 1, E_IO_OPENDRAIN); /* BCLK */
    DrvGPIO_SetBit(E_GPC, 0);
    DrvGPIO_SetBit(E_GPC, 1);

    /* I2C0 @ 100 kHz */
    DrvGPIO_InitFunction(E_FUNC_I2C0);
    DrvI2C_Open(I2C_PORT0, 100000);
    DrvI2C_EnableInt(I2C_PORT0);

    /* Configure codec */
    wau8822_setup();

    /* I2S setup */
    S_DRVI2S_DATA_T st;
    st.u32SampleRate      = 8000;
    st.u8WordWidth        = DRVI2S_DATABIT_16;
    st.u8AudioFormat      = DRVI2S_STEREO;
    st.u8DataFormat       = DRVI2S_FORMAT_I2S;
    st.u8Mode             = DRVI2S_MODE_SLAVE;
    st.u8TxFIFOThreshold  = DRVI2S_FIFO_LEVEL_WORD_4;
    st.u8RxFIFOThreshold  = DRVI2S_FIFO_LEVEL_WORD_4;
    DrvI2S_SelectClockSource(0);
    DrvI2S_Open(&st);
    DrvGPIO_InitFunction(E_FUNC_I2S);
    DrvI2S_SetMCLKFreq(12000000);
    DrvI2S_EnableMCLK();
}

/* ===== WAV parsing for IMA ADPCM mono ===== */
static int parse_and_prepare_current_file(void)
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

/* ===== Simple .WAV filter ===== */
static int is_wav_adpcm(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (!dot) return 0;
    return strcasecmp(dot, ".WAV") == 0;
}

static void rebuild_playlist(void)
{
    DIR dir;
    FILINFO fi;
    g_track_count = 0;

    if (f_opendir(&dir, "\\") != FR_OK) return;
    for (;;) {
        if (f_readdir(&dir, &fi) != FR_OK) break;
        if (fi.fname[0] == 0) break;
        if (fi.fattrib & AM_DIR) continue;
        if (!is_wav_adpcm(fi.fname)) continue;
        if (g_track_count < MAX_TRACKS) {
            strncpy(g_tracks[g_track_count], fi.fname, 12);
            g_tracks[g_track_count][12] = '\0';
            g_track_count++;
        }
    }
    /* Some BSPs omit f_closedir; skip for compatibility */
}

/* ===== Decode helpers ===== */
static int decode_block_into(int16_t *dest)
{
    UINT br = 0;
    if (f_read(&g_file, g_adpcm, ADPCM_BLOCK_BYTES, &br) != FR_OK) return 0;
    if (br < ADPCM_BLOCK_BYTES) return 0; /* require full block */
    AdpcmDec4(g_adpcm, dest, PCM_BLOCK_SAMPLES);
    return 1;
}

static int prefill_from_sample(uint32_t sample_pos)
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

/* ===== UI helpers ===== */
static void ui_progress_reset(void)
{
    g_last_bar_chars = 0xFF;
}

static void lcd_print_track_name(void)
{
    if (g_track_count == 0 || g_track_idx < 0) {
        print_Line(0, "No WAVs        ");
        return;
    }
    char name[13];
    strncpy(name, g_tracks[g_track_idx], 12);
    name[12] = '\0';
    char *dot = strrchr(name, '.');
    if (dot) *dot = '\0';

    char line[17];
    snprintf(line, sizeof line, "%02u %-13.13s", (unsigned)(g_track_idx + 1), name);
    print_Line(0, line);
}

static void ui_update_progress_bar(void)
{
    if (!WavFile.u32SampleNumber) return;

    uint32_t total  = WavFile.u32SampleNumber;
    uint32_t left   = g_samples_left; /* snapshot */
    uint32_t played = (left <= total) ? (total - left) : 0;

    uint8_t filled = (uint8_t)(((uint64_t)played * LCD_COLS) / total);
    if (filled > LCD_COLS) filled = LCD_COLS;

    if (filled == g_last_bar_chars) return;

    char line[LCD_COLS + 1];
    for (uint8_t i = 0; i < LCD_COLS; i++) line[i] = (i < filled) ? '=' : ' ';
    line[LCD_COLS] = '\0';
    print_Line(1, line);
    g_last_bar_chars = filled;
}

/* ===== Keys (debounced) ===== */
static int8_t get_key_press(void)
{
    enum { STABLE_N = 5 };
    static int8_t  last_raw = KEY_NONE;
    static uint8_t stable_cnt = 0;
    static uint8_t pressed_latch = 0;

    int8_t raw = ScanKey();

    if (raw == last_raw) {
        if (stable_cnt < 255) stable_cnt++;
    } else {
        stable_cnt = 0;
        last_raw   = raw;
    }

    if (stable_cnt < STABLE_N) return KEY_NONE;

    if (!pressed_latch && raw != KEY_NONE) { pressed_latch = 1; return raw; }
    if (pressed_latch && raw == KEY_NONE) { pressed_latch = 0; }

    return KEY_NONE;
}

static void poll_keys_and_dispatch(void)
{
    int8_t k = get_key_press();
    if (k == KEY_NONE) return;

    switch (k) {
    case KEY_VOLUME_UP:   volume_change_pct(+VOL_UI_STEP_PCT); break;
    case KEY_VOLUME_DOWN: volume_change_pct(-VOL_UI_STEP_PCT); break;
    case KEY_PLAY_PAUSE:
        if      (g_state == STATE_PLAYING) pause_playback();
        else if (g_state == STATE_PAUSED)  resume_playback();
        else                               start_playback();
        break;
    case KEY_NEXT:
        stop_playback();
        if (g_track_count) { g_track_idx = (g_track_idx + 1) % g_track_count; start_playback(); }
        break;
    case KEY_PREV:
        stop_playback();
        if (g_track_count) { g_track_idx = (g_track_idx + g_track_count - 1) % g_track_count; start_playback(); }
        break;
    default: break;
    }
}

/* ===== Control primitives ===== */
static void stop_playback(void)
{
    DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD);
    DrvI2S_DisableTx();
    g_samples_left = 0;
    f_close(&g_file);
    g_state = STATE_STOPPED;
    print_Line(2, "Stopped");
    ui_progress_reset();
    print_Line(1, "                ");
}

static void pause_playback(void)
{
    DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD);
    DrvI2S_DisableTx();
    g_resume_samples = WavFile.u32SampleNumber - g_samples_left;
    g_state = STATE_PAUSED;
    ui_update_progress_bar();
    print_Line(2, "Paused");
}

static void start_playback_from_sample(uint32_t sample_pos)
{
    if (g_track_count == 0) { print_Line(2, "No WAVs"); return; }
    if (g_track_idx < 0 || g_track_idx >= (int32_t)g_track_count) g_track_idx = 0;

    f_close(&g_file);
    if (!open_track_by_index(g_track_idx) || !parse_and_prepare_current_file()) {
        print_Line(2, "Open/parse");
        g_state = STATE_STOPPED; return;
    }

    if (sample_pos > WavFile.u32SampleNumber) sample_pos = WavFile.u32SampleNumber;

    g_pcm0_pos = g_pcm1_pos = 0;
    init_codec_i2s();
    lcd_print_track_name();
    ui_progress_reset();
    ui_update_progress_bar();
    codec_apply_vol_pct(g_vol_pct);

    g_samples_left = WavFile.u32SampleNumber - sample_pos;
    if (!prefill_from_sample(sample_pos)) { handle_end_of_track(); return; }

    DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn0);
    DrvI2S_EnableTx();

    g_state = STATE_PLAYING;
    print_Line(2, "Playing");
}

static void resume_playback(void)
{
    start_playback_from_sample(g_resume_samples);
}

static void start_playback(void)
{
    if (g_track_count == 0) { print_Line(2, "No WAVs"); return; }
    if (g_track_idx < 0 || g_track_idx >= (int32_t)g_track_count) g_track_idx = 0;

    if (!open_track_by_index(g_track_idx)) {
        print_Line(2, "Open fail"); g_state = STATE_STOPPED; return;
    }
    if (!parse_and_prepare_current_file()) {
        f_close(&g_file); print_Line(2, "Bad WAV"); g_state = STATE_STOPPED; return;
    }

    g_pcm0_pos = g_pcm1_pos = 0;
    init_codec_i2s();
    lcd_print_track_name();
    ui_progress_reset();
    ui_update_progress_bar();
    codec_apply_vol_pct(g_vol_pct);

    /* Prime both halves */
    if (!decode_block_into(g_pcm0)) { handle_end_of_track(); return; }
    g_pcm0_pos = 0;
    if (!decode_block_into(g_pcm1)) { /* last block case */ g_pcm1_pos = PCM_BUFF_SAMPLES; }

    DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn0);
    DrvI2S_EnableTx();

    g_state = STATE_PLAYING;
    print_Line(2, "Playing");
}

static void handle_end_of_track(void)
{
    stop_playback();
    if (g_track_count == 0) return;
    g_track_idx = (g_track_idx + 1) % g_track_count;
    start_playback();
}

/* ===== Track open ===== */
static int open_track_by_index(int32_t idx)
{
    if (idx < 0 || idx >= (int32_t)g_track_count) return 0;
    return f_open(&g_file, (TCHAR*)g_tracks[idx], FA_READ) == FR_OK;
}

/* ===== Main process ===== */
static int32_t player_mainloop(void)
{
    FRESULT res;

    res = (FRESULT)disk_initialize(0);
    if (res) { put_rc(res); printf("SD init failed\n"); }

    res = f_mount(0, &FatFs[0]);
    if (res) { put_rc(res); printf("mount failed\n"); }

    rebuild_playlist();
    if (g_track_count == 0) {
        lcd_print_track_name();
        print_Line(2, "Stopped");
    } else {
        g_track_idx = 0;
        start_playback();
    }

    print_Line(1, "16bit Mono 8KHz");

    for (;;) {
        poll_keys_and_dispatch();
        ui_update_progress_bar();

        if (g_state != STATE_PLAYING) { DrvSYS_Delay(1000); continue; }

        /* refill whichever half is empty; one ADPCM block per refill */
        if (g_pcm0_pos >= PCM_BUFF_SAMPLES) {
            if (!decode_block_into(g_pcm0)) { handle_end_of_track(); continue; }
            g_pcm0_pos = 0;
        }
        if (g_pcm1_pos >= PCM_BUFF_SAMPLES) {
            if (!decode_block_into(g_pcm1)) { handle_end_of_track(); continue; }
            g_pcm1_pos = 0;
        }

        if (g_samples_left == 0) { handle_end_of_track(); continue; }
    }
}

/* ===== Entry point ===== */
int32_t main(void)
{
    STR_UART_T sParam;

    UNLOCKREG();
    DrvSYS_Open(50000000);
    LOCKREG();

    sParam.u32BaudRate       = 115200;
    sParam.u8cDataBits       = DRVUART_DATABITS_8;
    sParam.u8cStopBits       = DRVUART_STOPBITS_1;
    sParam.u8cParity         = DRVUART_PARITY_NONE;
    sParam.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
    DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC, 0);
    DrvUART_Open(UART_PORT0, &sParam);
    DrvGPIO_InitFunction(E_FUNC_UART0);

    init_LCD();
    clear_LCD();
    OpenKeyPad();

    printf("\n\nNUC100 Series ADPCM\n");
    printf("Insert SDcard with mono IMA-ADPCM WAVs\n");

    return player_mainloop();
}

