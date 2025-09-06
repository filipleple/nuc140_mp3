#include "include/player.h"

/* Resume position in absolute samples */
static uint32_t g_resume_samples = 0;

/* Filesystem */
FATFS FatFs[_DRIVES];

static void handle_end_of_track(void)
{
    stop_playback();
    if (g_track_count == 0) return;
    g_track_idx = (g_track_idx + 1) % g_track_count;
    start_playback();
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

void start_playback(void)
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

/* ===== Control primitives ===== */
void stop_playback(void)
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

void pause_playback(void)
{
    DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD);
    DrvI2S_DisableTx();
    g_resume_samples = WavFile.u32SampleNumber - g_samples_left;
    g_state = STATE_PAUSED;
    ui_update_progress_bar();
    print_Line(2, "Paused");
}

void resume_playback(void)
{
    start_playback_from_sample(g_resume_samples);
}

void start_playback_from_sample(uint32_t sample_pos)
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

/* ===== Main process ===== */
int32_t player_mainloop(void)
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
