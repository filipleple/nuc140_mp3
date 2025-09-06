#include "include/playlist.h"

volatile uint32_t g_track_count = 0;
volatile int32_t  g_track_idx = 0;
volatile char     g_tracks[MAX_TRACKS][13];   /* 8.3 names */

/* ===== Simple .WAV filter ===== */
static int is_wav_adpcm(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (!dot) return 0;
    return strcasecmp(dot, ".WAV") == 0;
}

void rebuild_playlist(void)
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

int open_track_by_index(int32_t idx)
{
    if (idx < 0 || idx >= (int32_t)g_track_count) return 0;
    return f_open(&g_file, (TCHAR*)g_tracks[idx], FA_READ) == FR_OK;
}
