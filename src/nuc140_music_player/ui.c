#include "include/ui.h"

/* LCD */
#define LCD_COLS 16

/* UI progress redraw throttle */
static uint8_t  g_last_bar_chars = 0xFF;    /* invalid -> force first draw */

void ui_progress_reset(void)
{
    g_last_bar_chars = 0xFF;
}

void ui_update_progress_bar(void)
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

void lcd_print_track_name(void)
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

inline void volume_show_ui(void)
{
    char line[16];
    snprintf(line, sizeof line, "Vol:%3u%%", (unsigned)g_vol_pct);
    print_Line(3, line);
}
