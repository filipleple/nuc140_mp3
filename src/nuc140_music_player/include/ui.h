/* ui.h */
#ifndef UI_H
#define UI_H

#include "include/player.h"

#ifdef __cplusplus
extern "C" {
#endif

void ui_progress_reset(void);
void ui_update_progress_bar(void);
void lcd_print_track_name(void);
void volume_show_ui(void);

#ifdef __cplusplus
}
#endif
#endif
