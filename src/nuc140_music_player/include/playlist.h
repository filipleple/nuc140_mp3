/* playlist.h */
#ifndef PLAYLIST_H
#define PLAYLIST_H

#include "ff.h"
#include "include/player.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Track list */
#define MAX_TRACKS 32
extern volatile char     g_tracks[MAX_TRACKS][13];   /* 8.3 names */
extern volatile int32_t  g_track_idx;         /* current index in g_tracks */
extern volatile uint32_t g_track_count;

/* File handle for the current track */
extern FIL g_file;

void rebuild_playlist(void);
int  open_track_by_index(int32_t idx);

#ifdef __cplusplus
}
#endif
#endif
