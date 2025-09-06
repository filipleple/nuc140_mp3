/* utils.h */
#ifndef UTILS_H
#define UTILS_H

#include "include/player.h"

#ifdef __cplusplus
extern "C" {
#endif

void     rough_delay(uint32_t t);
uint32_t u32_le(const void *p);
uint16_t u16_le(const void *p);
void     put_rc(FRESULT rc);

#ifdef __cplusplus
}
#endif
#endif

