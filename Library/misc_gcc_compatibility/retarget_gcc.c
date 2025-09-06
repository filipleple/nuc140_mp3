// Minimal GCC/newlib retarget shim for NUC140
#include <stddef.h>
#include <stdio.h>

// If your BSP provides this, we'll use it; otherwise we just drop chars.
extern int SendChar_ToUART(int ch) __attribute__((weak));

int fputc(int ch, FILE *f) {
    (void)f;
    if (SendChar_ToUART) SendChar_ToUART(ch);
    return ch;
}

int fgetc(FILE *f) {
    (void)f;
    return -1; // no input
}

#ifdef ferror
#undef ferror
#endif
int ferror(FILE *f) {
    (void)f;
    return 0;
}

// Make printf work without semihosting; dump to UART if available.
int _write(int fd, const void *buf, size_t count) {
    (void)fd;
    if (SendChar_ToUART) {
        const char *p = (const char *)buf;
        for (size_t i = 0; i < count; i++) SendChar_ToUART(p[i]);
    }
    return (int)count; // pretend all written
}

void _sys_exit(int rc) {
    (void)rc;
    for (;;) { __asm__ volatile ("wfi"); }
}
